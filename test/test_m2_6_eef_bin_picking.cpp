// vNext M2.6 AFTER (ADR-0022), M5: the CANONICAL bin-picking loop ticked as XML, P12. The tree
// (test/data/m2_6_contract/bin_picking_canonical.xml) is a BYTE COPY of what TSA generates for its
// pinned canonical bin-picking project (TSA test/data/m2_6_eef/bin_picking_canonical.xml, asserted
// equal to its generation by TSA's test_m2_6_eef_bin_picking): the runtime proves the generated
// gate, not a hand-written one.
//
//   every EEF FAULT, a failure while an object is held, and a silent detector end the run with
//   FAILURE -- never "bin empty": the stall branch's recovery channel is never enabled and its
//   "bin empty" Log is never emitted; in contrast a VERDICT on every candidate (NO_OBJECT), a fresh
//   detection with no match, and a non-EEF failure with nothing held reach the stall branch, as
//   the legacy tree does.
//
// REAL nodes: DetectObject (a fake detector publishing Detection3DArray, a TF buffer in the
// context), SetWaypointFromDetection, SetWaypointRelative, SetAttachedCollisionCheck (against a
// stub SetBool server at the scene manager's service name), CloseGripper/OpenGripper/VerifyGrasp
// through the REAL registry against a FakeEefAdapter, Script/Retry/Fallback/Repeat/Sleep, Log
// (captured through rcutils' output handler), EnableProcess/DisableProcess (a recording
// ProcessController). The ONE stand-in, labelled: MoveWaypoint (no MoveIt here) records its
// waypoint and can be scripted to fail or to drop the object -- the gate does not depend on how a
// motion is planned. Domain 97, localhost only, graph-guarded.
#include <rcutils/logging.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstring>
#include <optional>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "m2_6/capture26.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "m2_6/eef_after_runner.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace
{
namespace eef = trainit::eef;
using m26::after::CanonicalRunner;
using m26::after::FakeEefAdapter;
using m26::after::SetBoolReply;
using m26::after::SiteScript;
using m26::after::State;
using m26::milliseconds;

// --- the "[BT] ..." lines the tree logs, captured through rcutils' output handler -----------------
std::mutex g_log_m;
std::vector<std::string> g_bt_lines;
rcutils_logging_output_handler_t g_previous = nullptr;

void tap(const rcutils_log_location_t* location, int severity, const char* name, rcutils_time_point_value_t ts,
         const char* format, va_list* args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[2048];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  if (std::strstr(buf, "[BT] ") != nullptr)
  {
    std::lock_guard<std::mutex> g(g_log_m);
    g_bt_lines.emplace_back(buf);
  }
  if (g_previous) g_previous(location, severity, name, ts, format, args);
}

int btLines(const std::string& text)
{
  std::lock_guard<std::mutex> g(g_log_m);
  int n = 0;
  for (const auto& l : g_bt_lines) n += l.find("[BT] " + text) != std::string::npos ? 1 : 0;
  return n;
}

// --- the stand-in motion (LABELLED: the one node that is not production code) ------------------------
struct Motions
{
  std::mutex m;
  std::vector<std::string> moved;
  std::set<std::string> failing;
  std::map<std::string, std::function<void()>> hooks;   // run when that waypoint is reached
};
Motions& motions()
{
  static Motions m;   // test harness: one per process
  return m;
}

class StandInMove : public BT::SyncActionNode
{
public:
  StandInMove(const std::string& name, const BT::NodeConfig& cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("waypoint")}; }
  BT::NodeStatus tick() override
  {
    const std::string wp = getInput<std::string>("waypoint").value_or("");
    std::function<void()> hook;
    bool fail = false;
    {
      std::lock_guard<std::mutex> g(motions().m);
      motions().moved.push_back(wp);
      fail = motions().failing.count(wp) != 0;
      const auto it = motions().hooks.find(wp);
      if (it != motions().hooks.end()) hook = it->second;
    }
    if (hook) hook();
    return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

void standInMotion(BT::BehaviorTreeFactory& f)
{
  f.unregisterBuilder("MoveWaypoint");
  f.registerNodeType<StandInMove>("MoveWaypoint");
}

// --- the cell I/O: which process channels the tree pulsed ------------------------------------------
class RecordingProcess : public trainit::ProcessController
{
public:
  bool executeEvent(const trainit::ProcessEvent& e) override
  {
    std::lock_guard<std::mutex> g(m_);
    if (e.type == trainit::ProcessEventType::ENABLE_PROCESS) ++enabled_[e.channel];
    return !fail_enable_ || e.type != trainit::ProcessEventType::ENABLE_PROCESS;
  }
  int enabled(const std::string& channel)
  {
    std::lock_guard<std::mutex> g(m_);
    return enabled_[channel];
  }
  void reset(bool fail_enable)
  {
    std::lock_guard<std::mutex> g(m_);
    enabled_.clear();
    fail_enable_ = fail_enable;
  }

private:
  std::mutex m_;
  std::map<std::string, int> enabled_;
  bool fail_enable_{false};
};

// --- a detector publishing ranked candidates (in its optical frame) ----------------------------------
class Detector
{
public:
  Detector()
  {
    node_ = std::make_shared<rclcpp::Node>("m26_fake_detector", "/", rclcpp::NodeOptions().use_global_arguments(false));
    pub_ = node_->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/perception/red_object/detections", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    timer_ = node_->create_wall_timer(milliseconds(20), [this] {
      if (silent_) return;
      vision_msgs::msg::Detection3DArray a;
      a.header.frame_id = "cam_optical";
      for (int i = 0; i < 3; ++i)
      {
        vision_msgs::msg::Detection3D d;
        d.header.frame_id = "cam_optical";
        vision_msgs::msg::ObjectHypothesisWithPose h;
        h.hypothesis.class_id = matching_ ? "object" : "bottle";
        h.hypothesis.score = 0.9 - 0.1 * i;
        h.pose.pose.position.x = 0.1 * (i + 1);
        h.pose.pose.orientation.w = 1.0;
        d.results.push_back(h);
        a.detections.push_back(d);
      }
      pub_->publish(a);
    });
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~Detector()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  void set(bool silent, bool matching)
  {
    silent_ = silent;
    matching_ = matching;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> silent_{false}, matching_{true};
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

// --- the scene manager's attached-collision-check service, answered at once (a stub server) --------
// The generated tree toggles it around the lift; without a server the REAL node waits 2 s and
// skips (its documented no-op), which would only slow the run down. Its calls are counted.
class SceneManagerStub
{
public:
  SceneManagerStub()
  {
    node_ = std::make_shared<rclcpp::Node>("m26_scene_manager_stub", "/",
                                           rclcpp::NodeOptions().use_global_arguments(false));
    srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/scene_manager_node/attached_collision_check",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        ++calls_;
        res->success = true;
      });
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~SceneManagerStub()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  int calls() const { return calls_.load(); }
  void reset() { calls_ = 0; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  std::atomic<int> calls_{0};
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

const char* kSuction =
  "/**:\n  ros__parameters:\n    gripper_action: \"\"\n    eef:\n      ids: [\"main\"]\n      main:\n"
  "        actuator_ids: [\"vacuum\"]\n        site_ids: [\"main\"]\n        scene_signal: \"adapter\"\n"
  "        actuators:\n          vacuum: {command_interface: \"set_bool\"}\n"
  "        sites:\n          main: {grip: \"vacuum\", engage: \"engage\", release: \"release\"}\n"
  "        verification:\n          require_grasp_confirmation: true\n          require_release_confirmation: true\n"
  "          timeout_ms: 1500\n";

// The cell: ONE runner node for the process (DetectObject's feed binds to the first node).
class Cell
{
public:
  Cell() : contract(m26::after::contractFromYaml(kSuction, "m26_bin_picking"))
  {
    runner = std::make_unique<CanonicalRunner>("/", contract);
    tf = std::make_shared<tf2_ros::Buffer>(runner->node()->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = "cam_optical";
    T.transform.translation.z = 1.0;
    T.transform.rotation.w = 1.0;
    tf->setTransform(T, "m26", true);
    tf->setUsingDedicatedThread(true);
    runner->context().tf = tf;
    runner->setProcess(&process);
    rcutils_logging_output_handler_t current = rcutils_logging_get_output_handler();
    if (current != &tap)
    {
      g_previous = current;
      rcutils_logging_set_output_handler(&tap);
    }
  }
  ~Cell()
  {
    rcutils_logging_set_output_handler(g_previous);
    adapter.reset();
    runner.reset();
  }

  // A new run: the previous adapter gone, a fresh registry (empty state subscription), then the
  // scenario's adapter (or none).
  void begin(std::optional<m26::after::AdapterOptions> options, bool fail_recovery = false)
  {
    adapter.reset();
    ASSERT_TRUE(m26::waitUntil([&] { return runner->node()->count_publishers("/eef/main/state") == 0; }, milliseconds(6000)));
    runner->rebuild(contract);
    {
      std::lock_guard<std::mutex> g(motions().m);
      motions().moved.clear();
      motions().failing.clear();
      motions().hooks.clear();
    }
    {
      std::lock_guard<std::mutex> g(g_log_m);
      g_bt_lines.clear();
    }
    process.reset(fail_recovery);
    detector.set(false, true);
    scene.reset();
    if (!options) return;
    adapter = std::make_unique<FakeEefAdapter>(log, "/", *contract.device("main"), *options);
    if (options->publish_state)
    {
      ASSERT_TRUE(runner->waitForState("main", milliseconds(6000)));
    }
    if (options->serve)
    {
      ASSERT_TRUE(m26::waitUntil([&] {
        return runner->node()->get_service_names_and_types().count("/eef/main/actuators/vacuum/set_engaged") == 1;
      }, milliseconds(6000)));
    }
  }

  std::string tick()
  {
    const auto r = runner->tickDocument(
      m26::after::readFile(m26::after::contractFile("bin_picking_canonical.xml")), "MainTree",
      [](BT::Blackboard& bb) {
        // the taught poses the generated tree derives from (bt_params' tcp waypoints)
        bb.set<std::string>("pick.orientation", "0;0;0;1");
        bb.set<std::string>("place.position", "0.4;0.0;0.3");
        bb.set<std::string>("place.orientation", "0;0;0;1");
      },
      standInMotion);
    return r.status;
  }

  // the vacuum's SetBool calls: "engage" / "release", in order
  std::vector<std::string> vacuum()
  {
    std::vector<std::string> out;
    if (!adapter || !adapter->serves()) return out;
    for (const auto& c : adapter->setBool("vacuum").calls()) out.push_back(c.data ? "engage" : "release");
    return out;
  }
  // the reason of the first FAULT the registry latched during the run ("" when none)
  std::string firstFault()
  {
    const auto f = runner->registry().latchedFaults();
    return f.empty() ? std::string() : f.front().substr(0, f.front().find(' '));
  }
  // the run never reached the stall branch: no recovery pulse, no "bin empty"
  void expectNeverBinEmpty(const std::string& what)
  {
    EXPECT_EQ(process.enabled("bin_agitator"), 0) << what << ": the recovery channel was enabled";
    EXPECT_EQ(btLines("bin empty"), 0) << what << ": \"bin empty\" was logged";
    EXPECT_EQ(btLines("bin stalled"), 0) << what;
  }

  m26::Log log;
  eef::Contract contract;
  std::unique_ptr<CanonicalRunner> runner;
  std::shared_ptr<tf2_ros::Buffer> tf;
  RecordingProcess process;
  Detector detector;
  SceneManagerStub scene;
  std::unique_ptr<FakeEefAdapter> adapter;
};

Cell& cell()
{
  static std::unique_ptr<Cell> c;   // one per process (test harness)
  if (!c)
  {
    c = std::make_unique<Cell>();
    m26::teardowns().push_back([] { c.reset(); });
  }
  return *c;
}

m26::after::AdapterOptions adapterOptions() { return m26::after::AdapterOptions{}; }

struct Guard
{
  explicit Guard(const std::string& what) { m26::watchdog().arm(what, std::chrono::seconds(120)); }
  ~Guard() { m26::watchdog().disarm(); }
};
}  // namespace

// --- the nominal loop, and the branches that DO reach "bin empty" -------------------------------------
TEST(M26EefBinPicking, BP00_two_confirmed_picks_and_two_confirmed_releases)
{
  Guard g("BP00");
  auto& c = cell();
  c.begin(adapterOptions());
  EXPECT_EQ(c.tick(), "SUCCESS");
  EXPECT_EQ(c.vacuum(), (std::vector<std::string>{"engage", "release", "engage", "release"}));
  EXPECT_EQ(c.firstFault(), "");
  c.expectNeverBinEmpty("BP00");
  EXPECT_EQ(c.scene.calls(), 4) << "the attached-collision check, twice per cycle (generated by TSA)";
  std::lock_guard<std::mutex> lk(motions().m);
  EXPECT_EQ(motions().moved, (std::vector<std::string>{"ready", "pre_pick",
                                                      "pick", "post_pick", "pre_place", "place", "post_place",
                                                      "pick", "post_pick", "pre_place", "place", "post_place"}));
}

TEST(M26EefBinPicking, BP17_the_tree_is_the_pinned_byte_copy_of_what_TSA_generates)
{
  EXPECT_EQ(m25::sha256(m26::after::readFile(m26::after::contractFile("bin_picking_canonical.xml"))),
            "25fb4e2533423d150c040dee2a19bb9d65905d2a9ff247c0c8e7cf6456716b5b")
    << "test/data/m2_6_contract/bin_picking_canonical.xml is not TSA's pinned generation (README.md)";
}

TEST(M26EefBinPicking, BP01_NO_OBJECT_on_every_candidate_is_bin_empty_SUCCESS_as_today)
{
  Guard g("BP01");
  auto& c = cell();
  c.begin(adapterOptions());
  c.adapter->setSite("main", SiteScript{State::GRASP_NOT_HELD});
  EXPECT_EQ(c.tick(), "SUCCESS");
  EXPECT_EQ(c.vacuum().size(), 6u) << "three candidates per cycle, a VERDICT advances the retry";
  EXPECT_EQ(c.process.enabled("bin_agitator"), 2) << "the stall branch ran in both cycles";
  EXPECT_EQ(c.firstFault(), "") << "a VERDICT is never latched";
}

TEST(M26EefBinPicking, BP16_the_bin_empty_log_is_reached_when_the_recovery_fails)
{
  Guard g("BP16");
  auto& c = cell();
  c.begin(adapterOptions(), /*fail_recovery=*/true);
  c.adapter->setSite("main", SiteScript{State::GRASP_NOT_HELD});
  EXPECT_EQ(c.tick(), "SUCCESS");
  EXPECT_EQ(btLines("bin empty"), 2) << "the detection this test relies on works";
}

TEST(M26EefBinPicking, BP14_a_fresh_detection_with_no_match_is_bin_empty)
{
  Guard g("BP14");
  auto& c = cell();
  c.begin(adapterOptions());
  c.detector.set(false, false);
  EXPECT_EQ(c.tick(), "SUCCESS");
  EXPECT_EQ(c.vacuum().size(), 0u);
  EXPECT_EQ(c.process.enabled("bin_agitator"), 2);
}

TEST(M26EefBinPicking, BP15_a_motion_failure_with_nothing_held_reaches_the_stall_branch_as_today)
{
  Guard g("BP15");
  auto& c = cell();
  c.begin(adapterOptions());
  {
    std::lock_guard<std::mutex> lk(motions().m);
    motions().failing.insert("pick");   // the approach inside the retry, before any engage
  }
  EXPECT_EQ(c.tick(), "SUCCESS");
  EXPECT_EQ(c.vacuum().size(), 0u);
  EXPECT_EQ(c.process.enabled("bin_agitator"), 2);
}

// --- P12: every one of these is FAILURE, never "bin empty" ---------------------------------------------
// P12(1): the SetBool server alive and NO state publisher at all -- the device's own state never
// shows the engage, so CloseGripper itself fails (a set_bool command is DONE only when it is
// reflected, ADR-0022 D2), and no sample ever arrived: STATE_MISSING.
TEST(M26EefBinPicking, BP02_no_state_publisher_at_all_is_STATE_MISSING_after_ONE_engage)
{
  Guard g("BP02");
  auto& c = cell();
  auto o = adapterOptions();
  o.publish_state = false;   // the SetBool server is alive, no state publisher exists
  c.begin(o);
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.vacuum(), (std::vector<std::string>{"engage"})) << "ONE engage, then the gate";
  EXPECT_EQ(c.firstFault(), "STATE_MISSING");
  c.expectNeverBinEmpty("BP02");
}

// The publisher exists and has published, then goes silent (FakeEefAdapter::pause), the SetBool
// server alive: no NEW sample since the engage -- STATE_STALE at the command, never a remap's
// CONTRACT_MISMATCH.
TEST(M26EefBinPicking, BP18_a_paused_state_publisher_is_STATE_STALE_at_the_command_after_ONE_engage)
{
  Guard g("BP18");
  auto& c = cell();
  c.begin(adapterOptions());   // the runner holds the adapter's samples ...
  c.adapter->pause(true);      // ... then the adapter publishes nothing more
  ASSERT_TRUE(c.runner->waitForQuiet("main", milliseconds(200), milliseconds(3000)));
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.vacuum(), (std::vector<std::string>{"engage"})) << "ONE engage, then the gate";
  EXPECT_EQ(c.firstFault(), "STATE_STALE");
  const auto f = c.runner->registry().latchedFaults();
  ASSERT_FALSE(f.empty());
  EXPECT_NE(f.front().find("CloseGripper: the state publisher sent nothing since the command"), std::string::npos) << f.front();
  c.expectNeverBinEmpty("BP18");
}

TEST(M26EefBinPicking, BP03_a_dead_adapter_is_SERVER_MISSING_with_no_engage)
{
  Guard g("BP03");
  auto& c = cell();
  c.begin(std::nullopt);
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.firstFault(), "SERVER_MISSING");
  c.expectNeverBinEmpty("BP03");
}

TEST(M26EefBinPicking, BP04_to_BP10_every_fault_ends_the_run)
{
  struct Case
  {
    std::string id;
    std::function<void(m26::after::AdapterOptions&)> options;
    std::function<void(FakeEefAdapter&)> script;
  };
  const std::vector<Case> cases{
    {"BP04 STATE_UNKNOWN", {}, [](FakeEefAdapter& a) { a.setSite("main", SiteScript{State::GRASP_UNKNOWN}); }},
    {"BP05 STATE_STALE", {}, [](FakeEefAdapter& a) { a.silence("main", true); }},
    {"BP06 WRONG_EEF", [](m26::after::AdapterOptions& o) { o.identity = "/r9/eef/main"; }, {}},
    {"BP07 COMMAND_REJECTED", {}, [](FakeEefAdapter& a) { a.setBool("vacuum").script({SetBoolReply::reject("no stage subscriber")}); }},
    {"BP08 CONTRACT_MISMATCH", [](m26::after::AdapterOptions& o) { o.published_actuators = {"pump"}; }, {}},
    {"BP09 RELEASE_NOT_CONFIRMED", {}, [](FakeEefAdapter& a) {
       a.setSite("main", SiteScript{State::GRASP_HELD, State::GRASP_HELD});
     }},
    {"BP10 EVIDENCE_NOT_ACCEPTED", {}, [](FakeEefAdapter& a) {
       a.setSite("main", SiteScript{State::GRASP_HELD, State::GRASP_NOT_HELD, State::EVIDENCE_ACTUATION_INFERENCE});
     }},
  };
  auto& c = cell();
  for (const auto& k : cases)
  {
    Guard g(k.id);
    auto o = adapterOptions();
    if (k.options) k.options(o);
    c.begin(o);
    if (k.script) k.script(*c.adapter);
    EXPECT_EQ(c.tick(), "FAILURE") << k.id;
    EXPECT_EQ(c.firstFault(), k.id.substr(5)) << k.id;
    c.expectNeverBinEmpty(k.id);
    const auto v = c.vacuum();
    EXPECT_FALSE(v.empty()) << k.id;
    EXPECT_EQ(std::count(v.begin(), v.end(), "engage"), 1) << k.id << ": no candidate is retried after a FAULT";
  }
}

TEST(M26EefBinPicking, BP11_an_object_lost_before_the_release_ends_the_run_in_HOLD)
{
  Guard g("BP11");
  auto& c = cell();
  c.begin(adapterOptions());
  {
    std::lock_guard<std::mutex> lk(motions().m);
    motions().hooks["place"] = [&c] {
      c.adapter->drop("main");   // the object falls during the place move
      m26::waitUntil([&c] {
        const auto s = c.runner->device("main").latest().sample;
        return s && !s->site_grasp.empty() && s->site_grasp[0] == State::GRASP_LOST;
      }, milliseconds(2000));
    };
  }
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.vacuum(), (std::vector<std::string>{"engage"})) << "never released (HOLD, EEF-D11)";
  EXPECT_EQ(c.firstFault(), "") << "OBJECT_LOST is a VERDICT: gated by `reason` and `holding`, not latched";
  c.expectNeverBinEmpty("BP11");
}

TEST(M26EefBinPicking, BP12_a_place_motion_failing_after_a_confirmed_HELD_ends_the_run_in_HOLD)
{
  Guard g("BP12");
  auto& c = cell();
  c.begin(adapterOptions());
  {
    std::lock_guard<std::mutex> lk(motions().m);
    motions().failing.insert("place");
  }
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.vacuum(), (std::vector<std::string>{"engage"})) << "the next cycle never re-confirms a carried object";
  EXPECT_EQ(c.firstFault(), "") << "a motion failure while holding: gated by `holding`";
  c.expectNeverBinEmpty("BP12");
}

TEST(M26EefBinPicking, BP13_a_silent_detector_ends_the_run)
{
  Guard g("BP13");
  auto& c = cell();
  c.begin(adapterOptions());
  c.detector.set(true, true);
  EXPECT_EQ(c.tick(), "FAILURE");
  EXPECT_EQ(c.vacuum().size(), 0u);
  EXPECT_EQ(c.firstFault(), "") << "DetectObject's `fault` gates the stall branch";
  c.expectNeverBinEmpty("BP13");
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
