// vNext M2.7 test harness (owner Q8.2, motion FAULT vs a candidate the robot cannot reach) -- the
// CANONICAL bin-picking loop ticked as XML with the REAL MoveWaypoint on a REAL MoveItMotionRuntime
// against the M2.5 fake move_group, and the REAL end-effector registry against the M2.6 fake
// end-effector adapter, in ONE process.
//
// test_m2_6_eef_bin_picking (M2.6 M5) proves the generated gate with a STAND-IN MoveWaypoint (a
// flag decides a move). Here every node of the tree is production code: a move's outcome is what
// MoveWaypoint makes of MoveIt's answer, and the fake move_group scripts that answer per goal
// ordinal. What the tree then does with it -- the next candidate, the stall branch, HOLD, the end of
// the run -- is recorded, together with every request MoveIt received.
//
//   Cell        ONE per process (DetectObject's process-static feed binds to the first node it sees;
//               MoveIt caches the robot model per process, so one runtime exists at a time,
//               test/m2_5/test_arm.hpp): the M2.5 Log (bodies on), FakeMoveGroup (every
//               endpoint, at "/") and FakeJointStates ("/"); the runner node trainit_run_bt at "/"
//               with the M2.5 test arm as parameter overrides; a TF buffer holding the static
//               base_link -> cam_optical (z = 1.0); COPIES of the M2.6 M5 harness's detector,
//               scene-manager stub, recording process controller and rcutils tap (that test file is
//               not edited); the canonical suction contract of M2.6 M5.
//               Per scenario, in trainit_run_bt's order (src/trainit_run_bt_node.cpp:169-192): the
//               previous spinner, runtime and registry gone (the M2.5 judge's teardown order), the
//               previous adapter gone from the graph, MoveIt's fakes back to every endpoint with
//               this scenario's script, a fresh EndEffectorRegistry BEFORE the executor adds the
//               node, the executor (2 + 1 threads) spinning, a fresh MoveItMotionRuntime (then
//               300 ms, m25::Rig::build), a fresh FakeEefAdapter with its state and its SetBool
//               server up, and the BtContext (THE ADAPTER m25::makeContext + process, tf, eefs).
//   Scenario    the moves' MoveIt answers (FakeMoveGroup scripts, consumed one per goal), the cell's
//               other inputs, and an optional hook run whenever a tick returns RUNNING.
//   run()       one scenario's capture block (format: test/data/m2_7_mf/README.md).
//   check()     writes a capture to TRAINIT_M27_ACTUAL_DIR and compares it with the committed one in
//               TRAINIT_M27_DATA_DIR, line for line the way m26::Family::check() does (the
//               environment first, then the whole capture). It is not a capture Family class. The
//               BEFORE capture (the M2.6 tree) was frozen on adf94b0, the AFTER one (the M2.7 tree) on
//               vNext M2.7 TMR-7: each only ever re-frozen from those production sources.
//
// Reused unchanged: the M2.5 rig pieces (Log, FakeMoveGroup, FakeJointStates, armParameters,
// defaultConfig, makeContext, settleTopics, sha256) and the M2.6 ones (environmentLine, Spinner,
// waitUntil, runAll, watchdog, contractFromYaml, FakeEefAdapter).
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_7_MF_BIN_PICKING_CELL_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_7_MF_BIN_PICKING_CELL_HPP_

#include <rcutils/logging.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "m2_5/motion_harness.hpp"
#include "m2_6/capture26.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace m27mf
{
using m26::milliseconds;

// The production sources the BEFORE capture was frozen on (test/data/m2_7_mf/README.md).
inline constexpr const char* kFrozenOn = "adf94b0";
// ... and the AFTER capture (the M2.7 tree): the commit that added MoveWaypoint's `fault` with it.
inline constexpr const char* kAfterFrozenOn = "vNext M2.7 TMR-7 (the commit that added it)";

// The family's name, line 1 of every capture of it (BEFORE and AFTER files alike).
inline constexpr const char* kFamily = "bin_picking_motion";

// --- the log lines the tree writes, captured through rcutils' output handler (copy of M2.6 M5's tap)
// "[BT] ..." lines (the tree's Log nodes) and the NAME of every MoveWaypoint INFO line (logged after
// every runtime call, MoveWaypoint::tick in src/bt_nodes.cpp: "MoveWaypoint '<name>' (type=...").
inline std::mutex g_tap_m;
inline std::vector<std::string> g_bt_lines;
inline std::vector<std::string> g_moves;
inline rcutils_logging_output_handler_t g_previous = nullptr;

inline void tap(const rcutils_log_location_t* location, int severity, const char* name,
                rcutils_time_point_value_t ts, const char* format, va_list* args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[2048];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  {
    std::lock_guard<std::mutex> g(g_tap_m);
    if (std::strstr(buf, "[BT] ") != nullptr) g_bt_lines.emplace_back(buf);
    const std::string line(buf);
    const std::string head = "MoveWaypoint '";
    if (severity == RCUTILS_LOG_SEVERITY_INFO && line.rfind(head, 0) == 0)
    {
      const auto end = line.find("' (type=", head.size());
      if (end != std::string::npos) g_moves.push_back(line.substr(head.size(), end - head.size()));
    }
  }
  if (g_previous) g_previous(location, severity, name, ts, format, args);
}

inline int btLines(const std::string& text)
{
  std::lock_guard<std::mutex> g(g_tap_m);
  int n = 0;
  for (const auto& l : g_bt_lines) n += l.find("[BT] " + text) != std::string::npos ? 1 : 0;
  return n;
}

inline std::vector<std::string> moves()
{
  std::lock_guard<std::mutex> g(g_tap_m);
  return g_moves;
}

inline void clearTap()
{
  std::lock_guard<std::mutex> g(g_tap_m);
  g_bt_lines.clear();
  g_moves.clear();
}

// --- the cell I/O: which process channels the tree pulsed (copy of M2.6 M5's RecordingProcess) -------
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

// --- a detector publishing ranked candidates (copy of M2.6 M5's Detector) ---------------------------
// Three candidates of class "object" (or "bottle": a fresh message with no match), best first, in the
// frame `frame` (cam_optical, which the TF buffer knows; another name has no transform), every 20 ms
// unless silent.
class Detector
{
public:
  static constexpr const char* kKnownFrame = "cam_optical";

  Detector()
  {
    node_ = std::make_shared<rclcpp::Node>("m27mf_fake_detector", "/",
                                           rclcpp::NodeOptions().use_global_arguments(false));
    pub_ = node_->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/perception/red_object/detections", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    timer_ = node_->create_wall_timer(milliseconds(20), [this] {
      if (silent_) return;
      std::string frame;
      {
        std::lock_guard<std::mutex> g(m_);
        frame = frame_;
      }
      vision_msgs::msg::Detection3DArray a;
      a.header.frame_id = frame;
      for (int i = 0; i < 3; ++i)
      {
        vision_msgs::msg::Detection3D d;
        d.header.frame_id = frame;
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
  Detector(const Detector&) = delete;
  Detector& operator=(const Detector&) = delete;

  void set(bool silent, bool matching, const std::string& frame = kKnownFrame)
  {
    {
      std::lock_guard<std::mutex> g(m_);
      frame_ = frame;
    }
    silent_ = silent;
    matching_ = matching;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> silent_{false}, matching_{true};
  std::mutex m_;
  std::string frame_{kKnownFrame};
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

// --- the scene manager's attached-collision-check service, answered at once (copy of M2.6 M5's stub)
// The generated tree toggles it around the lift; without a server the REAL node waits 2 s and skips
// (its documented no-op). Its calls are counted.
class SceneManagerStub
{
public:
  SceneManagerStub()
  {
    node_ = std::make_shared<rclcpp::Node>("m27mf_scene_manager_stub", "/",
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
  SceneManagerStub(const SceneManagerStub&) = delete;
  SceneManagerStub& operator=(const SceneManagerStub&) = delete;

  int calls() const { return calls_.load(); }
  void reset() { calls_ = 0; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  std::atomic<int> calls_{0};
  m26::Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

// The canonical contract of M2.6 M5 (one suction device "main", grasp and release confirmed).
inline const char* kSuction =
  "/**:\n  ros__parameters:\n    gripper_action: \"\"\n    eef:\n      ids: [\"main\"]\n      main:\n"
  "        actuator_ids: [\"vacuum\"]\n        site_ids: [\"main\"]\n        scene_signal: \"adapter\"\n"
  "        actuators:\n          vacuum: {command_interface: \"set_bool\"}\n"
  "        sites:\n          main: {grip: \"vacuum\", engage: \"engage\", release: \"release\"}\n"
  "        verification:\n          require_grasp_confirmation: true\n          require_release_confirmation: true\n"
  "          timeout_ms: 1500\n";

// The pinned canonical bin-picking project's waypoints, as trainit_run_bt's loadTaskParameters writes
// them on the blackboard (strings): TSA's generation for that project, the golden's bt_params
// waypoints (the same values as test_m2_5_more_captures' golden_replay). The tree derives pick from
// the detection (orientation kept), post_pick from pick and post_place from place.
inline std::vector<std::pair<std::string, std::string>> waypoints()
{
  return {
    {"ready.type", "joint"}, {"ready.named", "ready"}, {"ready.motion", "ptp"},
    {"ready.planner", "ompl"}, {"ready.speed", "90"},
    {"pre_pick.type", "tcp"}, {"pre_pick.position", "0.4316;-0.0137;0.161"},
    {"pre_pick.orientation", "-0.7006;0.7134;0.0113;-0.0061"}, {"pre_pick.motion", "free"},
    {"pre_pick.planner", "ompl"}, {"pre_pick.speed", "90"},
    {"pick.type", "tcp"}, {"pick.position", "0.5608;0.0321;0.0307"},
    {"pick.orientation", "-0.707;0.7071;-0.0072;0.0066"}, {"pick.motion", "lin"},
    {"pick.planner", "pilz"}, {"pick.speed", "50"},
    {"post_pick.type", "tcp"}, {"post_pick.position", "0.4241;-0.2772;0.0494"},
    {"post_pick.orientation", "-0.7006;0.7135;0.0116;-0.006"}, {"post_pick.motion", "lin"},
    {"post_pick.planner", "pilz"}, {"post_pick.speed", "80"},
    {"pre_place.type", "tcp"}, {"pre_place.position", "0.4267;-0.278;0.1589"},
    {"pre_place.orientation", "-0.7006;0.7134;0.0117;-0.006"}, {"pre_place.motion", "ptp"},
    {"pre_place.planner", "pilz"}, {"pre_place.speed", "80"},
    {"place.type", "tcp"}, {"place.position", "0.3375;-0.2562;0.0354"},
    {"place.orientation", "-0.707;0.7071;-0.0071;0.0068"}, {"place.motion", "lin"},
    {"place.planner", "pilz"}, {"place.speed", "50"},
    {"post_place.type", "tcp"}, {"post_place.orientation", "0;0;0;1"}, {"post_place.motion", "lin"},
    {"post_place.planner", "pilz"}, {"post_place.speed", "90"},
  };
}

inline std::string join(const std::vector<std::string>& v, const std::string& sep = " ")
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? sep : "") + v[i];
  return s;
}

inline std::string listOrNone(const std::vector<std::string>& v)
{
  return v.empty() ? std::string("(none)") : join(v);
}

// --- the tree a capture ticks ---------------------------------------------------------------------
struct TreeFile
{
  std::string label;   // as the capture names it, e.g. "m2_6_contract/bin_picking_canonical.xml"
  std::string path;    // where it is read from
};

inline std::string treeLine(const TreeFile& t)
{
  return "tree: " + t.label + " sha256=" + m25::sha256(m26::after::readFile(t.path));
}

class Cell;

// --- a scenario ------------------------------------------------------------------------------------
struct Scenario
{
  std::string id{};
  std::string pins{};                                    // one line: what this scenario pins
  std::function<void(m25::FakeMoveGroup&)> script{};     // MoveIt's answers, one per goal, in order
  std::string hook{"none"};                              // the capture's name for `between_ticks`
  std::function<void(Cell&)> setup{};                    // the cell's other inputs (the detector)
  std::function<void(Cell&)> between_ticks{};            // whenever a tick returns RUNNING
  std::function<void(Cell&)> after{};                    // e.g. restore a dropped server
};

// One tick of the tree to its end, the way trainit_run_bt ticks it.
struct Outcome
{
  std::string status{};       // SUCCESS | FAILURE | THROW ... | LOAD FAILED: ... | REFUSED AT START-UP: ...
  int runner_exit{1};         // the exit code trainit_run_bt derives (src/trainit_run_bt_node.cpp:228-314)
  BT::Blackboard::Ptr bb{};
};

// The M2.5 Log's events as the list it recorded: an event starts at a line "[n] " in column 0; its
// summary and body lines are indented (block YAML at indent >= 2, summaries "  | ...").
inline std::vector<std::string> splitEvents(const std::string& events)
{
  std::vector<std::string> out;
  std::istringstream in(events);
  std::string line;
  while (std::getline(in, line))
  {
    if (!line.empty() && line[0] == '[') out.push_back(line);
    else if (!out.empty()) out.back() += "\n" + line;
  }
  return out;
}

// One event, compact: its first line verbatim, its first summary line verbatim (when it has one), and
// the sha256 of the WHOLE rest of the event as the M2.5 Log recorded it after that first line: every
// summary line and the block-style YAML body at precision 17, stamps removed. An event with nothing
// after its first line (GetPlanningScene) hashes the empty text (e3b0c442...). The first line, with
// its ordinal, is not hashed: two identical requests (a plan's retries) carry the same digest.
inline std::string compactEvent(const std::string& event)
{
  const auto nl = event.find('\n');
  std::string out = event.substr(0, nl) + "\n";
  const std::string rest = nl == std::string::npos ? std::string() : event.substr(nl + 1);
  const auto nl2 = rest.find('\n');
  const std::string second = rest.substr(0, nl2);
  if (second.rfind("  | ", 0) == 0) out += second + "\n";
  return out + "  sha256=" + m25::sha256(rest) + "\n";
}

// The ordinals of the events must be 1..N: a gap would mean the list was not read as recorded.
inline bool consecutive(const std::vector<std::string>& events)
{
  for (std::size_t i = 0; i < events.size(); ++i)
    if (events[i].rfind("[" + std::to_string(i + 1) + "] ", 0) != 0) return false;
  return true;
}

// =====================================================================================================
// The cell
// =====================================================================================================
class Cell
{
public:
  Cell()
  : mg_(motion_log_, "/"),
    js_("/"),
    contract_(m26::after::contractFromYaml(kSuction, "m27mf_bin_picking"))
  {
    motion_log_.setBodies(true);
    node_ = std::make_shared<rclcpp::Node>(
      "trainit_run_bt", "/",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                           .parameter_overrides(m25::armParameters()));
    profiles_ = trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = Detector::kKnownFrame;
    T.transform.translation.z = 1.0;
    T.transform.rotation.w = 1.0;
    tf_->setTransform(T, "m27mf", true);
    tf_->setUsingDedicatedThread(true);
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
    adapter_.reset();
    stopSpinning();      // the M2.5 judge's teardown order: stop spinning, destroy the runtime
    runtime_.reset();
    registry_.reset();
  }
  Cell(const Cell&) = delete;
  Cell& operator=(const Cell&) = delete;

  // One scenario on a fresh runtime, registry and adapter: its capture block.
  std::string run(const Scenario& s, const TreeFile& tree)
  {
    std::ostringstream os;
    os << "=== scenario " << s.id << "\n"
       << "pins: " << s.pins << "\n";
    m26::watchdog().arm("build " + s.id, std::chrono::seconds(90));
    const std::string problem = begin(s);
    m26::watchdog().disarm();
    os << "script: " << mg_.scriptText() << " | hook: " << s.hook << "\n";
    if (!problem.empty())
    {
      ADD_FAILURE() << s.id << ": the cell could not be set up: " << problem;
      os << "setup: FAILED: " << problem << "\n\n";
      return os.str();
    }

    motion_log_.reset();   // what the build sent (the scene monitor's first request) is not the run's
    m26::watchdog().arm(s.id, std::chrono::seconds(240));
    const Outcome o = tick(m26::after::readFile(tree.path), s);
    m25::settleTopics(motion_log_);   // asynchronous topics land
    m26::watchdog().disarm();
    if (s.after) s.after(*this);

    const auto events = splitEvents(motion_log_.events());
    EXPECT_TRUE(consecutive(events)) << s.id << ": the M2.5 Log's events are not numbered 1..N";
    if (events.empty())
    {
      os << "events: (none)\n";
    }
    else
    {
      os << "events:\n";
      for (const auto& e : events) os << compactEvent(e);
    }
    const std::string topics = motion_log_.topics();
    if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;

    std::vector<std::string> vacuum;
    for (const auto& c : adapter_->setBool("vacuum").calls()) vacuum.push_back(c.data ? "engage" : "release");
    std::vector<std::string> latched;
    for (const auto& f : registry_->latchedFaults()) latched.push_back(f.substr(0, f.find(' ')));
    std::string eef_fault = "(absent)";
    if (o.bb && o.bb->getEntry("eef_fault") != nullptr)
    {
      try { eef_fault = "\"" + o.bb->get<std::string>("eef_fault") + "\""; }
      catch (const std::exception& e) { eef_fault = std::string("(unreadable: ") + e.what() + ")"; }
    }
    os << "moves: " << listOrNone(moves()) << "\n"
       << "bt: " << o.status << "\n"
       << "stall: recovery=" << process_.enabled("bin_agitator") << " bin_empty=" << btLines("bin empty")
       << " bin_stalled=" << btLines("bin stalled") << "\n"
       << "vacuum: " << listOrNone(vacuum) << "\n"
       << "scene_calls: " << scene_.calls() << "\n"
       << "eef_fault: " << eef_fault << "\n"
       << "latched: " << listOrNone(latched) << "\n"
       << "runner_exit: " << o.runner_exit << "\n\n";
    return os.str();
  }

  // --- what a scenario's setup and hooks touch ----------------------------------------------------------
  m25::Log& motionLog() { return motion_log_; }
  m25::FakeMoveGroup& moveGroup() { return mg_; }
  Detector& detector() { return detector_; }

private:
  // Per scenario, in trainit_run_bt's order (see the file comment). "" on success, else what failed.
  std::string begin(const Scenario& s)
  {
    // 1. the previous run's spinner, runtime and registry gone
    stopSpinning();
    runtime_.reset();
    registry_.reset();
    // 2. the previous adapter gone, and gone from the graph (the next registry starts empty)
    adapter_.reset();
    if (!m26::waitUntil([&] { return node_->count_publishers("/eef/main/state") == 0; }, milliseconds(6000)))
      return "the previous adapter's state publisher is still in the graph";
    // 3. MoveIt's fakes: every endpoint served again, the scripts cleared, then this scenario's answers
    mg_.start(m25::FakeMoveGroup::ALL);
    mg_.resetScenario();
    if (s.script) s.script(mg_);
    // the cell's other inputs back to nominal (the scenario's own setup comes last)
    process_.reset(false);
    detector_.set(false, true);
    scene_.reset();
    clearTap();
    // 4. the registry BEFORE the executor adds the node (its callback groups are collected with it)
    registry_ = std::make_unique<trainit::EndEffectorRegistry>(node_, contract_);
    // 5. the spinner FIRST: two threads plus one per device
    startSpinning();
    // 6. the runtime, then its clients get discovered (m25::Rig::build)
    runtime_ = std::make_unique<trainit::MoveItMotionRuntime>(node_, m25::defaultConfig());
    std::this_thread::sleep_for(milliseconds(300));
    // 7. the device's adapter, its state and its SetBool server up (M2.6 M5 Cell::begin)
    adapter_ = std::make_unique<m26::after::FakeEefAdapter>(eef_log_, "/", *contract_.device("main"),
                                                           m26::after::AdapterOptions{});
    if (!m26::waitUntil([&] { return registry_->device("main")->latest().sample != nullptr; }, milliseconds(6000)))
      return "no state sample of eef/main reached the registry";
    if (!m26::waitUntil([&] {
          return node_->get_service_names_and_types().count("/eef/main/actuators/vacuum/set_engaged") == 1;
        }, milliseconds(6000)))
      return "the adapter's SetBool server is not in the graph";
    // 8. the BtContext: THE ADAPTER (m25::makeContext), then what trainit_run_bt sets for a canonical run
    ctx_ = m25::makeContext(runtime_.get(), node_);
    ctx_.process = &process_;
    ctx_.tf = tf_;
    ctx_.eefs = registry_.get();
    if (s.setup) s.setup(*this);
    return "";
  }

  // Ticks `document` as trainit_run_bt does (tickOnce while RUNNING, 10 ms apart) with the blackboard
  // it sets (context, profiles, node, task parameters), and derives the runner's exit code: a tree that
  // does not load or a contract check that refuses it is exit 2 (trainit_run_bt_node.cpp:228-251); a
  // canonical tree that throws is halted and FAILS (:288-303); SUCCESS is 0, or 3 when an end-effector
  // FAULT was latched, FAILURE 1 (:306-314).
  Outcome tick(const std::string& document, const Scenario& s)
  {
    Outcome o;
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    o.bb = BT::Blackboard::create();
    o.bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    o.bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES, profiles_);
    o.bb->set<rclcpp::Node::SharedPtr>(trainit::BB_ROS_NODE, node_);
    for (const auto& [k, v] : waypoints()) o.bb->set<std::string>(k, v);
    BT::Tree tree;
    try
    {
      factory.registerBehaviorTreeFromText(document);
      tree = factory.createTree("MainTree", o.bb);
    }
    catch (const std::exception& e)
    {
      o.status = std::string("LOAD FAILED: ") + e.what();
      o.runner_exit = 2;
      return o;
    }
    const auto problems = registry_->checkTree(tree);
    if (!problems.empty())
    {
      o.status = "REFUSED AT START-UP: " + problems.front();
      o.runner_exit = 2;
      return o;
    }
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    try
    {
      while (status == BT::NodeStatus::RUNNING)
      {
        status = tree.tickOnce();
        if (status == BT::NodeStatus::RUNNING)
        {
          if (s.between_ticks) s.between_ticks(*this);
          std::this_thread::sleep_for(milliseconds(10));
        }
      }
      o.status = BT::toStr(status);
    }
    catch (const std::exception& e)
    {
      o.status = "THROW " + m25::demangle(typeid(e).name()) + ": " + e.what();
      try { tree.haltTree(); } catch (const std::exception&) {}
      status = BT::NodeStatus::FAILURE;
    }
    o.runner_exit = status == BT::NodeStatus::SUCCESS ? (registry_->faultLatched() ? 3 : 0) : 1;
    return o;
  }

  void startSpinning()
  {
    spin_ = std::make_unique<m26::Spinner<rclcpp::executors::MultiThreadedExecutor>>(
      rclcpp::ExecutorOptions(), 2 + contract_.devices.size());
    spin_->executor().add_node(node_);
    spin_->start();
  }
  void stopSpinning()
  {
    if (!spin_) return;
    spin_->stop();
    spin_->executor().remove_node(node_);
    spin_.reset();
  }

  // ONE per process, destroyed in reverse order after the destructor's body (the per-scenario parts
  // first, then the stubs, the node, and last the fake move_group and joint states).
  m25::Log motion_log_;
  m25::FakeMoveGroup mg_;
  m25::FakeJointStates js_;
  m26::Log eef_log_;
  trainit::eef::Contract contract_;
  rclcpp::Node::SharedPtr node_;
  trainit::MotionProfileRegistry profiles_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  RecordingProcess process_;
  Detector detector_;
  SceneManagerStub scene_;
  // per scenario
  std::unique_ptr<trainit::EndEffectorRegistry> registry_;
  std::unique_ptr<m26::Spinner<rclcpp::executors::MultiThreadedExecutor>> spin_;
  std::unique_ptr<trainit::MoveItMotionRuntime> runtime_;
  std::unique_ptr<m26::after::FakeEefAdapter> adapter_;
  trainit::BtContext ctx_{};
};

// The ONE cell of a process, destroyed by m26::runAll() before rclcpp::shutdown().
inline Cell& cell()
{
  static std::unique_ptr<Cell> c;   // test harness: one per process
  if (!c)
  {
    c = std::make_unique<Cell>();
    m26::teardowns().push_back([] { c.reset(); });
  }
  return *c;
}

// A whole capture: the header (the family, the environment, the tree) and one block per scenario.
inline std::string capture(const std::vector<Scenario>& scenarios, const TreeFile& tree)
{
  std::string text = std::string("m2_7 capture ") + kFamily + "\n" + m26::environmentLine() + "\n" +
                     treeLine(tree) + "\n\n";
  for (const auto& s : scenarios) text += cell().run(s, tree);
  return text;
}

// Writes `text` to TRAINIT_M27_ACTUAL_DIR/<file>.txt, then compares it with the committed
// TRAINIT_M27_DATA_DIR/<file>.txt as m26::Family::check() does: lines 1-2 (the family and the
// environment) first -- another environment is NOT a pass, and NOT a reason to re-capture -- then
// the whole capture.
inline void check(const std::string& text, const std::string& file, const char* frozen_on = kFrozenOn)
{
  const std::string actual = std::string(TRAINIT_M27_ACTUAL_DIR) + "/" + file + ".txt";
  {
    std::ofstream out(actual, std::ios::binary | std::ios::trunc);
    out << text;
  }
  const std::string frozen = std::string(TRAINIT_M27_DATA_DIR) + "/" + file + ".txt";
  std::ifstream in(frozen, std::ios::binary);
  ASSERT_TRUE(in.is_open()) << "no committed capture " << frozen << " -- this run's capture is " << actual
                            << ". This capture is frozen only from the " << frozen_on
                            << " production sources, after the determinism gate (test/data/m2_7_mf/README.md).";
  std::stringstream committed;
  committed << in.rdbuf();
  const std::string want = committed.str();
  const auto header_end = [](const std::string& s) {
    const auto a = s.find('\n');
    return a == std::string::npos ? s.size() : s.find('\n', a + 1);
  };
  ASSERT_EQ(want.substr(0, header_end(want)), text.substr(0, header_end(text)))
    << "the environment differs from the one the capture was recorded in (BT.CPP, the rmw or MoveIt): "
       "this is NOT a pass, and NOT a reason to re-capture from the current code. This capture is "
       "re-frozen only from the " << frozen_on << " production sources (test/data/m2_7_mf/README.md). "
       "This run's capture: " << actual;
  EXPECT_EQ(want, text) << "the " << file << " capture changed; compare " << frozen << " with " << actual
                        << ". A behaviour change is an owner decision, never a re-freeze "
                        << "(test/data/m2_7_mf/README.md; frozen on production " << frozen_on << ").";
}

}  // namespace m27mf

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_7_MF_BIN_PICKING_CELL_HPP_
