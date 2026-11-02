// test_m3_5_follow_path_range (vNext M3.5, ADR-0024, M3_DESIGN §9): FollowPath from= to= -- the
// event-free runs a path with process events is split into -- ticked for REAL (BT nodes,
// MoveItMotionRuntime, the Pilz sequence backend) against the M2.5 in-process fake move_group.
//
// What it pins:
//   F  a run states[from..to] is the SAME path: one sequence goal with the run's segments only, the
//      same speeds and blend radii as the whole path's items (a split path's runs together send what
//      the same path without events sends); from == to sends nothing (it confirms the robot is there);
//   G  the split's boundaries: a run cannot drive through an event state, end on a blend, run a dense
//      path, or name states the path does not have (PATH_RANGE, nothing sent); a run ending at an event
//      state confirms the arrival (the M3.4 settle: bounded, nothing sent; NOT_AT_PATH_STATE when the
//      arm is not there); the start check of a run is at states[from];
//   and FollowPath without the ports runs a path with no event exactly as before (M3.1's FP family is
//   byte-identical in its own test).
// The family is compared with the committed AFTER record test/data/m3_5/captures/ (new behaviour: no
// BEFORE; the legacy families stay byte-identical in their own tests).
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/path_library.hpp"

namespace
{
using trainit::PathLibrary;
using trainit::PathRuntimeFacts;
using trainit::ProcessPathBackend;

const std::string kData = TRAINIT_M35_DATA_DIR;

// TF for the rig: world -> base_link (the test arm's fixed joint), what the tip pose is read in
struct Tf
{
  explicit Tf(const rclcpp::Node::SharedPtr& node)
  {
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, true);
    pub_node = std::make_shared<rclcpp::Node>("m35_static_tf", "/");
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(pub_node);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.z = 0.1;
    t.transform.rotation.w = 1.0;
    broadcaster->sendTransform(t);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline &&
           !buffer->canTransform("base_link", "world", tf2::TimePointZero))
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Node::SharedPtr pub_node;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};

Tf& tf()
{
  static std::unique_ptr<Tf> t;   // test harness: one per process, torn down with the world
  if (!t)
  {
    t = std::make_unique<Tf>(m25::world().rig.node());
    m25::teardowns().push_back([] { t.reset(); });
  }
  return *t;
}

// the arm's yaw at each state of the fixture's paths (j2 = j3 = 0)
const std::vector<double> kYaw{0.0, 0.2, 0.4, 0.6};
std::vector<double> at(std::size_t state) { return {kYaw[state], 0.0, 0.0}; }

struct Case
{
  std::string id;
  std::string pins;
  std::string tree;
  std::vector<double> q;                         // where the arm is when the tick starts
  std::function<void(m25::World&)> during{};     // runs beside the tick (the arm "arrives")
  std::string bt;
  std::string fault;                             // "" = not written
  bool no_events{false};                         // nothing may reach MoveIt
};

std::shared_ptr<const PathLibrary> library()
{
  static std::shared_ptr<const PathLibrary> lib = std::make_shared<const PathLibrary>(
    trainit::loadPathLibrary(kData + "/paths.yaml", PathRuntimeFacts{ProcessPathBackend::PILZ_SEQUENCE, "tcp"}));
  return lib;
}

std::string tickOnce(m25::World& w, const std::string& xml, const BT::Blackboard::Ptr& bb)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  trainit::BtContext ctx = m25::makeContext(w.rig.runtime(), w.rig.node());
  ctx.tf = tf().buffer;
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES,
                                          trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE));
  bb->set<std::shared_ptr<const PathLibrary>>(trainit::BB_PATH_LIBRARY, library());
  try
  {
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         xml + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return BT::toStr(status);
  }
  catch (const std::exception& e)
  {
    return std::string("THROW ") + e.what();
  }
}

m25::Family& family()
{
  static m25::Family f("m3_5_follow_path_range", kData + "/captures");   // test harness: one per process
  return f;
}

std::string lineValue(const std::string& text, const std::string& prefix)
{
  const auto a = text.find("\n" + prefix);
  if (a == std::string::npos) return "<none>";
  const auto b = text.find('\n', a + 1);
  return text.substr(a + 1 + prefix.size(), b - a - 1 - prefix.size());
}

void runCase(const Case& c)
{
  m25::World& w = m25::world();
  tf();
  m25::Config cfg = m25::defaultConfig();
  w.js.set(c.q);
  w.js.on(true);
  w.mg.resetScenario();
  w.dog.arm("build " + c.id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();

  auto bb = BT::Blackboard::create();
  w.log.reset();
  w.dog.arm(c.id, std::chrono::seconds(120));
  std::thread during;
  if (c.during) during = std::thread([&c, &w] { c.during(w); });
  const std::string bt = tickOnce(w, c.tree, bb);
  if (during.joinable()) during.join();
  m25::settleTopics(w.log);
  w.dog.disarm();
  std::string fault = "<unset>";
  try { fault = "\"" + bb->get<std::string>("f") + "\""; } catch (const std::exception&) {}

  std::ostringstream os;
  os << "=== scenario " << c.id << "\n"
     << "pins: " << c.pins << "\n"
     << "config: " << m25::describe(cfg) << "\n"
     << "state: " << w.js.describe() << (c.during ? " (changed during the tick: see pins)" : "") << "\n"
     << "paths: test/data/m3_5/paths.yaml\n"
     << "--- tick: " << c.tree << "\n"
     << "events:\n" << w.log.events()
     << "bt: " << bt << "\n"
     << "fault: " << fault << "\n"
     << "counts: " << w.log.counts() << "\n\n";
  const std::string text = os.str();
  family().add(text);

  EXPECT_EQ(bt, c.bt) << c.id << "\n" << text;
  EXPECT_EQ(fault, c.fault.empty() ? "<unset>" : "\"" + c.fault + "\"") << c.id << "\n" << text;
  if (c.no_events) { EXPECT_EQ(lineValue(text, "counts: "), "none") << c.id << ": nothing may reach MoveIt\n" << text; }
}

std::string run(const std::string& path, const std::string& from, const std::string& to)
{
  return "<FollowPath path=\"" + path + "\"" + (from.empty() ? "" : " from=\"" + from + "\"") +
         (to.empty() ? "" : " to=\"" + to + "\"") + " fault=\"{f}\"/>";
}

// the arm reaches `state` 300 ms into the tick (after the fake's sequence goal has executed)
std::function<void(m25::World&)> arrives(std::size_t state)
{
  return [state](m25::World& w) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    w.js.set(at(state));
  };
}
}  // namespace

// --- F: a run is the same path --------------------------------------------------------------------

TEST(M35FollowPathRange, F_a_run_to_an_event_state_is_one_goal_with_its_segments_and_confirms_the_arrival)
{
  runCase({"RNG-01", "inspect [0..1]: ONE sequence goal with segment 0 only (LIN to state 1, blend 0, the "
           "process profile's 0.05); the run ends at an event state, the arm arrives 0.3 s in: SUCCESS",
           run("inspect", "0", "1"), at(0), arrives(1), "SUCCESS", ""});
  runCase({"RNG-02", "inspect [1..2]: the start check at states[1] (the arm is there), ONE goal with segment 1 "
           "only", run("inspect", "1", "2"), at(1), arrives(2), "SUCCESS", ""});
  runCase({"RNG-03", "inspect [2..3]: the last run, segment 2 only; state 3 is an event state too",
           run("inspect", "2", "3"), at(2), arrives(3), "SUCCESS", ""});
}

TEST(M35FollowPathRange, F_a_one_state_run_sends_nothing_and_confirms_the_robot_is_there)
{
  runCase({"RNG-04", "inspect [1..1]: nothing sent, the robot is at state 1: SUCCESS", run("inspect", "1", "1"),
           at(1), {}, "SUCCESS", "", true});
  runCase({"RNG-05", "inspect [1..1] with the arm at state 0: NOT_AT_PATH_START after the bounded settle, nothing "
           "sent", run("inspect", "1", "1"), at(0), {}, "FAILURE", "MOTION_FAULT: NOT_AT_PATH_START during request",
           true});
}

TEST(M35FollowPathRange, F_the_runs_of_a_split_path_send_what_the_whole_path_sends)
{
  runCase({"RNG-06", "glue_whole (no events), whole: ONE goal, items blend 0.02 / 0 / 0 -- the reference",
           run("glue_whole", "", ""), at(0), {}, "SUCCESS", ""});
  runCase({"RNG-07", "glue [0..2] (an event before departing state 2): ONE goal, items blend 0.02 / 0 -- "
           "RNG-06's first two items", run("glue", "0", "2"), at(0), arrives(2), "SUCCESS", ""});
  runCase({"RNG-08", "glue [2..3]: ONE goal, RNG-06's last item", run("glue", "2", "3"), at(2), {}, "SUCCESS", ""});
}

// --- G: the boundaries ------------------------------------------------------------------------------

TEST(M35FollowPathRange, G_a_run_never_drives_through_an_event_state)
{
  runCase({"RNG-09", "inspect whole (no ports): it would drive through the event states 1 and 2: PATH_RANGE, "
           "nothing sent", run("inspect", "", ""), at(0), {}, "FAILURE", "MOTION_FAULT: PATH_RANGE during request",
           true});
  runCase({"RNG-10", "inspect [0..2]: through state 1: PATH_RANGE, nothing sent", run("inspect", "0", "2"), at(0),
           {}, "FAILURE", "MOTION_FAULT: PATH_RANGE during request", true});
  runCase({"RNG-11", "glue whole: through state 2 (before_departure): PATH_RANGE", run("glue", "", ""), at(0), {},
           "FAILURE", "MOTION_FAULT: PATH_RANGE during request", true});
}

TEST(M35FollowPathRange, G_a_run_names_states_of_a_segment_path_in_order)
{
  for (const auto& [id, from, to] : std::vector<std::tuple<std::string, std::string, std::string>>{
         {"RNG-12", "2", "1"}, {"RNG-13", "0", "4"}, {"RNG-14", "-1", "1"}, {"RNG-15", "x", "1"},
         {"RNG-16", "0", "1.5"}, {"RNG-22", "+0", "1"}, {"RNG-23", " 0", "1"}})
    runCase({id, "inspect [" + from + ".." + to + "]: not a run of the path: PATH_RANGE, nothing sent",
             run("inspect", from, to), at(0), {}, "FAILURE", "MOTION_FAULT: PATH_RANGE during request", true});
  runCase({"RNG-17", "a dense path with from/to: it runs whole: PATH_RANGE", run("dense", "0", "1"), at(0), {},
           "FAILURE", "MOTION_FAULT: PATH_RANGE during request", true});
  runCase({"RNG-18", "blended [0..1]: segment 0 blends into state 1, a run cannot end there: PATH_RANGE",
           run("blended", "0", "1"), at(0), {}, "FAILURE", "MOTION_FAULT: PATH_RANGE during request", true});
  runCase({"RNG-19", "only `to`: from defaults to 0 -- inspect [..1]", run("inspect", "", "1"), at(0), arrives(1),
           "SUCCESS", ""});
}

TEST(M35FollowPathRange, G_a_run_ending_at_an_event_state_fails_when_the_arm_is_not_there)
{
  runCase({"RNG-20", "inspect [0..1], the arm never leaves state 0 (the fake executes without moving it): the "
           "goal was sent and succeeded, then the arrival check fails after the bounded settle: NOT_AT_PATH_STATE",
           run("inspect", "0", "1"), at(0), {}, "FAILURE", "MOTION_FAULT: NOT_AT_PATH_STATE during execute"});
  runCase({"RNG-21", "glue_whole [1..2]: no event at state 2 (and no blend into it), so no arrival check: "
           "SUCCESS with the arm still at state 0", run("glue_whole", "1", "2"), at(1), {}, "SUCCESS", ""});
}

TEST(M35Family, zz_the_family_is_the_committed_record)
{
  family().check();
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
