// vNext M2.5 (review): more BEFORE captures, frozen on TMR 1.8.0 (production == 9cfb601) by the
// re-freeze procedure of test/data/m2_5/README.md, after the M2.5 review found what the first
// captures could not tell apart:
//
//   state_reads_2  which link the read measures: a configured tip that is not the MoveGroup
//                  end-effector, and the read after a cartesian path moved MoveGroup's
//                  end-effector link (a read of MGI's DEFAULT link would differ in both)
//   servo_track_2  ServoTrack's feedback while the arm MOVES (it converges), with no state at all,
//                  and its arbitration watch OFF (port unset) with an execution really in flight
//   task_runner_2  TaskRunner's process path on the cartesian backend, executed and plan-only
//   golden_replay  the golden pick-and-place sequence's nodes and per-waypoint data, twice on one
//                  runtime: the planner choices and the mode drift a real cell's cycle carries
//
// Built and run unchanged on 1.8.0 (it uses no API M2.5 added).
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/task_loader.hpp"
#include "trainit_motion_runtime/task_runner.hpp"

using m25::Config;
using m25::Scenario;
using trainit::PlannerMode;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

const std::vector<double> kHome{0.0, 0.0, 0.0};
const std::vector<double> kReady{0.3, -0.4, 0.6};
const std::vector<double> kOther{-0.5, 0.25, -0.75};

std::function<std::string(BT::Blackboard&)> report(std::vector<std::string> keys)
{
  return [keys](BT::Blackboard& bb) {
    std::string s;
    for (const auto& k : keys)
    {
      try { s += "stored: " + k + "=Pose" + m25::describe(bb.get<geometry_msgs::msg::Pose>(k)) + "\n"; }
      catch (const std::exception&) { s += "stored: " + k + "=(none)\n"; }
    }
    return s;
  };
}

void family(const std::string& name, const std::vector<Scenario>& scenarios)
{
  m25::World& w = m25::world();
  m25::Family f(name);
  for (const auto& s : scenarios) f.add(m25::run(w, s));
  f.check();
}

std::function<void(Config&)> cartesianBackend()
{
  return [](Config& c) { c.process_backend = trainit::ProcessPathBackend::CARTESIAN_WAYPOINTS; };
}
}  // namespace

// --- which link a state read measures ------------------------------------------------------------
TEST(M25MoreCaptures, state_reads_2)
{
  family("state_reads_2", {
    {"READ-06", "a configured tip that is not the tcp: the read measures that link (l3), not tcp",
     [](Config& c) { c.tip_link = "l3"; }, {}, {},
     {"<StoreCurrentPose pose_key=\"cur\"/>", "<CheckRobotState position=\"0.3;0;0.4\"/>"},
     nullptr, nullptr, nullptr, kOther, true, 90, report({"cur"})},
    {"READ-07", "after a cartesian path left MoveGroup's end-effector link at l3, a read still measures the configured tcp",
     cartesianBackend(), {}, {},
     {"<StoreCurrentPose pose_key=\"before\"/>",
      "<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>",
      "<StoreCurrentPose pose_key=\"after\"/>",
      "<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" out_key=\"target\"/>"},
     nullptr, nullptr, nullptr, kOther, true, 90, report({"before", "after", "target"})},
  });
}

// --- ServoTrack: a moving arm, no state, arbitration with an execution in flight -------------------
namespace
{
std::unique_ptr<m25::FakeServo> g_servo;

Scenario servo(Scenario s, const std::string& node, std::function<void()> arrive = nullptr)
{
  m25::World& w = m25::world();
  s.before_tick = [&w, node, arrive, inner = s.before_tick] {
    g_servo = std::make_unique<m25::FakeServo>(w.log, "/", node);
    if (arrive) g_servo->onFirstNonzeroTwist(arrive);
    if (inner) inner();
  };
  auto inner_report = s.report;
  s.report = [inner_report](BT::Blackboard& bb) {
    std::string out = g_servo ? g_servo->twists() : std::string("twists: no servo node\n");
    if (inner_report) out += inner_report(bb);
    return out;
  };
  auto inner_after = s.after;
  s.after = [inner_after] { g_servo.reset(); if (inner_after) inner_after(); };
  return s;
}
}  // namespace

TEST(M25MoreCaptures, servo_track_2)
{
  m25::World& w = m25::world();
  using Execute = moveit_msgs::action::ExecuteTrajectory;
  auto client = rclcpp_action::create_client<Execute>(w.rig.node(), "/execute_trajectory");

  // FK(ready) of the test arm in the model frame (state_reads.txt READ-01)
  const Strings to_ready = {{"t.type", "tcp"},
                            {"t.position", "0.45123562560121699;0.13958353611373253;0.47709163653358294"}};

  Scenario s15 = servo({"SERVO-15", "arbitration OFF (port unset) streams although an execution IS in flight; "
                        "with the port set and the execution still in flight it refuses before any call",
                        nullptr, {{"t.type", "tcp"}, {"t.position", "0.5;0;0.4"}}, {},
                        {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s15\"/>",
                         "<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s15\" "
                         "execution_status_topic=\"/execute_trajectory/_action/status\"/>"}},
                       "m25_servo_s15");
  s15.before_tick = [&, outer = s15.before_tick] {
    outer();
    w.mg.ex_script.push_back(m25::Reply::hold());
    client->wait_for_action_server(std::chrono::seconds(5));
    Execute::Goal goal;
    goal.trajectory.joint_trajectory.joint_names = m25::kJoints;
    auto sent = client->async_send_goal(goal);
    sent.wait_for(std::chrono::seconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));   // the status array is latched
  };
  int s15_tick = 0;
  s15.report = [&, inner = s15.report](BT::Blackboard& bb) {
    const std::string out = inner(bb);
    if (++s15_tick == 2) w.mg.releaseHeld();
    return out;
  };

  m25::Family f("servo_track_2");
  for (const Scenario& s : std::vector<Scenario>{
    servo({"SERVO-13", "the arm ARRIVES after the first twist (the fake joint states jump to the target): "
           "the feedback read sees it and the move converges",
           nullptr, to_ready, {}, {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s13\"/>"},
           nullptr, nullptr, nullptr, kHome},
          "m25_servo_s13", [&w] { w.js.set(kReady); }),
    servo({"SERVO-14", "no joint state at all: the frame and the feedback come from the identity pose "
           "(KNOWN_DEBT: the silent identity read); the stream times out",
           nullptr, {{"t.type", "tcp"}, {"t.position", "0.05;0;0"}}, {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s14\" timeout_ms=\"500\"/>"},
           nullptr, nullptr, nullptr, kHome, false},
          "m25_servo_s14"),
    s15,
  })
  {
    f.add(m25::run(w, s));
  }
  f.check();
}

// --- TaskRunner on the cartesian backend ------------------------------------------------------------
namespace
{
std::string runTask(const std::string& id, const std::string& pins, const std::string& file,
                    bool plan_only)
{
  m25::World& w = m25::world();
  m25::FakeGripper gripper(w.log, "/", "m25_gripper");
  Config cfg = m25::defaultConfig();
  cfg.process_backend = trainit::ProcessPathBackend::CARTESIAN_WAYPOINTS;
  w.js.set(kHome);
  w.js.on(true);
  w.mg.resetScenario();
  w.dog.arm("build " + id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();
  trainit::GripperController gc(w.rig.node(), "m25_gripper");
  trainit::TaskRunner runner(*w.rig.runtime(), &gc, nullptr);
  trainit::MotionTask task =
    trainit::TaskLoader::fromYamlFile(std::string(TRAINIT_M25_TASKS_DIR) + "/" + file);
  if (plan_only)
    for (auto& step : task.steps) step.options.plan_only = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  w.log.reset();
  w.dog.arm(id, std::chrono::seconds(120));
  std::string result;
  try { result = runner.run(task) ? "true" : "false"; }
  catch (const std::exception& e) { result = "THROW " + m25::demangle(typeid(e).name()) + ": " + e.what(); }
  m25::settleTopics(w.log);
  w.dog.disarm();
  std::ostringstream os;
  os << "=== scenario " << id << "\n"
     << "pins: " << pins << "\n"
     << "config: ns=/ " << m25::describe(cfg) << "\n"
     << "state: " << w.js.describe() << "\n"
     << "task: " << file << (plan_only ? " (plan_only on every step)" : "") << "\n"
     << "events:\n" << w.log.events();
  const std::string topics = w.log.topics();
  if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;
  os << "run: " << result << "\n"
     << "planner_mode_after: " << trainit::toString(w.rig.runtime()->plannerMode()) << "\n"
     << "counts: " << w.log.counts() << "\n\n";
  return os.str();
}
}  // namespace

TEST(M25MoreCaptures, task_runner_2)
{
  m25::Family f("task_runner_2");
  f.add(runTask("TASK-03a", "process paths on the cartesian backend: a generated polyline and a YAML file in world/l3",
                "m2_5_task_cartesian.yaml", false));
  f.add(runTask("TASK-03b", "the same task with plan_only on every step: computeCartesianPath, no execute",
                "m2_5_task_cartesian.yaml", true));
  f.check();
}

// --- the golden cycle's nodes and data, twice on one runtime --------------------------------------
TEST(M25MoreCaptures, golden_replay)
{
  m25::World& w = m25::world();
  // The golden bundle's bt_params waypoints, in the form trainit_run_bt writes them (arrays joined
  // by ';' at ostream precision, integers by std::to_string); the vision-written pick pose is the
  // golden pick (SetWaypointFromDetection with orientation=keep keeps the params' orientation).
  const Strings golden = {
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
  const std::string cycle =
    "<Sequence>"
    "<SetWaypointRelative waypoint=\"post_place\" from=\"place\" dz=\"0.100\"/>"
    "<MoveWaypoint waypoint=\"ready\"/>"
    "<MoveWaypoint waypoint=\"pre_pick\"/>"
    "<SetWaypointRelative waypoint=\"post_pick\" from=\"pick\" dz=\"0.050\"/>"
    "<MoveWaypoint waypoint=\"pick\"/>"
    "<CloseGripper/>"
    "<SetAttachedCollisionCheck value=\"true\"/>"
    "<MoveWaypoint waypoint=\"post_pick\"/>"
    "<MoveWaypoint waypoint=\"pre_place\"/>"
    "<MoveWaypoint waypoint=\"place\"/>"
    "<SetReleasePolicy policy=\"freeze\"/>"
    "<OpenGripper/>"
    "<MoveWaypoint waypoint=\"post_place\"/>"
    "</Sequence>";
  std::unique_ptr<m25::FakeGripper> fake_gripper;
  std::unique_ptr<trainit::GripperController> gc;
  Scenario s{"T5-10", "the golden pick-and-place cycle (its nodes, without perception; its waypoint "
             "data verbatim), run twice on one runtime: tick 2 starts from the mode tick 1 left",
             nullptr, golden, {}, {cycle, cycle}};
  s.before_tick = [&] {
    fake_gripper = std::make_unique<m25::FakeGripper>(w.log, "/", "m25_gripper");
    gc = std::make_unique<trainit::GripperController>(w.rig.node(), "m25_gripper");
    gc->waitForServer(5.0);
    w.rig.setGripper(gc.get());
  };
  s.after = [&] { w.rig.setGripper(nullptr); gc.reset(); fake_gripper.reset(); };
  family("golden_replay", {s});
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
