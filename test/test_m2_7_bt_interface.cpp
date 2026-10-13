// vNext M2.7 (owner decision 11.2), AFTER: BtContext and TaskRunner depend on the MotionRuntime door.
//
//   1. the types, at compile time: BtContext::runtime is a MotionRuntime*, BtContext::scene the
//      runner's PlanningSceneManager*, TaskRunner is built from ANY MotionRuntime with a scene and
//      a logger, and the 2.0.0 constructor from the concrete runtime still exists;
//   2. task_runner_via_interface: the M2.5 task_runner family (test/data/m2_5/task_runner.txt,
//      frozen on 1.8.0), run through the NEW constructor with the runtime's scene and the node's
//      logger handed over (the way trainit_run_task builds it now), is byte-identical to the frozen
//      capture -- the M2.5 binary reproduces the same file through the compatibility constructor;
//   3. the_bt_nodes_need_only_the_door: MoveWaypoint (a LIN to a NAMED state: tipLink, namedPose)
//      and ExecuteProcessPath (a CSV path: baseFrame, tipLink) ticked on a recording fake runtime,
//      with no MoveIt endpoint at all: what each node asked the door, in order.
//
// The scene nodes are proven by test_m2_7_scene_nodes (frozen on adf94b0), which THE ADAPTER now
// runs through BtContext::scene. This file needs the M2.7 API: it is left out of a re-freeze copy.
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/task_loader.hpp"
#include "trainit_motion_runtime/task_runner.hpp"

using m25::Config;

// --- 1. the types --------------------------------------------------------------------------------
static_assert(std::is_same_v<decltype(trainit::BtContext::runtime), trainit::MotionRuntime*>,
              "BtContext::runtime is the MotionRuntime door");
static_assert(std::is_same_v<decltype(trainit::BtContext::scene), trainit::PlanningSceneManager*>,
              "BtContext::scene is the runner's planning scene");
static_assert(std::is_constructible_v<trainit::TaskRunner, trainit::MotionRuntime&,
                                      trainit::PlanningSceneManager&, rclcpp::Logger>,
              "TaskRunner is built from the door, a scene and a logger");
static_assert(std::is_constructible_v<trainit::TaskRunner, trainit::MoveItMotionRuntime&,
                                      trainit::GripperController*, trainit::ProcessController*>,
              "the 2.0.0 constructor stays (it delegates)");

namespace
{
// --- 2. the M2.5 runTask (test_m2_5_motion_requests.cpp), the construction line excepted ---------
std::string runTask(const std::string& id, const std::string& pins, const std::string& file,
                    bool plan_only)
{
  m25::World& w = m25::world();
  m25::FakeGripper gripper(w.log, "/", "m25_gripper");
  Config cfg = m25::defaultConfig();
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.mg.resetScenario();
  w.dog.arm("build " + id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();
  trainit::GripperController gc(w.rig.node(), "m25_gripper");
  // THE difference: the door, the runtime's scene and the node's logger, handed over
  trainit::TaskRunner runner(*w.rig.runtime(), w.rig.runtime()->scene(), w.rig.node()->get_logger(),
                             &gc, nullptr);
  trainit::MotionTask task = trainit::TaskLoader::fromYamlFile(std::string(TRAINIT_M25_TASKS_DIR) + "/" + file);
  if (plan_only)
    for (auto& step : task.steps) step.options.plan_only = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // the gripper client is discovered
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

// --- 3. a MotionRuntime with no MoveIt behind it: it records what it is asked -------------------
class RecordingRuntime : public trainit::MotionRuntime
{
public:
  mutable std::vector<std::string> calls;

  trainit::MotionResult movePtp(const trainit::JointTarget&, const trainit::MotionOptions&) override
  { return rec("movePtp"); }
  trainit::MotionResult moveLinear(const trainit::CartesianTarget& t, const trainit::MotionOptions&) override
  { return rec("moveLinear tip=" + t.tip_link + " pose=" + m25::describe(t.pose)); }
  trainit::MotionResult moveCircular(const trainit::CircularTarget&, const trainit::MotionOptions&) override
  { return rec("moveCircular"); }
  trainit::MotionResult moveCollisionFree(const trainit::MotionTarget&, const trainit::PlanningOptions&) override
  { return rec("moveCollisionFree"); }
  trainit::MotionResult executeProcessSequence(const std::vector<trainit::ProcessSegment>&,
                                               const trainit::MotionOptions&) override
  { return rec("executeProcessSequence"); }
  trainit::MotionResult executeCartesianProcess(const trainit::CartesianProcessPath& p,
                                                const trainit::MotionOptions&) override
  {
    return rec("executeCartesianProcess frame=" + p.reference_frame + " tcp=" + p.tcp_link +
               " waypoints=" + std::to_string(p.waypoints.size()));
  }
  trainit::MotionResult executeTrajectory(const moveit_msgs::msg::RobotTrajectory&) override
  { return rec("executeTrajectory"); }
  trainit::MotionResult stop() override { return rec("stop"); }
  void setPlannerMode(trainit::PlannerMode mode) override { mode_ = mode; }
  trainit::PlannerMode plannerMode() const override { return mode_; }
  geometry_msgs::msg::PoseStamped currentTipPose() const override
  {
    calls.push_back("currentTipPose");
    return geometry_msgs::msg::PoseStamped();
  }
  const std::string& tipLink() const override
  {
    calls.push_back("tipLink");
    return tip_;
  }
  const std::string& baseFrame() const override
  {
    calls.push_back("baseFrame");
    return base_;
  }
  std::optional<geometry_msgs::msg::Pose> namedPose(const std::string& named) const override
  {
    calls.push_back("namedPose(" + named + ")");
    if (named != "ready") return std::nullopt;
    return m25::pose(0.4, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0);
  }

private:
  trainit::MotionResult rec(const std::string& call) const
  {
    calls.push_back(call);
    return trainit::MotionResult::ok(call);
  }
  trainit::PlannerMode mode_{trainit::PlannerMode::OMPL};
  std::string tip_{"fake_tcp"};
  std::string base_{"fake_base"};
};

BT::NodeStatus tickOnce(trainit::BtContext& ctx, const std::string& xml,
                        const std::vector<std::pair<std::string, std::string>>& strings)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES,
                                          trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE));
  for (const auto& [k, v] : strings) bb->set<std::string>(k, v);
  factory.registerBehaviorTreeFromText(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + xml +
    "</BehaviorTree></root>");
  auto tree = factory.createTree("T", bb);
  return tree.tickOnce();
}
}  // namespace

TEST(M27BtInterface, task_runner_via_interface)
{
  m25::Family f("task_runner");
  f.add(runTask("TASK-01a", "every TaskRunner step kind, executed", "m2_5_task.yaml", false));
  f.add(runTask("TASK-01b", "the same task with plan_only on every step", "m2_5_task.yaml", true));
  f.add(runTask("TASK-02", "a CIRCULAR step given by its centre throws out of TaskRunner::run",
                "m2_5_task_center.yaml", false));
  f.check();
}

TEST(M27BtInterface, the_bt_nodes_need_only_the_door)
{
  auto node = std::make_shared<rclcpp::Node>("m27_door", "/",
                                             rclcpp::NodeOptions().use_global_arguments(false));
  RecordingRuntime fake;
  trainit::BtContext ctx{};
  ctx.runtime = &fake;   // no MoveGroupInterface, no planning scene: the door only
  ctx.node = node;

  EXPECT_EQ(tickOnce(ctx, "<MoveWaypoint waypoint=\"w\"/>",
                     {{"w.type", "joint"}, {"w.named", "ready"}, {"w.motion", "lin"}}),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake.calls, (std::vector<std::string>{
                          "tipLink", "namedPose(ready)",
                          "moveLinear tip=fake_tcp pose=" + m25::describe(m25::pose(0.4, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0))}));

  fake.calls.clear();
  EXPECT_EQ(tickOnce(ctx, std::string("<ExecuteProcessPath path_file=\"") + TRAINIT_M25_PATHS_DIR + "/p3.csv\"/>", {}),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake.calls, (std::vector<std::string>{
                          "baseFrame", "tipLink", "executeCartesianProcess frame=fake_base tcp=fake_tcp waypoints=3"}));

  // an unknown named state: the door says so, the node fails before any move
  fake.calls.clear();
  EXPECT_EQ(tickOnce(ctx, "<MoveWaypoint waypoint=\"w\"/>",
                     {{"w.type", "joint"}, {"w.named", "nowhere"}, {"w.motion", "lin"}}),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(fake.calls, (std::vector<std::string>{"tipLink", "namedPose(nowhere)"}));
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M27_DOMAIN);
}
