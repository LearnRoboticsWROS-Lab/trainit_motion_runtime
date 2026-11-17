// vNext M4.5d (ADR-0024 amendment M4.5d, owner-approved 2026-11-11): a motion that fails while a part is HELD is a
// FAULT whatever its cause. The REAL MoveWaypoint on a REAL MoveItMotionRuntime against the M2.5 fake move_group
// (test/m2_5/), as test_m2_7_motion_fault.cpp; every case is the SAME MoveIt script ticked as
//   <MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/>     with h = "true" / "false" / bool true
//   <MoveWaypoint waypoint="a" fault="{f}"/>                     (2.8: the port not remapped)
// What MoveIt receives must be identical whatever `holding` says (the port changes no traffic and no BT status);
// only the fault differs:
//   HELD-01  a PLANNING failure (PLANNING_FAILED: the target's, a verdict) while held  -> a FAULT "... while holding"
//   HELD-02  NO_IK_SOLUTION (the target's) while held                                  -> a FAULT
//   HELD-03  an EXECUTION failure (CONTROL_FAILED, already a fault) while held         -> the fault, "while holding"
//   HELD-04  a planning failure NOT held (holding false, and the port absent)          -> no fault (a verdict)
//   HELD-05  a SUCCESS while held                                                       -> nothing written
//   HELD-06  holding written as a BOOL by a Script (true)                               -> read as held
//   HELD-07  holding remapped to a key that holds no readable value                     -> fail closed: HELD
//   HELD-08  holding remapped to a key that is ABSENT (a tree that never set it)          -> fail closed: HELD;
//            an EMPTY literal                                                           -> refused when the tree is built
//   HELD-09  the held fault with `fault` NOT remapped                                     -> FAILURE, nothing written
//   HELD-10  TSA 8.8.0's OWN idiom: the MainTree reset Script, a gripping node's `_onSuccess` post-condition, then
//            the motion reading `holding="{trainit__holding}"` (BehaviorTree.CPP's post-conditions on THIS version)
// FollowPath carries the same port and the same rule (its own tick, an inline one-segment path library):
//   HELD-11  a refusal (no path library) while held                                     -> the fault, "while holding"
//   HELD-12  the Pilz sequence answers PLANNING_FAILED (the target's) while held          -> a FAULT "... while holding"
//   HELD-13  the same failure not held / the port absent                                -> no fault, same traffic
//   HELD-14  CONTROL_FAILED on the sequence while held                                  -> the fault, "while holding"
// Domain 98 (INTEGRATION 3.2), set in code and in CMake, localhost only, graph-guarded (m25::runAll).
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/path_library.hpp"

using m25::Reply;
using m25::Scenario;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

Strings data(const std::string& holding)
{
  Strings out = {{"a.type", "tcp"}, {"a.position", "0.4;0.1;0.3"}, {"a.orientation", "1;0;0;0"},
                 {"a.motion", "free"}, {"a.planner", "ompl"}, {"f", ""}};
  if (!holding.empty()) out.emplace_back("h", holding);
  return out;
}

std::function<void(m25::FakeMoveGroup&)> plans(int n, const Reply& r)
{
  return [n, r](m25::FakeMoveGroup& mg) { for (int i = 0; i < n; ++i) mg.mg_script.push_back(r); };
}

std::string reportFault(BT::Blackboard& bb)
{
  try { return "fault: \"" + bb.get<std::string>("f") + "\"\n"; }
  catch (const std::exception& e) { return std::string("fault: (unreadable: ") + e.what() + ")\n"; }
}

std::string lineValue(const std::string& block, const std::string& key)
{
  const auto a = block.find("\n" + key);
  if (a == std::string::npos) return "(no " + key + ")";
  const auto b = block.find('\n', a + 1 + key.size());
  return block.substr(a + 1 + key.size(), b - a - 1 - key.size());
}

std::string eventsOf(const std::string& block)
{
  const auto a = block.find("\nevents:\n");
  const auto b = block.find("\nbt: ");
  return (a == std::string::npos || b == std::string::npos) ? "(none)" : block.substr(a, b - a);
}

struct Case
{
  std::string id, pins;
  Strings bb;
  std::function<void(m25::FakeMoveGroup&)> script;
  std::string tree;
  std::string bt;
  std::string fault;
};

std::string run(const Case& c)
{
  m25::World& w = m25::world();
  const std::string out = m25::run(w, Scenario{c.id, c.pins, nullptr, c.bb, {}, {c.tree}, c.script, nullptr, nullptr,
                                               {0.0, 0.0, 0.0}, true, 90, reportFault});
  EXPECT_EQ(lineValue(out, "bt: "), c.bt) << c.id;
  EXPECT_EQ(lineValue(out, "fault: "), "\"" + c.fault + "\"") << c.id;
  return out;
}

const char* kHeld = R"(<MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/>)";
const char* kPlain = R"(<MoveWaypoint waypoint="a" fault="{f}"/>)";
}  // namespace

TEST(M45dHeldMotionFault, a_target_failure_while_held_is_a_fault)
{
  const auto pf = plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED));
  const std::string held = run({"HELD-01", "PLANNING_FAILED, held", data("true"), pf, kHeld, "FAILURE",
                                "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  const std::string free_ = run({"HELD-04a", "PLANNING_FAILED, not held", data("false"), pf, kHeld, "FAILURE", ""});
  const std::string plain = run({"HELD-04b", "PLANNING_FAILED, the 2.8 tree", data(""), pf, kPlain, "FAILURE", ""});
  // the port changes nothing MoveIt receives
  EXPECT_EQ(eventsOf(held), eventsOf(free_));
  EXPECT_EQ(eventsOf(held), eventsOf(plain));
  run({"HELD-02", "NO_IK_SOLUTION, held", data("true"), plans(4, Reply::abort(m25::ErrorCodes::NO_IK_SOLUTION)),
       kHeld, "FAILURE", "MOTION_FAULT: NO_IK_SOLUTION during plan while holding"});
}

TEST(M45dHeldMotionFault, an_execution_failure_while_held_says_so)
{
  auto script = [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::abort(m25::ErrorCodes::CONTROL_FAILED)); };
  run({"HELD-03", "CONTROL_FAILED on execution, held", data("true"), script, kHeld, "FAILURE",
       "MOTION_FAULT: CONTROL_FAILED during execute while holding"});
  run({"HELD-03b", "CONTROL_FAILED on execution, not held (2.8: already a fault)", data("false"), script, kHeld,
       "FAILURE", "MOTION_FAULT: CONTROL_FAILED during execute"});
}

TEST(M45dHeldMotionFault, success_and_the_ways_holding_is_written)
{
  run({"HELD-05", "a success while held writes nothing", data("true"),
       nullptr,
       kHeld, "SUCCESS", ""});
  // a canonical tree writes the key with a Script: a BOOL value
  const std::string script_tree =
      R"(<Sequence><Script code="h:=true"/><MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/></Sequence>)";
  run({"HELD-06", "holding written as a bool by a Script", data(""),
       plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED)), script_tree, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  // remapped to a key holding a value that is not a boolean: fail closed (HELD)
  run({"HELD-07", "holding unreadable ('maybe')", data("maybe"),
       plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED)), kHeld, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
}

TEST(M45dHeldMotionFault, an_unset_holding_key_is_held_and_an_unremapped_fault_is_not_written)
{
  const auto pf = plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED));
  // the key is remapped but nothing ever wrote it: a tree that forgot its initialiser fails CLOSED
  run({"HELD-08a", "holding remapped to an absent key", data(""), pf, kHeld, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  // an EMPTY LITERAL never runs: BT.CPP converts a literal when the tree is built and refuses it (fail loud)
  const std::string empty = run({"HELD-08b", "holding given as an empty literal", data(""), pf,
                                 R"(<MoveWaypoint waypoint="a" fault="{f}" holding=""/>)",
                                 "THROW BT::LogicError: The port with name \"holding\" and value \"\" can not be "
                                 "converted to bool", ""});
  EXPECT_EQ(lineValue(empty, "counts: "), "none") << "nothing may reach MoveIt:\n" << empty;
  // `fault` not remapped: the status is FAILURE as ever, the blackboard's f stays as written (""), no throw
  run({"HELD-09", "held, fault not remapped", data("true"), pf, R"(<MoveWaypoint waypoint="a" holding="{h}"/>)",
       "FAILURE", ""});
}

TEST(M45dHeldMotionFault, the_tsa_generated_idiom_drives_the_real_node)
{
  const auto pf = plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED));
  // as task_program.py emits it: reset false; a node whose SUCCESS means "now holding" (CloseGripper in a program;
  // here a Script stands in for it: the post-condition is BehaviorTree.CPP's, not the gripper's); then the move
  const std::string tsa = R"(<Sequence><Script code="trainit__holding := false"/>)"
                          R"(<Script code="x := 1" _onSuccess="trainit__holding := true"/>)"
                          R"(<MoveWaypoint waypoint="a" fault="{f}" holding="{trainit__holding}"/></Sequence>)";
  run({"HELD-10a", "TSA idiom: held after the post-condition", data(""), pf, tsa, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  // ... and the same tree whose gripping step FAILED: _onFailure clears it, the move is free (a verdict)
  const std::string tsa_free = R"(<Sequence><Script code="trainit__holding := true"/>)"
                               R"(<ForceSuccess><Script code="x := 1" _failureIf="true" _onFailure="trainit__holding := false"/></ForceSuccess>)"
                               R"(<MoveWaypoint waypoint="a" fault="{f}" holding="{trainit__holding}"/></Sequence>)";
  run({"HELD-10b", "TSA idiom: cleared by _onFailure", data(""), pf, tsa_free, "FAILURE", ""});
}

// --- FollowPath: the same rule on its own tick ---------------------------------------------------------------
namespace
{
using trainit::PathLibrary;

// TF for the rig (as test_m3_1): world -> base_link, the test arm's fixed joint -- FollowPath reads the current pose
struct Tf
{
  explicit Tf(const rclcpp::Node::SharedPtr& node)
  {
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, true);
    pub_node = std::make_shared<rclcpp::Node>("m45d_static_tf", "/");
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

// one LIN segment from the test arm's start tcp pose (q = 0: (0.5, 0, 0.3), identity) -- the base frame, no TF
std::shared_ptr<const PathLibrary> onePath()
{
  return std::make_shared<const PathLibrary>(trainit::parsePathLibrary(
      "schema_version: 1\npaths:\n  move:\n    continuity: stop\n    states:\n"
      "      - {pose: {position: [0.5, 0.0, 0.3], orientation: [0.0, 0.0, 0.0, 1.0]}}\n"
      "      - {pose: {position: [0.5, 0.1, 0.3], orientation: [0.0, 0.0, 0.0, 1.0]}}\n"
      "    segments:\n      - {type: lin}\n",
      ".", trainit::PathRuntimeFacts{trainit::ProcessPathBackend::PILZ_SEQUENCE, "tcp"}));
}

struct PathCase
{
  std::string id;
  std::string tree;
  std::string holding;                       // "" = the key not set
  std::shared_ptr<const PathLibrary> paths;  // nullptr = no library
  std::function<void(m25::FakeMoveGroup&)> script;
  std::string bt, fault;
};

// m3_1's runner, reduced: the real FollowPath on the real runtime; returns the MoveIt traffic
std::string runPath(const PathCase& c)
{
  m25::World& w = m25::world();
  tf();
  m25::Config cfg = m25::defaultConfig();
  cfg.process_backend = trainit::ProcessPathBackend::PILZ_SEQUENCE;
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.mg.resetScenario();
  if (c.script) c.script(w.mg);
  w.dog.arm("build " + c.id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();

  auto bb = BT::Blackboard::create();
  bb->set<std::string>("f", "");
  if (!c.holding.empty()) bb->set<std::string>("h", c.holding);
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  trainit::BtContext ctx = m25::makeContext(w.rig.runtime(), w.rig.node());
  ctx.tf = tf().buffer;
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES,
                                          trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE));
  if (c.paths) bb->set<std::shared_ptr<const PathLibrary>>(trainit::BB_PATH_LIBRARY, c.paths);

  w.log.reset();
  w.dog.arm(c.id, std::chrono::seconds(120));
  std::string bt;
  try
  {
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">)" +
                                         c.tree + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    bt = BT::toStr(status);
  }
  catch (const std::exception& e) { bt = std::string("THROW ") + e.what(); }
  m25::settleTopics(w.log);
  w.dog.disarm();
  const std::string events = w.log.events();
  EXPECT_EQ(bt, c.bt) << c.id << "\n" << events;
  EXPECT_EQ(bb->get<std::string>("f"), c.fault) << c.id << "\n" << events;
  return events;
}

const char* kPathHeld = R"(<FollowPath path="move" fault="{f}" holding="{h}"/>)";
const char* kPathPlain = R"(<FollowPath path="move" fault="{f}"/>)";
}  // namespace

TEST(M45dHeldMotionFault, follow_path_carries_the_same_rule)
{
  runPath({"HELD-11", kPathHeld, "true", nullptr, nullptr, "FAILURE",
           "MOTION_FAULT: NO_PATH_LIBRARY during request while holding"});
  auto seq_pf = [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::abort(-1)); };
  const std::string held = runPath({"HELD-12", kPathHeld, "true", onePath(), seq_pf, "FAILURE",
                                    "MOTION_FAULT: PLANNING_FAILED during plan_and_execute while holding"});
  const std::string free_ = runPath({"HELD-13a", kPathHeld, "false", onePath(), seq_pf, "FAILURE", ""});
  const std::string plain = runPath({"HELD-13b", kPathPlain, "", onePath(), seq_pf, "FAILURE", ""});
  EXPECT_EQ(held, free_);
  EXPECT_EQ(held, plain);
  EXPECT_NE(held.find("sequence"), std::string::npos) << "the path reached MoveIt:\n" << held;
  runPath({"HELD-14", kPathHeld, "true", onePath(),
           [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::abort(m25::ErrorCodes::CONTROL_FAILED)); },
           "FAILURE", "MOTION_FAULT: CONTROL_FAILED during plan_and_execute while holding"});
}

int main(int argc, char** argv) { return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN); }
