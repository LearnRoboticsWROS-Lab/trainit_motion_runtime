// vNext M2.5 test harness -- the test arm, the rig that runs real BT nodes on a real
// MoveItMotionRuntime against the fakes, the scenario runner, and the isolated process start.
//
// The arm is INLINE: TMR's tests never depend on a demo or reference application (TrainIt
// invariant). Its URDF root "world" sits 0.1 m below base_link on purpose: the robot model frame
// (getCurrentPose, namedPose FK) and the pose reference frame (base_frame) then DIFFER, so a frame
// change shows up as numbers. No collision geometry: validation is collision-trivial.
//
// The rig mirrors trainit_run_bt: ONE long-lived runtime node (also the BtContext node, so the
// process-static caches of ServoTrack, RunPolicy and CheckRobotState stay bound to a live node),
// spun by a 2-thread MultiThreadedExecutor started BEFORE the runtime, and a FRESH
// MoveItMotionRuntime per scenario (the previous one destroyed first: MoveIt caches the robot model
// and the current-state monitor per process). Ticks run on the gtest main thread.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_5_TEST_ARM_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_5_TEST_ARM_HPP_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <cxxabi.h>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"

#include "capture.hpp"
#include "fake_move_group.hpp"

namespace m25
{

// --- the test arm ----------------------------------------------------------------------------------
inline const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="m25_test_arm">
  <link name="world"/><link name="base_link"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="tcp"/><link name="suction_link"/>
  <joint name="world_to_base" type="fixed"><parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/></joint>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="1.0"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="10" velocity="1.0"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="10" velocity="1.0"/></joint>
  <joint name="tcp_joint" type="fixed"><parent link="l3"/><child link="tcp"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/></joint>
  <joint name="suction_joint" type="fixed"><parent link="tcp"/><child link="suction_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/></joint>
</robot>)";

// "arm" is a CHAIN: only then is MoveGroupInterface's end-effector link non-empty.
inline const char* kSrdf = R"(<?xml version="1.0"?>
<robot name="m25_test_arm">
  <group name="arm"><chain base_link="base_link" tip_link="tcp"/></group>
  <group name="eef"><joint name="suction_joint"/></group>
  <group_state name="home" group="arm">
    <joint name="j1" value="0"/><joint name="j2" value="0"/><joint name="j3" value="0"/>
  </group_state>
  <group_state name="ready" group="arm">
    <joint name="j1" value="0.3"/><joint name="j2" value="-0.4"/><joint name="j3" value="0.6"/>
  </group_state>
  <end_effector name="suction" parent_link="tcp" group="eef" parent_group="arm"/>
</robot>)";

inline std::vector<rclcpp::Parameter> armParameters()
{
  std::vector<rclcpp::Parameter> p{
    {"robot_description", std::string(kUrdf)},
    {"robot_description_semantic", std::string(kSrdf)},
    {"use_sim_time", false},
    {"robot_description_planning.default_velocity_scaling_factor", 0.1},
    {"robot_description_planning.default_acceleration_scaling_factor", 0.1},
  };
  for (const char* j : {"j1", "j2", "j3"})
  {
    const std::string k = std::string("robot_description_planning.joint_limits.") + j;
    p.emplace_back(k + ".has_velocity_limits", true);
    p.emplace_back(k + ".max_velocity", 1.0);
    p.emplace_back(k + ".has_acceleration_limits", true);
    p.emplace_back(k + ".max_acceleration", 2.0);
  }
  return p;
}

using Config = trainit::MoveItMotionRuntime::Config;

// The runner's defaults for this arm (planner_mode ompl: the golden launch's mode).
inline Config defaultConfig()
{
  Config c;
  c.group = "arm";
  c.base_frame = "base_link";
  c.tip_link = "tcp";
  c.mode = trainit::PlannerMode::OMPL;
  c.process_backend = trainit::ProcessPathBackend::PILZ_SEQUENCE;
  c.available_pipelines = {"ompl", "pilz_industrial_motion_planner", "chomp"};
  c.enforce_validation = true;
  return c;
}

inline std::string describe(const Config& c)
{
  std::ostringstream os;
  os << "group=" << c.group << " base=" << c.base_frame << " tip=" << c.tip_link
     << " mode=" << trainit::toString(c.mode) << " backend=" << trainit::toString(c.process_backend)
     << " pipelines=[";
  for (std::size_t i = 0; i < c.available_pipelines.size(); ++i)
    os << (i ? "," : "") << c.available_pipelines[i];
  os << "] validation=" << (c.enforce_validation ? "on" : "off") << " retries=" << c.plan_retries
     << " max_jump=" << num17(c.max_joint_jump) << " frac=" << num17(c.cartesian_fraction_threshold)
     << " partial=" << (c.allow_partial_cartesian ? "on" : "off");
  return os.str();
}

inline geometry_msgs::msg::Pose pose(double x, double y, double z, double qx, double qy, double qz,
                                     double qw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  p.orientation.x = qx; p.orientation.y = qy; p.orientation.z = qz; p.orientation.w = qw;
  return p;
}

inline std::string describe(const geometry_msgs::msg::Pose& p)
{
  return "(" + num17(p.position.x) + "," + num17(p.position.y) + "," + num17(p.position.z) + " | " +
         num17(p.orientation.x) + "," + num17(p.orientation.y) + "," + num17(p.orientation.z) + "," +
         num17(p.orientation.w) + ")";
}

inline std::string demangle(const char* name)
{
  int status = 0;
  std::unique_ptr<char, void (*)(void*)> r{abi::__cxa_demangle(name, nullptr, nullptr, &status),
                                           std::free};
  return status == 0 ? std::string(r.get()) : std::string(name);
}

// vNext M2.7 (owner decision 11.2): from TMR 2.1.0 the scene nodes write BtContext::scene, which the
// runner sets to &runtime->scene(). Wired only where the field exists, so this header still builds
// against the frozen production sources the re-freeze procedures use (9cfb601, 1dda503, adf94b0),
// where it does nothing.
template <typename Ctx, typename = void>
struct HasSceneField : std::false_type {};
template <typename Ctx>
struct HasSceneField<Ctx, std::void_t<decltype(std::declval<Ctx&>().scene)>> : std::true_type {};

template <typename Ctx>
inline void wireScene(Ctx& ctx, trainit::MoveItMotionRuntime* runtime)
{
  if constexpr (HasSceneField<Ctx>::value)
    ctx.scene = runtime ? &runtime->scene() : nullptr;
  else
    (void)runtime;
}

// THE ADAPTER: the one place a test builds a BtContext. Whatever M2.5 does to BtContext, only
// this function may follow -- every captured request must stay the same.
inline trainit::BtContext makeContext(trainit::MoveItMotionRuntime* runtime,
                                      const rclcpp::Node::SharedPtr& node,
                                      trainit::GripperController* gripper = nullptr)
{
  trainit::BtContext ctx{};
  ctx.runtime = runtime;
  ctx.node = node;
  ctx.gripper = gripper;
  wireScene(ctx, runtime);   // the runner's ctx.scene = &runtime->scene() (TMR >= 2.1.0)
  return ctx;
}

// --- the rig ---------------------------------------------------------------------------------------
class Rig
{
public:
  explicit Rig(const std::string& ns = "/")
  {
    node_ = std::make_shared<rclcpp::Node>(
      "trainit_run_bt", ns,
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                           .parameter_overrides(armParameters()));
    profiles_ = trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE);
    startSpinning();
  }

  ~Rig() { teardown(); }

  // the judge's teardown order: stop spinning, destroy the runtime, then the node
  void teardown()
  {
    stopSpinning();
    runtime_.reset();
  }

  // A FRESH runtime the way trainit_run_bt builds it (the spinner already runs).
  void build(const Config& cfg)
  {
    stopSpinning();
    runtime_.reset();
    startSpinning();
    runtime_ = std::make_unique<trainit::MoveItMotionRuntime>(node_, cfg);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));   // its clients get discovered
  }

  void destroyRuntime()
  {
    stopSpinning();
    runtime_.reset();
    startSpinning();
  }

  // One tick of a tree, the way trainit_run_bt ticks (tickOnce while RUNNING, 10 ms apart).
  // Any std::exception is the result "THROW <dynamic type>: <what>".
  std::string tick(const std::string& xml, const BT::Blackboard::Ptr& bb)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    ctx_ = makeContext(runtime_.get(), node_, gripper_);
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES, profiles_);
    try
    {
      factory.registerBehaviorTreeFromText(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + xml +
        "</BehaviorTree></root>");
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
      return "THROW " + demangle(typeid(e).name()) + ": " + e.what();
    }
  }

  trainit::MoveItMotionRuntime* runtime() { return runtime_.get(); }
  rclcpp::Node::SharedPtr node() const { return node_; }
  // the BtContext's gripper for the next ticks (Close/OpenGripper); null = none configured
  void setGripper(trainit::GripperController* gripper) { gripper_ = gripper; }

private:
  void startSpinning()
  {
    spin_ = std::make_unique<Spinner<rclcpp::executors::MultiThreadedExecutor>>(rclcpp::ExecutorOptions(), 2);
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

  rclcpp::Node::SharedPtr node_;
  trainit::MotionProfileRegistry profiles_;
  std::unique_ptr<trainit::MoveItMotionRuntime> runtime_;
  trainit::BtContext ctx_{};
  trainit::GripperController* gripper_{nullptr};
  std::unique_ptr<Spinner<rclcpp::executors::MultiThreadedExecutor>> spin_;
};

// --- scenarios -------------------------------------------------------------------------------------
struct Scenario
{
  std::string id{};
  std::string pins{};                                      // one line: what this scenario pins
  std::function<void(Config&)> config{};                   // edits the default config
  std::vector<std::pair<std::string, std::string>> strings{};               // blackboard strings
  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> poses{};    // blackboard poses
  std::vector<std::string> trees{};                        // ticked in order on ONE runtime
  std::function<void(FakeMoveGroup&)> script{};            // replies, rules
  std::function<void()> before_tick{};                     // e.g. drop a server
  std::function<void()> after{};                           // e.g. restore it
  std::vector<double> q{0.0, 0.0, 0.0};                    // joint_states positions
  bool joint_states{true};
  int budget_s{90};
  std::function<std::string(BT::Blackboard&)> report{};    // extra lines after each tick
};

// Everything a binary shares, ONE per process: the log, the fakes in the root namespace, the
// joint states, the rig. One per process because ServoTrack's and RunPolicy's static clients,
// publishers and feeds bind to the FIRST node they see and must outlive every test.
// Destroyed (members in reverse order: rig first, then the joint states, then the fakes) by
// runAll() before rclcpp::shutdown().
struct World
{
  explicit World(const std::string& ns = "/")
  : mg(log, "/"), js(ns), rig(ns), ns_(ns) {}
  Log log;
  FakeMoveGroup mg;
  FakeJointStates js;
  Rig rig;
  Watchdog dog;
  std::string ns_;
};

inline std::unique_ptr<World>& worldSlot()
{
  static std::unique_ptr<World> w;
  return w;
}
inline std::string& worldNamespace()
{
  static std::string ns = "/";
  return ns;
}
inline World& world()
{
  if (!worldSlot()) worldSlot() = std::make_unique<World>(worldNamespace());
  return *worldSlot();
}

// A binary with its own per-process fixture registers how to destroy it; runAll() runs these
// before it destroys the World and before rclcpp::shutdown().
inline std::vector<std::function<void()>>& teardowns()
{
  static std::vector<std::function<void()>> t;
  return t;
}

// A tree names test data as ${PATHS}/<file>: the capture keeps the placeholder (a capture must
// not depend on where the source tree is checked out), the tick gets the real directory.
inline std::string substitute(std::string text)
{
#ifdef TRAINIT_M25_PATHS_DIR
  const std::string key = "${PATHS}";
  for (auto at = text.find(key); at != std::string::npos; at = text.find(key))
    text.replace(at, key.size(), TRAINIT_M25_PATHS_DIR);
#endif
  return text;
}

inline std::string blackboardLine(const Scenario& s)
{
  std::vector<std::string> kv;
  for (const auto& [k, v] : s.strings) kv.push_back(k + "=" + v);
  for (const auto& [k, p] : s.poses) kv.push_back(k + "=Pose" + describe(p));
  std::sort(kv.begin(), kv.end());
  std::string line;
  for (const auto& e : kv) line += (line.empty() ? "" : "; ") + e;
  return line.empty() ? "(none)" : line;
}

// Asynchronous topics (a PlanningSceneInterface removal, a trajectory_execution_event) are not
// ordered with the tick: wait at least 150 ms, then until no new topic message has arrived for
// 300 ms (at most 3 s), so a slow machine does not lose one.
inline void settleTopics(const Log& log)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  std::size_t seen = log.topicCount();
  auto quiet_since = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline &&
         std::chrono::steady_clock::now() - quiet_since < std::chrono::milliseconds(300))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const std::size_t now = log.topicCount();
    if (now != seen) { seen = now; quiet_since = std::chrono::steady_clock::now(); }
  }
}

// Runs one scenario on a fresh runtime and returns its capture block. `mg` is the fake whose
// scripts the scenario writes; `js` the joint states it sets.
inline std::string run(Log& log, FakeMoveGroup& mg, FakeJointStates& js, Rig& rig, Watchdog& dog,
                       const std::string& ns, const Scenario& s)
{
  Config cfg = defaultConfig();
  if (s.config) s.config(cfg);
  js.set(s.q);
  js.on(s.joint_states);
  mg.resetScenario();
  if (s.script) s.script(mg);

  dog.arm("build " + s.id, std::chrono::seconds(60));
  rig.build(cfg);
  dog.disarm();

  std::ostringstream os;
  os << "=== scenario " << s.id << "\n"
     << "pins: " << s.pins << "\n"
     << "config: ns=" << ns << " " << describe(cfg) << " profiles=config/bt/trainit_profiles.yaml\n"
     << "state: " << js.describe() << "\n"
     << "blackboard: " << blackboardLine(s) << "\n"
     << "script: " << mg.scriptText() << "\n";

  auto bb = BT::Blackboard::create();
  for (const auto& [k, v] : s.strings) bb->set<std::string>(k, v);
  for (const auto& [k, p] : s.poses) bb->set<geometry_msgs::msg::Pose>(k, p);

  if (s.before_tick) s.before_tick();
  for (std::size_t i = 0; i < s.trees.size(); ++i)
  {
    log.reset();
    dog.arm(s.id + " tick " + std::to_string(i + 1), std::chrono::seconds(s.budget_s));
    const std::string bt = rig.tick(substitute(s.trees[i]), bb);
    settleTopics(log);                                             // asynchronous topics land
    dog.disarm();
    os << "--- tick " << (i + 1) << ": " << s.trees[i] << "\n"
       << "events:\n" << log.events();
    const std::string topics = log.topics();
    if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;
    os << "bt: " << bt << "\n";
    if (s.report) os << s.report(*bb);
    os << "planner_mode_after: " << trainit::toString(rig.runtime()->plannerMode()) << "\n"
       << "counts: " << log.counts() << "\n";
  }
  if (s.after) s.after();
  os << "\n";
  return os.str();
}

inline std::string run(World& w, const Scenario& s)
{
  return run(w.log, w.mg, w.js, w.rig, w.dog, w.ns_, s);
}

// --- the isolated process --------------------------------------------------------------------------------
// The private domain is set PROGRAMMATICALLY (authoritative even from a shell exporting another
// ROS_DOMAIN_ID), localhost only; the ament ENV is a second guard. Fail closed: after discovery,
// any node this process did not create (a live move_group, a controller manager, a servo node...)
// is a FAILURE -- these tests must never be able to reach a robot.
inline int runAll(int argc, char** argv, std::size_t domain, const std::vector<const char*>& extra_args = {})
{
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  ::testing::InitGoogleTest(&argc, argv);
  std::vector<const char*> args{argv[0]};
  for (const char* a : extra_args) args.push_back(a);
  rclcpp::InitOptions options;
  options.set_domain_id(domain);
  rclcpp::init(static_cast<int>(args.size()), args.data(), options);
  {
    auto probe = std::make_shared<rclcpp::Node>("m25_graph_guard", "/",
                                                rclcpp::NodeOptions().use_global_arguments(false));
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::vector<std::string> foreign;
    for (const auto& n : probe->get_node_names())
      if (n != "/m25_graph_guard" && n.rfind("/_", 0) != 0) foreign.push_back(n);
    if (!foreign.empty())
    {
      std::fprintf(stderr, "M2.5 GRAPH GUARD: foreign nodes on private domain %zu:", domain);
      for (const auto& n : foreign) std::fprintf(stderr, " %s", n.c_str());
      std::fprintf(stderr, " -- refusing to run (no test may reach a robot)\n");
      rclcpp::shutdown();
      return 4;
    }
  }
  const int rc = RUN_ALL_TESTS();
  // A hang in teardown (MoveIt's destructors join their own threads) is a named failure too.
  std::thread([] {
    std::this_thread::sleep_for(std::chrono::seconds(60));
    std::fprintf(stderr, "\nM2.5 WATCHDOG: teardown did not finish in 60 s\n");
    std::fflush(stderr);
    std::_Exit(5);
  }).detach();
  for (auto& t : teardowns()) t();
  worldSlot().reset();
  rclcpp::shutdown();
  return rc;
}

}  // namespace m25

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_5_TEST_ARM_HPP_
