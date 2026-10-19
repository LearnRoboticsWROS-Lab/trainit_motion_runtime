// =============================================================================
// trainit_run_bt_node.cpp — GENERIC BehaviorTree runner (no application logic).
//
// Loads a BT XML (param `bt_tree_file`) and ticks it, with the TrainIt runtime
// stack injected on the blackboard. The SAME executable runs any application a
// setup assistant generates — the application IS the tree + task_parameters, not
// this code. Mirrors trainit_run_task_node.cpp; coexists with it.
//
//   ros2 run trainit_motion_runtime trainit_run_bt --ros-args -p bt_tree_file:=<xml>
// (usually launched by the application package, which injects the MoveIt params).
// =============================================================================
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#ifdef BTCPP_GROOT2_SUPPORT
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/path_library.hpp"

using namespace trainit;

namespace
{
std::string getStr(const rclcpp::Node::SharedPtr& n, const std::string& name, const std::string& def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_string();
}
bool getBool(const rclcpp::Node::SharedPtr& n, const std::string& name, bool def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_bool();
}
int getInt(const rclcpp::Node::SharedPtr& n, const std::string& name, int def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return static_cast<int>(n->get_parameter(name).as_int());
}
double getDouble(const rclcpp::Node::SharedPtr& n, const std::string& name, double def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_double();
}
std::vector<std::string> getStrArray(const rclcpp::Node::SharedPtr& n, const std::string& name,
                                     const std::vector<std::string>& def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_string_array();
}

// vNext M3.4 (owner decision D8, the MINIMAL stop scope): when orchestration stops while the tree
// runs -- SIGINT/SIGTERM, or any rclcpp::shutdown() -- the motion in flight gets the runtime's stop
// request, MotionRuntime::stop() (move_group's execution "stop" event: it stops the execution of
// every route, the Pilz sequence included). A PRE-shutdown callback, so the context is still valid
// and the request can be published. Once per run; removed with this object. The motion nodes stay
// synchronous: the tick blocked inside the motion is NOT interrupted (a halt is not preemption, D8).
// The context is the global default one (rclcpp::init's, the one SIGINT shuts down). The callback
// runs on the signal handler's thread: the destructor disarms under the same mutex, so it returns
// only once an invocation in flight has finished (the runtime outlives this object).
class StopOnShutdown
{
public:
  StopOnShutdown(MotionRuntime& runtime, const rclcpp::Logger& log)
  : context_(rclcpp::contexts::get_global_default_context()), state_(std::make_shared<State>())
  {
    handle_ = context_->add_pre_shutdown_callback([state = state_, &runtime, log]() {
      std::lock_guard<std::mutex> lock(state->m);
      if (!state->armed) return;
      state->armed = false;
      RCLCPP_WARN(log, "shutdown requested while the tree runs: stopping the motion in flight "
                       "(MotionRuntime::stop)");
      const MotionResult r = runtime.stop();
      if (!r.success) RCLCPP_ERROR(log, "MotionRuntime::stop failed: %s", r.stage.c_str());
    });
  }
  ~StopOnShutdown()
  {
    context_->remove_pre_shutdown_callback(handle_);
    std::lock_guard<std::mutex> lock(state_->m);   // waits for an invocation in flight
    state_->armed = false;
  }
  StopOnShutdown(const StopOnShutdown&) = delete;
  StopOnShutdown& operator=(const StopOnShutdown&) = delete;

private:
  struct State
  {
    std::mutex m;
    bool armed{true};
  };
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<State> state_;
  rclcpp::PreShutdownCallbackHandle handle_;
};

// task_parameters.* (ROS params) -> blackboard STRINGS (arrays -> "a;b;c").
// Ported from the reference framework's loader.
void putBlackboard(BT::Blackboard::Ptr bb, const std::string& key, const rclcpp::Parameter& p)
{
  using PT = rclcpp::ParameterType;
  switch (p.get_type())
  {
    case PT::PARAMETER_BOOL:    bb->set<std::string>(key, p.as_bool() ? "true" : "false"); break;
    case PT::PARAMETER_INTEGER: bb->set<std::string>(key, std::to_string(p.as_int())); break;
    case PT::PARAMETER_DOUBLE:  bb->set<std::string>(key, std::to_string(p.as_double())); break;
    case PT::PARAMETER_STRING:  bb->set<std::string>(key, p.as_string()); break;
    case PT::PARAMETER_DOUBLE_ARRAY: {
      std::ostringstream os; const auto v = p.as_double_array();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str()); break;
    }
    case PT::PARAMETER_INTEGER_ARRAY: {
      std::ostringstream os; const auto v = p.as_integer_array();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str()); break;
    }
    default: break;
  }
}

void loadTaskParameters(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr bb)
{
  const std::string prefix = "task_parameters";
  auto names = node->list_parameters({prefix}, 10).names;
  for (const auto& full : names)
  {
    const std::string key = full.substr(prefix.size() + 1);
    rclcpp::Parameter p;
    if (node->get_parameter(full, p))
    {
      putBlackboard(bb, key, p);
      RCLCPP_INFO(node->get_logger(), "BB[%s] = %s", key.c_str(), bb->get<std::string>(key).c_str());
    }
  }
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "trainit_run_bt",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  const std::string bt_tree_file = getStr(node, "bt_tree_file", "");
  if (bt_tree_file.empty())
  {
    RCLCPP_FATAL(log, "param 'bt_tree_file' is required (path to a BT XML)");
    rclcpp::shutdown();
    return 2;
  }
  const std::string bt_tree_id = getStr(node, "bt_tree_id", "MainTree");
  const std::string mode_str   = getStr(node, "planner_mode", toString(kDefaultPlannerMode));
  const std::string backend_str= getStr(node, "process_path_backend", "pilz_sequence");
  auto mode = plannerModeFromString(mode_str);
  auto backend = processPathBackendFromString(backend_str);
  if (!mode || !backend)
  {
    RCLCPP_FATAL(log, "invalid planner_mode '%s' or process_path_backend '%s'",
                 mode_str.c_str(), backend_str.c_str());
    rclcpp::shutdown();
    return 2;
  }

  // The MoveIt start-up wait (vNext M2.7, owner decision D1): a finite number of seconds > 0, or
  // -1 for the legacy unbounded wait (which still WARNs every 5 s). Default 60.
  std::string refused;
  const auto moveit_wait_s = MoveItMotionRuntime::moveItWaitTimeout(node, refused);
  if (!moveit_wait_s)
  {
    RCLCPP_FATAL(log, "invalid moveit_wait_timeout_s %s: a finite number of seconds > 0, or -1 to "
                 "wait without a bound", refused.c_str());
    rclcpp::shutdown();
    return 2;
  }

  MoveItMotionRuntime::Config cfg;
  cfg.group       = getStr(node, "group", "manipulator");
  cfg.base_frame  = getStr(node, "base_frame", "base_link");
  cfg.tip_link    = getStr(node, "tip_link", "tcp");
  cfg.mode        = *mode;
  cfg.process_backend = *backend;
  cfg.available_pipelines = getStrArray(node, "available_pipelines", defaultAvailablePipelines());
  cfg.enforce_validation = getBool(node, "enforce_validation", true);

  const std::string gripper_action = getStr(node, "gripper_action", "");
  const std::string process_kind   = getStr(node, "process_controller", "mock");
  const std::string profiles_file  = getStr(node, "motion_profiles_file", "");
  const bool groot_monitor         = getBool(node, "groot_monitor", true);
  const int  groot_port            = getInt(node, "groot_port", 1667);

  // The end-effector contract (ADR-0022). An `eef` block makes the project canonical; without
  // one the LEGACY contract runs exactly as before. It is read from the parameter OVERRIDES (the
  // params files and -p arguments), so a key this runner declares with a default is never taken
  // for one the project set. A malformed block is a configuration error: exit 2, naming the key.
  eef::Contract contract;
  try
  {
    contract = eef::parseContract(*node);
  }
  catch (const eef::Refused& refused)
  {
    RCLCPP_FATAL(log, "end-effector contract REFUSED %s: %s (ADR-0022)", refused.key.c_str(), refused.why.c_str());
    rclcpp::shutdown();
    return 2;
  }
  const bool canonical = contract.canonical();
  RCLCPP_INFO(log, "%s", contractLine(contract).c_str());

  // The application's path library (vNext M3.1, ADR-0024): optional. A paths file that does not
  // load -- structure, a capability its provider lacks, inline geometry -- is a configuration error,
  // exit 2 naming the path, before anything moves.
  const std::string paths_file = getStr(node, "paths_file", "");
  std::shared_ptr<const PathLibrary> paths;
  if (!paths_file.empty())
  {
    try
    {
      paths = std::make_shared<const PathLibrary>(
        loadPathLibrary(paths_file, PathRuntimeFacts{*backend, cfg.tip_link}));
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(log, "paths_file %s REFUSED: %s", paths_file.c_str(), e.what());
      rclcpp::shutdown();
      return 2;
    }
    RCLCPP_INFO(log, "paths_file %s: %zu path(s)", paths_file.c_str(), paths->size());
  }

  // A namespaced robot's MoveIt, scene, joint-state, ServoTrack and end-effector names stay in its
  // namespace (ADR-0023 Decision 5): a remap that sends one of them out of it, onto another robot, is
  // refused here, once the contract is read and before any client exists. The root is not checked
  // and does not change.
  const auto escaped = MoveItMotionRuntime::crossNamespaceRemaps(node, contract);
  if (!escaped.empty())
  {
    std::string all;
    for (const auto& e : escaped) all += (all.empty() ? "" : ", ") + e;
    RCLCPP_FATAL(log, "MoveIt start-up: a remap sends this robot's resources out of its namespace %s: %s "
                 "(ADR-0023: a cross-namespace remap) -- exit 2", node->get_namespace(), all.c_str());
    rclcpp::shutdown();
    return 2;
  }

  // THIS robot's MoveIt, before anything blocks on it (vNext M2.7, D1): the runtime's constructor
  // waits for ever and in silence. Silent when MoveIt is up; else named, every 5 s, and bounded.
  switch (MoveItMotionRuntime::waitForMoveIt(node, *moveit_wait_s))
  {
    case MoveItMotionRuntime::MoveItWait::READY:
      break;
    case MoveItMotionRuntime::MoveItWait::TIMED_OUT:
      rclcpp::shutdown();
      return 2;
    case MoveItMotionRuntime::MoveItWait::INTERRUPTED:
      RCLCPP_INFO(log, "interrupted while waiting for MoveIt: exit 1");
      rclcpp::shutdown();
      return 1;
  }

  // The canonical end-effector devices: their clients, state subscriptions and scene signals,
  // relative to this node's (cell) namespace (ADR-0022). Built BEFORE the executor adds the node,
  // so their callback groups are collected with it (a group created later, once MoveIt's clients
  // exist, was never served in this process).
  std::unique_ptr<EndEffectorRegistry> eefs;
  if (canonical) eefs = std::make_unique<EndEffectorRegistry>(node, contract);

  // Spinner FIRST (MoveGroupInterface needs the node spinning), on its own threads: two, plus
  // one per end-effector device (each device's clients and state share one callback group).
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(),
                                                canonical ? 2 + contract.devices.size() : 2);
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  auto runtime = std::make_shared<MoveItMotionRuntime>(node, cfg);
  // LEGACY (ADR-0022, ADR-0023): the gripper action is used as the project writes it (TSA writes it
  // absolute, with the cell namespace in front).
  std::unique_ptr<GripperController> gripper;
  if (!canonical && !gripper_action.empty())   // a canonical run never builds the legacy client
    gripper = std::make_unique<GripperController>(node, gripper_action);
  std::unique_ptr<ProcessController> process;
  if (process_kind == "ros") process = std::make_unique<RosProcessController>(node);
  else                       process = std::make_unique<MockProcessController>(node->get_logger());

  MotionProfileRegistry profiles = loadMotionProfiles(profiles_file);
  // vNext M3.1: a path or segment that names a profile this runner did not load is refused here,
  // before the tree runs, not when FollowPath reaches it after earlier motions.
  if (paths)
  {
    for (const auto& name : paths->names())
    {
      const PathSpec* spec = paths->find(name);
      std::vector<std::string> named;
      if (!spec->profile.empty()) named.push_back(spec->profile);
      for (const auto& seg : spec->segments)
        if (!seg.profile.empty()) named.push_back(seg.profile);
      for (const auto& prof : named)
        if (!profiles.count(prof))
        {
          RCLCPP_FATAL(log, "paths_file %s REFUSED: path '%s' names profile '%s', which motion_profiles_file "
                       "'%s' does not define -- exit 2", paths_file.c_str(), name.c_str(), prof.c_str(),
                       profiles_file.c_str());
          rclcpp::shutdown();
          return 2;
        }
    }
  }
  BtContext ctx{runtime.get(), gripper.get(), process.get(), node};
  ctx.scene = &runtime->scene();   // what the scene nodes write (vNext M2.7: the nodes see only the door)

  // End-effector actuation targets (ADR-0010). Default 1.0/0.0 == suction on/off, so a
  // TRIGGER cell (no override in bt_params) behaves exactly as before. A JOINT_POSITION
  // cell (e.g. a Robotiq) sets the closed/open joint angles here.
  ctx.gripper_close_pos = getDouble(node, "gripper_close_position", 1.0);
  ctx.gripper_open_pos  = getDouble(node, "gripper_open_position", 0.0);
  // Sim grasp adapter signal (ADR-0010): when a gripper_cmd_topic is set, the runtime
  // publishes a LATCHED Bool on grasp/release that fires the planning-scene attach AND
  // the backend's sim adapter (Isaac SurfaceGripper / Gazebo LinkAttacher). Empty (the
  // default, and the deterministic golden) => no publisher => the adaptation-layer bridge
  // keeps driving the Bool, unchanged.
  // LEGACY (ADR-0022, ADR-0023): the topic is used as the project writes it: an absolute name is
  // GLOBAL, shared by every robot (M2.6 E2; E7 RC-04/RC-05).
  const std::string gripper_cmd_topic = getStr(node, "gripper_cmd_topic", "");
  if (!canonical && !gripper_cmd_topic.empty())   // canonical: the scene signal is eef/<id>/engaged
  {
    auto qos = rclcpp::QoS(1).transient_local();  // latched, like the Isaac suction bridge
    ctx.gripper_cmd_pub = node->create_publisher<std_msgs::msg::Bool>(gripper_cmd_topic, qos);
    RCLCPP_INFO(log, "gripper_cmd_topic='%s' -> runtime fires the sim grasp adapter (ADR-0010)",
                gripper_cmd_topic.c_str());
  }
  // TF for perception consumers (DetectObject). spin_thread=false: the executor above
  // already spins this node, so the listener's subscriptions ride on it.
  // CELL-GLOBAL (ADR-0023): tf2 subscribes to the absolute /tf and /tf_static, one tree for the
  // cell whatever this runner's namespace.
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  ctx.tf = tf_buffer;
  ctx.eefs = eefs.get();   // null on the LEGACY contract

  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);

  auto bb = BT::Blackboard::create();
  bb->set<BtContext*>(BB_BT_CONTEXT, &ctx);
  bb->set<MotionProfileRegistry>(BB_MOTION_PROFILES, profiles);
  bb->set<rclcpp::Node::SharedPtr>(BB_ROS_NODE, node);
  if (paths) bb->set<std::shared_ptr<const PathLibrary>>(BB_PATH_LIBRARY, paths);
  loadTaskParameters(node, bb);

  BT::Tree tree;
  try
  {
    factory.registerBehaviorTreeFromFile(bt_tree_file);
    tree = factory.createTree(bt_tree_id, bb);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(log, "failed to load BT '%s' (id=%s): %s", bt_tree_file.c_str(), bt_tree_id.c_str(), e.what());
    exec.cancel(); spinner.join(); rclcpp::shutdown();
    return 2;
  }
  // A canonical tree's LITERAL end-effector ports are checked against the contract before any
  // tick, and SetGripper is not part of it (ADR-0022 D2/D4): a configuration error, exit 2.
  if (eefs)
  {
    const auto problems = eefs->checkTree(tree);
    for (const auto& p : problems) RCLCPP_FATAL(log, "end-effector contract: %s (ADR-0022)", p.c_str());
    if (!problems.empty())
    {
      exec.cancel(); spinner.join(); rclcpp::shutdown();
      return 2;
    }
  }

  // vNext M3.5: the process channels the tree's ProcessEvent nodes name (literally) are made ready
  // before the first tick -- with the ros controller, their publishers exist during the DDS settle
  // below, so a driver that subscribed early has matched before the first event (nothing is published
  // before the first event). A channel read from the blackboard ({key}) and the legacy aliases
  // (EnableProcess / DisableProcess) keep creating theirs at their first event.
  tree.applyVisitor([&process](const BT::TreeNode* n) {
    if (n->registrationName() != "ProcessEvent") return;
    const auto& ports = n->config().input_ports;
    const auto channel = ports.find("channel");
    if (channel == ports.end() || !isLogicalChannel(channel->second)) return;   // a {key} is read at its tick
    const auto kind = ports.find("kind");
    process->prepare(channel->second, kind != ports.end() && kind->second == "set_value");
  });

#ifdef BTCPP_GROOT2_SUPPORT
  std::unique_ptr<BT::Groot2Publisher> groot;
  if (groot_monitor)
  {
    groot = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
    RCLCPP_INFO(log, "Groot2 live monitor on port %d", groot_port);
  }
#else
  (void)groot_monitor; (void)groot_port;
#endif

  RCLCPP_INFO(log, "trainit_run_bt: tree='%s' id=%s planner_mode=%s backend=%s",
              bt_tree_file.c_str(), bt_tree_id.c_str(), mode_str.c_str(), backend_str.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(3));  // let DDS action channels settle
  // ONE writer per device state topic (ADR-0022 D3/D4): a second publisher is a wiring error.
  if (eefs)
  {
    const auto writers = eefs->checkStateWriters();
    for (const auto& w : writers) RCLCPP_FATAL(log, "end-effector contract: %s (ADR-0022)", w.c_str());
    if (!writers.empty())
    {
      exec.cancel(); spinner.join(); rclcpp::shutdown();
      return 2;
    }
  }

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  try
  {
    const StopOnShutdown stop_on_shutdown(*runtime, log);   // vNext M3.4 (D8): only while the tree runs
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  catch (const std::exception& e)
  {
    if (!eefs) throw;   // LEGACY: unchanged, the exception ends the process as it always has (ADR-0022 D9)
    // A canonical node threw (e.g. a ROS call once a shutdown has begun): the run FAILS and the
    // tree is halted -- HOLD, no end-effector action -- never std::terminate (ADR-0022 D5).
    RCLCPP_FATAL(log, "the tree threw: %s; it is halted and the run fails (ADR-0022)", e.what());
    try
    {
      tree.haltTree();
    }
    catch (const std::exception& h)
    {
      RCLCPP_FATAL(log, "halting the tree threw: %s", h.what());
    }
    status = BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(log, "BT finished: %s", BT::toStr(status).c_str());

  int rc = (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
  // Exit code 3 (canonical projects only, ADR-0022 D5): the tree ended SUCCESS, but an
  // end-effector FAULT was latched during the run (a tree absorbed it). No launch reacts to it.
  if (eefs && status == BT::NodeStatus::SUCCESS && eefs->faultLatched())
  {
    for (const auto& f : eefs->latchedFaults()) RCLCPP_ERROR(log, "end-effector FAULT during the run: %s", f.c_str());
    RCLCPP_ERROR(log, "the tree ended SUCCESS with an end-effector FAULT latched: exit code 3 (ADR-0022)");
    rc = 3;
  }

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return rc;
}
