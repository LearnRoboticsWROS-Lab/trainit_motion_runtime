// runner_stack.cpp -- trainit_run_bt's runtime stack, factored (P0.a). The steps, their order and their log lines are
// exactly those of trainit_run_bt_node.cpp before the factoring (TMR 2.9.0); see the header.
#include "trainit_motion_runtime/runner_stack.hpp"

#include <chrono>
#include <sstream>
#include <vector>

#include "trainit_motion_runtime/bt_nodes.hpp"

namespace trainit
{

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

// task_parameters.* (ROS params) -> blackboard STRINGS (arrays -> "a;b;c").
// Ported from the reference framework's loader.
void putBlackboard(const BT::Blackboard::Ptr& bb, const std::string& key, const rclcpp::Parameter& p)
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
}  // namespace

RunnerStack::~RunnerStack()
{
  stopSpinning();
}

void RunnerStack::stopSpinning()
{
  if (exec) exec->cancel();
  if (spinner.joinable()) spinner.join();
}

int RunnerStack::configure(const rclcpp::Node::SharedPtr& n)
{
  node = n;
  auto log = node->get_logger();

  bt_tree_file = getStr(node, "bt_tree_file", "");
  if (bt_tree_file.empty())
  {
    RCLCPP_FATAL(log, "param 'bt_tree_file' is required (path to a BT XML)");
    return 2;
  }
  bt_tree_id  = getStr(node, "bt_tree_id", "MainTree");
  mode_str    = getStr(node, "planner_mode", toString(kDefaultPlannerMode));
  backend_str = getStr(node, "process_path_backend", "pilz_sequence");
  auto mode = plannerModeFromString(mode_str);
  auto backend = processPathBackendFromString(backend_str);
  if (!mode || !backend)
  {
    RCLCPP_FATAL(log, "invalid planner_mode '%s' or process_path_backend '%s'",
                 mode_str.c_str(), backend_str.c_str());
    return 2;
  }

  // The MoveIt start-up wait (vNext M2.7, owner decision D1): a finite number of seconds > 0, or
  // -1 for the legacy unbounded wait (which still WARNs every 5 s). Default 60.
  std::string refused;
  const auto wait_s = MoveItMotionRuntime::moveItWaitTimeout(node, refused);
  if (!wait_s)
  {
    RCLCPP_FATAL(log, "invalid moveit_wait_timeout_s %s: a finite number of seconds > 0, or -1 to "
                 "wait without a bound", refused.c_str());
    return 2;
  }
  moveit_wait_s = *wait_s;

  cfg.group       = getStr(node, "group", "manipulator");
  cfg.base_frame  = getStr(node, "base_frame", "base_link");
  cfg.tip_link    = getStr(node, "tip_link", "tcp");
  cfg.mode        = *mode;
  cfg.process_backend = *backend;
  cfg.available_pipelines = getStrArray(node, "available_pipelines", defaultAvailablePipelines());
  cfg.enforce_validation = getBool(node, "enforce_validation", true);
  // TMR 2.6.0 (ADR-0028): how a PTP / free move to a pose gets its joint configuration
  {
    std::string pose_refused;
    if (!MoveItMotionRuntime::poseConfigurationParams(node, cfg, pose_refused))
    {
      RCLCPP_FATAL(node->get_logger(), "invalid %s", pose_refused.c_str());
      return 2;
    }
  }

  gripper_action = getStr(node, "gripper_action", "");
  process_kind   = getStr(node, "process_controller", "mock");
  profiles_file  = getStr(node, "motion_profiles_file", "");
  groot_monitor  = getBool(node, "groot_monitor", true);
  groot_port     = getInt(node, "groot_port", 1667);
  // P0.b (P0B_DESIGN §3.4: LIVE = the heartbeat changed within it): the machine services' status max age, steady clock
  channel_status_max_age_ms = getInt(node, "channel_status_max_age_ms", 1000);
  if (channel_status_max_age_ms < 200 || channel_status_max_age_ms > 10000)
  {
    RCLCPP_FATAL(log, "invalid channel_status_max_age_ms %d (bounds 200..10000 ms) -- exit 2", channel_status_max_age_ms);
    return 2;
  }

  // The end-effector contract (ADR-0022). An `eef` block makes the project canonical; without
  // one the LEGACY contract runs exactly as before. It is read from the parameter OVERRIDES (the
  // params files and -p arguments), so a key this runner declares with a default is never taken
  // for one the project set. A malformed block is a configuration error: exit 2, naming the key.
  try
  {
    contract = eef::parseContract(*node);
  }
  catch (const eef::Refused& r)
  {
    RCLCPP_FATAL(log, "end-effector contract REFUSED %s: %s (ADR-0022)", r.key.c_str(), r.why.c_str());
    return 2;
  }
  canonical = contract.canonical();
  RCLCPP_INFO(log, "%s", contractLine(contract).c_str());

  // The application's path library (vNext M3.1, ADR-0024): optional. A paths file that does not
  // load -- structure, a capability its provider lacks, inline geometry -- is a configuration error,
  // exit 2 naming the path, before anything moves.
  paths_file = getStr(node, "paths_file", "");
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
      return 2;
    }
    RCLCPP_INFO(log, "paths_file %s: %zu path(s)", paths_file.c_str(), paths->size());
  }

  // A namespaced robot's MoveIt, scene, joint-state, ServoTrack and end-effector names stay in its
  // namespace (ADR-0023 Decision 5): a remap that sends one of them out of it, onto another robot, is
  // refused here, once the contract is read and before any client exists. The root is not checked
  // and does not change.
  const auto escaped = MoveItMotionRuntime::crossNamespaceRemaps(node, contract, extra_scoped_topics);
  if (!escaped.empty())
  {
    std::string all;
    for (const auto& e : escaped) all += (all.empty() ? "" : ", ") + e;
    RCLCPP_FATAL(log, "MoveIt start-up: a remap sends this robot's resources out of its namespace %s: %s "
                 "(ADR-0023: a cross-namespace remap) -- exit 2", node->get_namespace(), all.c_str());
    return 2;
  }

  // THIS robot's MoveIt, before anything blocks on it (vNext M2.7, D1): the runtime's constructor
  // waits for ever and in silence. Silent when MoveIt is up; else named, every 5 s, and bounded.
  switch (MoveItMotionRuntime::waitForMoveIt(node, moveit_wait_s))
  {
    case MoveItMotionRuntime::MoveItWait::READY:
      break;
    case MoveItMotionRuntime::MoveItWait::TIMED_OUT:
      return 2;
    case MoveItMotionRuntime::MoveItWait::INTERRUPTED:
      RCLCPP_INFO(log, "interrupted while waiting for MoveIt: exit 1");
      return 1;
  }

  // The canonical end-effector devices: their clients, state subscriptions and scene signals,
  // relative to this node's (cell) namespace (ADR-0022). Built BEFORE the executor adds the node,
  // so their callback groups are collected with it (a group created later, once MoveIt's clients
  // exist, was never served in this process).
  if (canonical) eefs = std::make_unique<EndEffectorRegistry>(node, contract);
  // P0.b (D2): the machine-channel registry -- its callback group too is created before the executor adds the node;
  // its subscriptions and publishers come when the loaded trees name their services (prepareMachineServices)
  MachineChannels::Options machine_options;
  machine_options.max_age = std::chrono::milliseconds(channel_status_max_age_ms);
  machines = std::make_unique<MachineChannels>(node, machine_options);
  return 0;
}

int RunnerStack::start(std::size_t extra_threads)
{
  auto log = node->get_logger();
  // Spinner FIRST (MoveGroupInterface needs the node spinning), on its own threads: two, plus
  // one per end-effector device (each device's clients and state share one callback group), plus one for the
  // machine-channel registry's group (P0.b: its status callbacks never wait behind the RTI's heartbeat, nor it behind them).
  exec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(),
    (canonical ? 2 + contract.devices.size() : 2) + (machines ? 1 : 0) + extra_threads);
  exec->add_node(node);
  spinner = std::thread([this]() { exec->spin(); });

  runtime = std::make_shared<MoveItMotionRuntime>(node, cfg);
  // LEGACY (ADR-0022, ADR-0023): the gripper action is used as the project writes it (TSA writes it
  // absolute, with the cell namespace in front).
  if (!canonical && !gripper_action.empty())   // a canonical run never builds the legacy client
    gripper = std::make_unique<GripperController>(node, gripper_action);
  if (process_kind == "ros") process = std::make_unique<RosProcessController>(node);
  else                       process = std::make_unique<MockProcessController>(node->get_logger());
  // D0.5 (ADR-0030): every DISPATCHED process event is also an execution event; the observer is set once
  // the tree exists. The controller itself is unchanged.
  process_observer = std::make_shared<ObservedProcessController::Observer>();
  process = std::make_unique<ObservedProcessController>(std::move(process), process_observer);
  // P0.b: a machine request actually written is an execution event too (kind `request`), through the same observer
  if (machines) machines->setProcessObserver(process_observer);

  profiles = loadMotionProfiles(profiles_file);
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
          return 2;
        }
    }
  }
  ctx.runtime = runtime.get();
  ctx.gripper = gripper.get();
  ctx.process = process.get();
  ctx.node = node;
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
  // The spinner thread fills the buffer while the tree ticks on this one, so a lookup may WAIT (its
  // timeout): without the flag tf2 refuses every timeout, logs an error and answers at once (ROS2ML-SYNC D3).
  // The wait runs on the node's clock: under use_sim_time a FROZEN /clock never ends it (a paused
  // simulator; Ctrl-C still does) -- a wall-time bound is H0's bounded waits (KNOWN_DEBT).
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  ctx.tf = tf_buffer;
  ctx.eefs = eefs.get();   // null on the LEGACY contract
  ctx.machines = machines.get();   // P0.b: RequestMachine / WaitForMachine
  return 0;
}

void RunnerStack::prepareChannels(BT::Tree& tree)
{
  tree.applyVisitor([this](const BT::TreeNode* n) {
    const auto& ports = n->config().input_ports;
    const auto channel = ports.find("channel");
    if (channel == ports.end() || !isLogicalChannel(channel->second)) return;   // a {key} is read at its tick
    const std::string& id = n->registrationName();
    if (id == "RequestMachine" || id == "WaitForMachine")
    {
      machine_services.insert(channel->second);   // P0.b: a machine service (prepareMachineServices)
      return;
    }
    if (id != "ProcessEvent") return;
    const auto kind = ports.find("kind");
    if (kind != ports.end() && kind->second == "request") return;   // never a process output (ProcessEvent refuses it)
    process->prepare(channel->second, kind != ports.end() && kind->second == "set_value");
  });
}

int RunnerStack::prepareMachineServices()
{
  if (machine_services.empty() || !machines) return 0;
  std::vector<std::string> topics;
  for (const auto& s : machine_services)
  {
    topics.push_back(machine::requestTopic(s));
    topics.push_back(machine::statusTopic(s));
  }
  // ADR-0023 Decision 5: a machine service's request and status stay in this robot's slot (the names are known only
  // once the trees are loaded: a second check, the same rule and exit as configure()'s)
  const auto escaped = MoveItMotionRuntime::crossNamespaceRemaps(node, contract, topics);
  if (!escaped.empty())
  {
    std::string all;
    for (const auto& e : escaped) all += (all.empty() ? "" : ", ") + e;
    RCLCPP_FATAL(node->get_logger(), "machine services: a remap sends this robot's resources out of its namespace %s: "
                 "%s (ADR-0023: a cross-namespace remap) -- exit 2", node->get_namespace(), all.c_str());
    return 2;
  }
  for (const auto& s : machine_services) machines->prepare(s);
  return 0;
}

void RunnerStack::seed(const BT::Blackboard::Ptr& bb, bool log_parameters) const
{
  bb->set<BtContext*>(BB_BT_CONTEXT, const_cast<BtContext*>(&ctx));
  bb->set<MotionProfileRegistry>(BB_MOTION_PROFILES, profiles);
  bb->set<rclcpp::Node::SharedPtr>(BB_ROS_NODE, node);
  if (paths) bb->set<std::shared_ptr<const PathLibrary>>(BB_PATH_LIBRARY, paths);
  const std::string prefix = "task_parameters";
  auto names = node->list_parameters({prefix}, 10).names;
  for (const auto& full : names)
  {
    const std::string key = full.substr(prefix.size() + 1);
    rclcpp::Parameter p;
    if (node->get_parameter(full, p))
    {
      putBlackboard(bb, key, p);
      if (log_parameters)
        RCLCPP_INFO(node->get_logger(), "BB[%s] = %s", key.c_str(), bb->get<std::string>(key).c_str());
    }
  }
}

StopOnShutdown::StopOnShutdown(MotionRuntime& runtime, const rclcpp::Logger& log)
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

StopOnShutdown::~StopOnShutdown()
{
  context_->remove_pre_shutdown_callback(handle_);
  std::lock_guard<std::mutex> lock(state_->m);   // waits for an invocation in flight
  state_->armed = false;
}

EndEventsOnShutdown::EndEventsOnShutdown(ExecutionEventPublisher& events)
: context_(rclcpp::contexts::get_global_default_context()), state_(std::make_shared<State>())
{
  handle_ = context_->add_pre_shutdown_callback([state = state_, &events]() {
    std::lock_guard<std::mutex> lock(state->m);
    if (state->armed) events.runEnded(BT::NodeStatus::RUNNING, true);
    state->armed = false;
  });
}

EndEventsOnShutdown::~EndEventsOnShutdown()
{
  context_->remove_pre_shutdown_callback(handle_);
  std::lock_guard<std::mutex> lock(state_->m);
  state_->armed = false;
}

}  // namespace trainit
