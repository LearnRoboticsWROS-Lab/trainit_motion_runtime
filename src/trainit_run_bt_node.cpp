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
#include <chrono>
#include <memory>
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
std::vector<std::string> getStrArray(const rclcpp::Node::SharedPtr& n, const std::string& name,
                                     const std::vector<std::string>& def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_string_array();
}

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
  const std::string mode_str   = getStr(node, "planner_mode", "pilz");
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

  MoveItMotionRuntime::Config cfg;
  cfg.group       = getStr(node, "group", "manipulator");
  cfg.base_frame  = getStr(node, "base_frame", "base_link");
  cfg.tip_link    = getStr(node, "tip_link", "tcp");
  cfg.mode        = *mode;
  cfg.process_backend = *backend;
  cfg.available_pipelines = getStrArray(node, "available_pipelines",
                                        {"ompl", "pilz_industrial_motion_planner", "chomp"});
  cfg.enforce_validation = getBool(node, "enforce_validation", true);

  const std::string gripper_action = getStr(node, "gripper_action", "");
  const std::string process_kind   = getStr(node, "process_controller", "mock");
  const std::string profiles_file  = getStr(node, "motion_profiles_file", "");
  const bool groot_monitor         = getBool(node, "groot_monitor", true);
  const int  groot_port            = getInt(node, "groot_port", 1667);

  // Spinner FIRST (MoveGroupInterface needs the node spinning), on its own threads.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  auto runtime = std::make_shared<MoveItMotionRuntime>(node, cfg);
  std::unique_ptr<GripperController> gripper;
  if (!gripper_action.empty())
    gripper = std::make_unique<GripperController>(node, gripper_action);
  std::unique_ptr<ProcessController> process;
  if (process_kind == "ros") process = std::make_unique<RosProcessController>(node);
  else                       process = std::make_unique<MockProcessController>(node->get_logger());

  MotionProfileRegistry profiles = loadMotionProfiles(profiles_file);
  BtContext ctx{runtime.get(), gripper.get(), process.get(), node};

  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);

  auto bb = BT::Blackboard::create();
  bb->set<BtContext*>(BB_BT_CONTEXT, &ctx);
  bb->set<MotionProfileRegistry>(BB_MOTION_PROFILES, profiles);
  bb->set<rclcpp::Node::SharedPtr>(BB_ROS_NODE, node);
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

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(log, "BT finished: %s", BT::toStr(status).c_str());

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
