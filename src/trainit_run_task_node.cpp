// =============================================================================
// trainit_run_task_node.cpp — GENERIC application runner (no app logic).
//
// Loads a MotionTask from a YAML file (param `task_file`) and executes it with
// the selected planner_mode / process backend. The SAME executable runs any
// application a setup assistant generates — the application IS the YAML, not
// this code.
//
// Run on top of the application bring-up (usually launched by the application
// package, which injects the MoveIt params):
//   ros2 run trainit_motion_runtime trainit_run_task --ros-args -p task_file:=<path>
//   -p planner_mode:=ompl  ...
// =============================================================================
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"
#include "trainit_motion_runtime/task_runner.hpp"
#include "trainit_motion_runtime/task_loader.hpp"

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
std::vector<std::string> getStrArray(const rclcpp::Node::SharedPtr& n, const std::string& name,
                                     const std::vector<std::string>& def)
{
  if (!n->has_parameter(name)) n->declare_parameter(name, def);
  return n->get_parameter(name).as_string_array();
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "trainit_run_task",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  const std::string task_file = getStr(node, "task_file", "");
  if (task_file.empty())
  {
    RCLCPP_FATAL(log, "param 'task_file' is required (path to a MotionTask YAML)");
    rclcpp::shutdown();
    return 2;
  }

  const std::string mode_str    = getStr(node, "planner_mode", "pilz");
  const std::string backend_str = getStr(node, "process_path_backend", "pilz_sequence");
  auto mode = plannerModeFromString(mode_str);
  if (!mode)
  {
    RCLCPP_FATAL(log, "invalid planner_mode '%s'. Allowed: pilz | ompl | ompl_chomp | stomp",
                 mode_str.c_str());
    rclcpp::shutdown();
    return 2;
  }
  auto backend = processPathBackendFromString(backend_str);
  if (!backend)
  {
    RCLCPP_FATAL(log, "invalid process_path_backend '%s'. Allowed: pilz_sequence | cartesian_waypoints",
                 backend_str.c_str());
    rclcpp::shutdown();
    return 2;
  }

  // Load the application task (this is the only "what to do" source).
  MotionTask task;
  try
  {
    task = TaskLoader::fromYamlFile(task_file);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(log, "failed to load task '%s': %s", task_file.c_str(), e.what());
    rclcpp::shutdown();
    return 2;
  }

  const bool plan_only = getBool(node, "plan_only", false);
  if (plan_only)
    for (auto& s : task.steps) s.options.plan_only = true;

  MoveItMotionRuntime::Config cfg;
  cfg.group       = !task.group.empty()      ? task.group      : getStr(node, "group", "manipulator");
  cfg.base_frame  = !task.base_frame.empty() ? task.base_frame : getStr(node, "base_frame", "base_link");
  cfg.tip_link    = !task.tip_link.empty()   ? task.tip_link   : getStr(node, "tip_link", "tcp");
  cfg.mode        = *mode;
  cfg.process_backend = *backend;
  cfg.available_pipelines = getStrArray(node, "available_pipelines",
                                        {"ompl", "pilz_industrial_motion_planner", "chomp"});
  cfg.enforce_validation = getBool(node, "enforce_validation", true);

  const std::string gripper_action = getStr(node, "gripper_action", "");
  const std::string process_kind   = getStr(node, "process_controller", "mock");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  auto runtime = std::make_shared<MoveItMotionRuntime>(node, cfg);

  std::unique_ptr<GripperController> gripper;
  if (!gripper_action.empty())
    gripper = std::make_unique<GripperController>(node, gripper_action);

  std::unique_ptr<ProcessController> process;
  if (process_kind == "ros")
    process = std::make_unique<RosProcessController>(node);
  else
    process = std::make_unique<MockProcessController>(node->get_logger());

  TaskRunner runner(*runtime, gripper.get(), process.get());

  RCLCPP_INFO(log, "task='%s' planner_mode=%s backend=%s plan_only=%d",
              task.name.c_str(), mode_str.c_str(), backend_str.c_str(), plan_only);

  const bool ok = runner.run(task);

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
