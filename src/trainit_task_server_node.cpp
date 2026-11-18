// =============================================================================
// trainit_task_server_node.cpp -- the RobotTaskServer executable (P0.a, ADR-0032 Decision 11).
//
// One process serves a robot's EXPORTED tasks to ONE machine master over the Robot Task Interface v1: one run per
// JOB (ADR-0030 Amendment P0), MoveIt and the end-effector registry alive across jobs. It shares trainit_run_bt's
// runtime stack (RunnerStack: the same parameters, checks and bring-up) and adds:
//   tasks_file                      the bundle's config/tasks.yaml (its exported tasks + export_table_id) -- required
//   rti_status_period_ms            100   (50..10000)
//   rti_requester_timeout_ms        1000  (50..10000, >= 2 periods)
//   rti_stop_timeout_ms             5000  (50..60000, >= 2 periods): STOPPING's bound -- a job whose tree has not
//                                         returned by then is decided FAILED with the held state unknown, FAULTED
//   rti_runtime_state_max_age_ms    1000  (50..10000): an end-effector state older than this = runtime not ready
// It starts FAULTED / SERVER_STARTED (R9) until the requester's RESET. Exit codes: 0 after a shutdown, 2 a
// configuration error (before serving: a tasks file, a job tree that does not load, a parameter out of bounds, the
// runner's own checks), 1 interrupted while waiting for MoveIt, 3 an internal error of the RTI binding ended serving
// (in order: the motion stopped, the tree halted). trainit_run_bt is unchanged; a bundle without an
// exported task never starts this executable. NOT a safety function (ADR-0033).
//
//   ros2 run trainit_motion_runtime trainit_task_server --ros-args --params-file <bt_params> \
//     -p bt_tree_file:=<bt_trees/task_program.xml> -p tasks_file:=<config/tasks.yaml>
// (usually launched by the bundle's trainit_task_server.launch.py, TSA >= 8.9.0)
// =============================================================================
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/bt_job_executor.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/moveit_runtime_probe.hpp"
#include "trainit_motion_runtime/process_controller.hpp"
#include "trainit_motion_runtime/robot_task_server.hpp"
#include "trainit_motion_runtime/rti_contract.hpp"
#include "trainit_motion_runtime/runner_stack.hpp"

using namespace trainit;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "trainit_task_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();
  auto fail = [](int rc) {
    rclcpp::shutdown();
    return rc;
  };

  // The exported tasks: refused before anything else is built.
  if (!node->has_parameter("tasks_file")) node->declare_parameter<std::string>("tasks_file", "");
  const std::string tasks_file = node->get_parameter("tasks_file").as_string();
  if (tasks_file.empty())
  {
    RCLCPP_FATAL(log, "param 'tasks_file' is required (the bundle's config/tasks.yaml) -- exit 2");
    return fail(2);
  }
  rti::ExportTable table;
  try
  {
    table = rti::loadExportTable(tasks_file);
  }
  catch (const rti::TableRefused& e)
  {
    RCLCPP_FATAL(log, "%s -- exit 2", e.what());
    return fail(2);
  }
  std::string refused;
  const auto params = rti::RobotTaskServer::readParams(node, refused);
  if (!params)
  {
    RCLCPP_FATAL(log, "invalid %s -- exit 2", refused.c_str());
    return fail(2);
  }
  if (!node->has_parameter("rti_runtime_state_max_age_ms"))
    node->declare_parameter<int64_t>("rti_runtime_state_max_age_ms", 1000);
  const int64_t max_age = node->get_parameter("rti_runtime_state_max_age_ms").as_int();
  if (max_age < 50 || max_age > 10000)
  {
    RCLCPP_FATAL(log, "invalid rti_runtime_state_max_age_ms %lld (bounds 50..10000 ms) -- exit 2",
                 static_cast<long long>(max_age));
    return fail(2);
  }

  // The runtime stack, shared with trainit_run_bt.
  RunnerStack stack;
  // the RTI's command and status stay in this robot's slot too: a remap sending them to another robot is refused
  // with the runner's other robot-scoped names (ADR-0023 Decision 5, ADR-0032)
  stack.extra_scoped_topics = {rti::kCommandTopic, rti::kStatusTopic};
  if (const int rc = stack.configure(node); rc != 0) return fail(rc);
  // The RTI's ONE mutually-exclusive callback group, created before the executor collects the node's groups.
  auto rti_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  if (const int rc = stack.start(1); rc != 0) return fail(rc);

  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);
  try
  {
    factory.registerBehaviorTreeFromFile(stack.bt_tree_file);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(log, "failed to load BT file '%s': %s -- exit 2", stack.bt_tree_file.c_str(), e.what());
    return fail(2);
  }
  // Every exported task's job tree loads, passes the end-effector contract, and its literal process channels are
  // prepared -- before serving (V3): a server never accepts a job it cannot run.
  bool first = true;
  for (const auto& entry : table.entries)
  {
    auto bb = BT::Blackboard::create();
    stack.seed(bb, first);
    first = false;
    BT::Tree tree;
    try
    {
      tree = factory.createTree(entry.treeId(), bb);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(log, "exported task '%s' (#%u): its job tree %s does not load from '%s': %s -- exit 2",
                   entry.name.c_str(), entry.number, entry.treeId().c_str(), stack.bt_tree_file.c_str(), e.what());
      return fail(2);
    }
    if (stack.eefs)
    {
      const auto problems = stack.eefs->checkTree(tree);
      for (const auto& p : problems) RCLCPP_FATAL(log, "end-effector contract (%s): %s (ADR-0022)",
                                                  entry.treeId().c_str(), p.c_str());
      if (!problems.empty()) return fail(2);
    }
    auto& process = stack.process;
    tree.applyVisitor([&process](const BT::TreeNode* n) {
      if (n->registrationName() != "ProcessEvent") return;
      const auto& ports = n->config().input_ports;
      const auto channel = ports.find("channel");
      if (channel == ports.end() || !isLogicalChannel(channel->second)) return;
      const auto kind = ports.find("kind");
      process->prepare(channel->second, kind != ports.end() && kind->second == "set_value");
    });
  }
  RCLCPP_INFO(log, "trainit_task_server: tree file '%s', tasks file '%s', planner_mode=%s backend=%s",
              stack.bt_tree_file.c_str(), tasks_file.c_str(), stack.mode_str.c_str(), stack.backend_str.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(3));   // let DDS action channels settle (as trainit_run_bt)
  if (stack.eefs)
  {
    const auto writers = stack.eefs->checkStateWriters();
    for (const auto& w : writers) RCLCPP_FATAL(log, "end-effector contract: %s (ADR-0022)", w.c_str());
    if (!writers.empty()) return fail(2);
  }

  rti::MoveItRuntimeProbe probe(node, stack.eefs.get(), std::chrono::milliseconds(max_age));
  rti::BtJobExecutor executor(stack, factory);
  rti::RobotTaskServer server(node, rti_group, *params, table, executor, probe);
  executor.setServer(&server);
  executor.run();   // the tick thread: until SIGINT / SIGTERM (or an internal error of the RTI binding)
  server.shutdown();
  const bool failed = server.failed();
  RCLCPP_INFO(log, "trainit_task_server: shut down%s", failed ? " after an internal error -- exit 3" : "");
  stack.stopSpinning();
  rclcpp::shutdown();
  return failed ? 3 : 0;
}
