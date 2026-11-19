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
#include "trainit_motion_runtime/execution_events.hpp"
#include "trainit_motion_runtime/path_library.hpp"
#include "trainit_motion_runtime/runner_stack.hpp"

using namespace trainit;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "trainit_run_bt",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  // P0.a (ADR-0032 Decision 11): the runtime stack is FACTORED into RunnerStack, shared with trainit_task_server --
  // the same parameters, checks, MoveIt wait, registry, executor, runtime, controllers, profiles, context and TF,
  // in the same order with the same log lines. This runner's one-process-one-run model is unchanged.
  RunnerStack stack;
  if (const int rc = stack.configure(node); rc != 0)
  {
    rclcpp::shutdown();
    return rc;
  }
  if (const int rc = stack.start(); rc != 0)
  {
    rclcpp::shutdown();   // the stack joins its spinner (TMR <= 2.9.0 left it joinable here: std::terminate)
    return rc;
  }
  const std::string& bt_tree_file = stack.bt_tree_file;
  const std::string& bt_tree_id = stack.bt_tree_id;
  const std::string& mode_str = stack.mode_str;
  const std::string& backend_str = stack.backend_str;
  const bool groot_monitor = stack.groot_monitor;
  const int groot_port = stack.groot_port;
  auto& exec = *stack.exec;
  auto& spinner = stack.spinner;
  auto& runtime = stack.runtime;
  auto& process_observer = stack.process_observer;
  auto& eefs = stack.eefs;
  auto& machines = stack.machines;

  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);

  auto bb = BT::Blackboard::create();
  stack.seed(bb);

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
  // P0.b: the same visitor (factored, shared with trainit_task_server) collects the machine services the tree's
  // RequestMachine / WaitForMachine name; their status subscriptions exist during the settle too (PB-P18), after the
  // cross-namespace remap check of their names (exit 2).
  stack.prepareChannels(tree);
  if (const int rc = stack.prepareMachineServices(); rc != 0)
  {
    exec.cancel(); spinner.join(); rclcpp::shutdown();
    return rc;
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

  // D0.5 (ADR-0030): the run's canonical execution events, observed centrally (never code in the nodes),
  // on the relative trainit/execution_events -- created BEFORE the DDS settle below, so a subscriber in
  // this slot has matched before RUN_STARTED.
  const auto events = ExecutionEventPublisher::forNode(node, tree);
  *process_observer = [&events](const ProcessEvent& e) { events->processDispatched(e); };
  // the observer never outlives `events`, whichever way this scope is left (a rethrow included)
  struct ResetObserver
  {
    std::shared_ptr<ObservedProcessController::Observer> observer;
    ~ResetObserver() { *observer = nullptr; }
  } const observer_reset{process_observer};
  const EndEventsOnShutdown end_events_on_shutdown(*events);
  const std::string events_ns = node->get_namespace();
  RCLCPP_INFO(log, "execution events: run %s on %s/%s", events->runId().c_str(),
              events_ns == "/" ? "" : events_ns.c_str(), kExecutionEventsTopic);

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

  // P0.b: each machine service the tree names, with or without a status seen (a WARN names the unserved one, PB-P8)
  if (machines) machines->reportStart();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  // P0.b: the run is the machine-channel registry's JOB scope (no RTI here: the nodes' MACHINE: lines are the activity)
  if (machines) machines->beginJob(nullptr);
  events->runStarted();
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
    if (!eefs)          // LEGACY: unchanged, the exception ends the process as it always has (ADR-0022 D9)
    {
      events->runEnded(BT::NodeStatus::FAILURE, false);   // D0.5: the run's events still end
      throw;
    }
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
  events->runEnded(status, status == BT::NodeStatus::RUNNING);   // RUNNING here: stopped by a shutdown
  *process_observer = nullptr;
  if (machines) machines->endJob();

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
