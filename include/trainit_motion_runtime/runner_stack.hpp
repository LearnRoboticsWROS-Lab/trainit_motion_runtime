// =============================================================================
// runner_stack.hpp -- the runtime stack trainit_run_bt builds before it loads a tree, FACTORED so that
// trainit_task_server (P0.a, ADR-0032 Decision 11) shares it: the same parameters, the same exit-2 checks, the same
// MoveIt wait, the same end-effector registry, executor, runtime, controllers, profiles, context and TF -- in the same
// order, with the same log lines. trainit_run_bt's behaviour is unchanged (P0.a assumption A3, checked by its suites).
//
//   configure(node)  -- the parameters and every configuration check, the MoveIt wait, the end-effector registry,
//                       the machine-channel registry (P0.b: its callback group).
//                       0 = ready; otherwise the exit code (2 a configuration error, 1 interrupted).
//   start(extra)     -- the executor (2 threads + 1 per end-effector device + 1 for the machine-channel registry +
//                       `extra`) and its spinner, the runtime, the gripper / process controllers, the profiles, the
//                       BtContext, TF. 0, or 2.
//   seed(bb)         -- a blackboard's runtime entries and task_parameters (what every tree of the bundle reads).
//   prepareChannels(tree) / prepareMachineServices() -- P0.b: the channels a loaded tree names LITERALLY, made ready
//                       before its first tick (the prepare visitor both runners share). 0, or 2.
// A callback group the caller needs served must be created between configure() and start() (Humble's executor
// collects a node's groups when the node is added).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__RUNNER_STACK_HPP_
#define TRAINIT_MOTION_RUNTIME__RUNNER_STACK_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/execution_events.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/machine_channels.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/path_library.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace trainit
{

class RunnerStack
{
public:
  RunnerStack() = default;
  ~RunnerStack();   // stops the spinner (joins it) if start() ran
  RunnerStack(const RunnerStack&) = delete;
  RunnerStack& operator=(const RunnerStack&) = delete;

  int configure(const rclcpp::Node::SharedPtr& node);
  int start(std::size_t extra_threads = 0);
  void stopSpinning();
  // `log_parameters`: log each task parameter (the runner, and the server once); false: quiet (every job)
  void seed(const BT::Blackboard::Ptr& bb, bool log_parameters = true) const;
  // vNext M3.5 + P0.b: the channels a loaded tree names LITERALLY, made ready before its first tick -- every
  // ProcessEvent channel through the process controller (its publisher exists during the DDS settle; `request` is no
  // process output: never prepared there), and every machine service a RequestMachine / WaitForMachine names,
  // collected for prepareMachineServices(). A channel read from the blackboard ({key}) is not prepared. Call it for
  // every tree the runner will tick (trainit_task_server: each exported task's job tree).
  void prepareChannels(BT::Tree& tree);
  // P0.b: the collected machine services -- `<service>/request` and `<service>/status` refused when a remap sends them
  // out of this robot's namespace (ADR-0023 Decision 5: FATAL, 2, before anything is created), then the registry's
  // status subscription and request publisher for each. 0, or 2.
  int prepareMachineServices();

  rclcpp::Node::SharedPtr node;
  std::string bt_tree_file;
  std::string bt_tree_id;
  std::string mode_str;
  std::string backend_str;
  MoveItMotionRuntime::Config cfg;
  std::string gripper_action;
  std::string process_kind;
  std::string profiles_file;
  bool groot_monitor{true};
  int groot_port{1667};
  eef::Contract contract;
  bool canonical{false};
  std::string paths_file;
  std::shared_ptr<const PathLibrary> paths;
  double moveit_wait_s{MoveItMotionRuntime::kMoveItWaitDefaultS};
  // Set BEFORE configure(): further robot-scoped topics the caller resolves in this robot's slot, refused by the
  // same cross-namespace remap check (ADR-0023 Decision 5). trainit_run_bt: none; trainit_task_server: the RTI's.
  std::vector<std::string> extra_scoped_topics;
  // P0.b: `channel_status_max_age_ms` (default 1000, bounds 200..10000; outside: exit 2) -- a machine service is LIVE
  // only while its status heartbeat changed within it (the steady clock), and a request synchronises within it.
  int channel_status_max_age_ms{1000};
  std::set<std::string> machine_services;   // collected by prepareChannels()

  std::unique_ptr<EndEffectorRegistry> eefs;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
  std::thread spinner;
  std::shared_ptr<MoveItMotionRuntime> runtime;
  std::unique_ptr<GripperController> gripper;
  std::unique_ptr<ProcessController> process;
  std::shared_ptr<ObservedProcessController::Observer> process_observer;
  MotionProfileRegistry profiles;
  BtContext ctx{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  // P0.b (D2): the machine-channel registry, built by configure() (its callback group before the executor collects
  // the node's groups), wired by start() (the process observer, BtContext::machines). Always present.
  std::unique_ptr<MachineChannels> machines;
};

// vNext M3.4 (owner decision D8, the MINIMAL stop scope): when orchestration stops while a tree runs -- SIGINT /
// SIGTERM, or any rclcpp::shutdown() -- the motion in flight gets the runtime's stop request, MotionRuntime::stop().
// A PRE-shutdown callback, once, removed with this object; the destructor disarms under the same mutex, so it
// returns only once an invocation in flight has finished (the runtime outlives this object).
class StopOnShutdown
{
public:
  StopOnShutdown(MotionRuntime& runtime, const rclcpp::Logger& log);
  ~StopOnShutdown();
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

// D0.5 (ADR-0030): a run stopped by a shutdown ends its execution events -- every open scope HALTED, then RUN_ENDED
// HALTED -- from a PRE-shutdown callback, while the context can still publish. The publisher outlives this object.
class EndEventsOnShutdown
{
public:
  explicit EndEventsOnShutdown(ExecutionEventPublisher& events);
  ~EndEventsOnShutdown();
  EndEventsOnShutdown(const EndEventsOnShutdown&) = delete;
  EndEventsOnShutdown& operator=(const EndEventsOnShutdown&) = delete;

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

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__RUNNER_STACK_HPP_
