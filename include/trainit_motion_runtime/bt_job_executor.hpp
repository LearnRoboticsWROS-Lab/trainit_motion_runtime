// =============================================================================
// bt_job_executor.hpp -- trainit_task_server's job executor (P0.a, ADR-0032 Decisions 8, 11).
//
// BtJobExecutor runs ONE job at a time on the thread that calls run() (the tick thread):
//   - a FRESH blackboard per job (the runner's entries + task_parameters, RunnerStack::seed), seeded with the held
//     state the previous job ended with (`trainit__holding`, through the same script assignment the tree's own
//     post-conditions use; unknown = the key stays absent, which ADR-0024 Amendment M4.5d reads as HELD);
//   - the job's tree `job_<task>` created from the factory the server registered once;
//   - its own ExecutionEvent run (ADR-0030 Amendment P0): run id `<server session>-<counter>`, job_id on every event,
//     the exported task's TASK_EXITED held until the result code is known;
//   - stop: requestStop() (from the RTI's callback) calls MotionRuntime::stop() at once and the tick thread halts the
//     tree as soon as the current tick returns; a stop is applied only to the job it names (its counter);
//   - the end: the tree's status, `trainit__outcome`, `trainit__holding`, the fault gate's reason
//     (`trainit__fault`) and whether an end-effector fault was latched during the job -> RobotTaskServer::jobEnded;
//   - P0.b: the machine-channel registry's JOB scope begins before RUN_STARTED (this job's outstanding requests, its
//     machine failure latch, its activity sink -> RobotTaskServer::jobActivity); the job's machine failure (its
//     STATION code and text) is read into the end (D3).
// Anything that escapes a job (an exception outside the tree's own catch) FAILS that job, held state UNKNOWN.
// The runtime probe is MoveItRuntimeProbe (moveit_runtime_probe.hpp).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__BT_JOB_EXECUTOR_HPP_
#define TRAINIT_MOTION_RUNTIME__BT_JOB_EXECUTOR_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/robot_task_server.hpp"
#include "trainit_motion_runtime/runner_stack.hpp"

namespace trainit::rti
{

constexpr const char* kHoldKey = "trainit__holding";     // TSA's HOLD_KEY (task_program.py)
constexpr const char* kOutcomeKey = "trainit__outcome";  // TSA's OUTCOME_KEY
constexpr const char* kGateKey = "trainit__fault";       // TSA's GATE_KEY

// The held state on a blackboard: the key's boolean (or a number 0 / 1, or "true" / "false"); nullopt when absent or
// anything else (UNKNOWN).
std::optional<bool> readHeld(const BT::Blackboard::Ptr& bb);
// Writes it the way a tree's post-condition does (`trainit__holding := true|false`).
void seedHeld(const BT::Blackboard::Ptr& bb, bool held);

class BtJobExecutor : public JobExecutor
{
public:
  BtJobExecutor(RunnerStack& stack, BT::BehaviorTreeFactory& factory);
  void setServer(RobotTaskServer* server) { server_ = server; }

  void start(const StartJob& job) override;
  void requestStop(uint64_t counter) override;
  // The tick thread: runs the jobs it is given until the ROS context shuts down.
  void run();

private:
  void runJob(const StartJob& job);
  void failJob(const StartJob& job, const std::string& what);
  void clearStop(uint64_t counter);

  RunnerStack& stack_;
  BT::BehaviorTreeFactory& factory_;
  RobotTaskServer* server_{nullptr};
  rclcpp::Publisher<ExecutionEventPublisher::Msg>::SharedPtr events_pub_;

  std::mutex m_;
  std::condition_variable cv_;
  std::optional<StartJob> pending_;
  uint64_t current_{0};
  std::atomic<uint64_t> stop_{0};
};


}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__BT_JOB_EXECUTOR_HPP_
