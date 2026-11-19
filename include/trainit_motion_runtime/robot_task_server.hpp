// =============================================================================
// robot_task_server.hpp -- the RobotTaskServer's ROS binding (ADR-0032 Decisions 4, 8a, 9, 10, 11, 12).
//
// The Robot Task Interface v1 on ROS: the requester's command image arrives on the relative
// `trainit/robot_task/command`, the status is published on `trainit/robot_task/status` (robot slot, RELIABLE,
// TRANSIENT_LOCAL, depth 1): every period (a WALL timer: the steady clock, never sim time) and at once on every
// change. ONE mutually-exclusive callback group carries the command subscription and the timer, and ONE mutex
// guards the pure state machine (rti_state_machine.hpp): every input is applied in arrival order (8a.1).
//
// What runs a job and what says the runtime is ready are injected:
//   - JobExecutor: production = the behaviour-tree executor of trainit_task_server (the tick thread); tests = a
//     scripted executor. start() never blocks; requestStop() stops the job's motion and halts its tree -- only if
//     that job (its counter) is still the one running.
//   - RuntimeProbe: production = MoveIt's action servers answer + every canonical end-effector state is fresh.
// The executor reports the end of a job through jobEnded() (from its own thread).
// NOT a safety function (ADR-0033): heartbeat, cancel and FAULTED are ordinary operational control.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__ROBOT_TASK_SERVER_HPP_
#define TRAINIT_MOTION_RUNTIME__ROBOT_TASK_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trainit_interfaces/msg/robot_task_command.hpp>
#include <trainit_interfaces/msg/robot_task_status.hpp>

#include "trainit_motion_runtime/rti_contract.hpp"
#include "trainit_motion_runtime/rti_state_machine.hpp"

namespace trainit::rti
{

class JobExecutor
{
public:
  virtual ~JobExecutor() = default;
  virtual void start(const StartJob& job) = 0;
  virtual void requestStop(uint64_t counter) = 0;
};

class RuntimeProbe
{
public:
  virtual ~RuntimeProbe() = default;
  // true when the runtime can run a job; otherwise `why` names what is missing (diagnosis only)
  virtual bool ready(std::string& why) = 0;
};

class RobotTaskServer
{
public:
  using CommandMsg = trainit_interfaces::msg::RobotTaskCommand;
  using StatusMsg = trainit_interfaces::msg::RobotTaskStatus;

  struct Params
  {
    std::chrono::milliseconds period{100};              // status / heartbeat period
    std::chrono::milliseconds requester_timeout{1000};  // the requester declared lost after this without a change
    std::chrono::milliseconds stop_timeout{5000};       // STOPPING's bound (Decision 8): the stop must complete
  };
  // Reads rti_status_period_ms / rti_requester_timeout_ms / rti_stop_timeout_ms (declared with the defaults). The
  // period and the requester timeout are bounded to 50 ms .. 10 s, the stop timeout to 50 ms .. 60 s, and both
  // timeouts are >= 2 periods; outside: nullopt and `refused` names it (the server exits 2).
  static std::optional<Params> readParams(const rclcpp::Node::SharedPtr& node, std::string& refused);
  static rclcpp::QoS qos();   // RELIABLE, TRANSIENT_LOCAL, depth 1

  // `group`: the mutually-exclusive callback group of the RTI (created BEFORE the node joins its executor).
  RobotTaskServer(const rclcpp::Node::SharedPtr& node, const rclcpp::CallbackGroup::SharedPtr& group, Params params,
                  ExportTable table, JobExecutor& executor, RuntimeProbe& probe);
  ~RobotTaskServer();
  RobotTaskServer(const RobotTaskServer&) = delete;
  RobotTaskServer& operator=(const RobotTaskServer&) = delete;

  // The job `counter` returned (called by the executor, from its thread). Returns the decided result code, or
  // nullopt when the report is not the current job's (ignored).
  std::optional<Decided> jobEnded(uint64_t counter, const JobEnd& end);
  // P0.b (D5): the job `counter`'s activity (called by the executor's tick thread, through the machine-channel
  // registry's activity sink): under the ONE lock, StateMachine::onActivity; on a change, one `RTI: job N: <text>` log
  // line and the status published at once (ADR-0032 Decision 8a.1: every input under one lock).
  void jobActivity(uint64_t counter, const std::string& text);

  uint32_t session() const { return session_; }
  // The ExecutionEvent run id of the job `counter`: "<server_session as 8 hex>-<counter>" (ADR-0030 Amendment P0).
  std::string runId(uint64_t counter) const;
  Status status() const;
  const ExportTable& table() const { return machine_.table(); }
  // Stops the timer and the subscription (a clean shutdown); a later input is ignored.
  void shutdown();
  // An unexpected error inside the RTI's own callbacks ended serving: the server logged it (FATAL) and requested an
  // ORDERLY shutdown (the motion in flight gets MotionRuntime::stop, the tree is halted, the process exits) instead
  // of letting the exception escape the executor (std::terminate: no stop request at all).
  bool failed() const { return failed_.load(); }

private:
  void onCommand(const CommandMsg& msg);
  void onTimer();
  void execute(const Effects& fx);
  void publishLocked();
  void logTransition(const Status& before, const Status& after, const char* why);
  // The command publisher count; nullopt when the graph cannot be queried (the context is shutting down, or the
  // query threw): the last known count stands (the state machine keeps it).
  std::optional<int> countPublishers() const;
  // The runtime probe, guarded: a probe that throws = not ready (its text in the log, a neutral reason on the wire).
  bool probeReady(std::string& why);
  // Runs one callback body; an exception ends serving in order (failed()).
  template <class F>
  void guarded(const char* what, F&& body);

  rclcpp::Node::SharedPtr node_;
  Params params_;
  JobExecutor& executor_;
  RuntimeProbe& probe_;
  uint32_t session_;

  mutable std::mutex mutex_;   // guards everything below
  StateMachine machine_;
  uint16_t heartbeat_{0};
  uint32_t last_image_session_{0};
  bool stopped_{false};
  mutable std::atomic<int> last_publishers_{0};   // the last successful count (the timer's fallback)
  std::atomic<bool> failed_{false};

  rclcpp::Publisher<StatusMsg>::SharedPtr status_pub_;
  rclcpp::Subscription<CommandMsg>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string command_topic_;   // resolved: the publisher count's name
};

// A random, nonzero server session (std::random_device).
uint32_t newServerSession();

}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__ROBOT_TASK_SERVER_HPP_
