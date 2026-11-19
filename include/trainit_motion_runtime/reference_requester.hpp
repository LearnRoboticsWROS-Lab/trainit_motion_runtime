// =============================================================================
// reference_requester.hpp -- the REFERENCE requester of the Robot Task Interface v1 (ADR-0032 Decision 10a): what a
// PLC-side library does, in C++, for the fake PLC (trainit_fake_plc) and the tests. It is a TEST / COMMISSIONING
// tool, never a machine master in production and never run by TSA.
//
// It owns the protocol so its user does not (Decision 14): a session per start, the heartbeat, the command sequence,
// the acknowledgement wait and its one retry, the server's liveness, the version / export-table check, JOB LOST.
// Its user says only: start <task> <job>, cancel <job>, reset -- and reads Ready / Busy / Done / Result / Fault.
// NOT a safety function (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__REFERENCE_REQUESTER_HPP_
#define TRAINIT_MOTION_RUNTIME__REFERENCE_REQUESTER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trainit_interfaces/msg/robot_task_command.hpp>
#include <trainit_interfaces/msg/robot_task_status.hpp>

#include "trainit_motion_runtime/rti_contract.hpp"

namespace trainit::rti
{

class ReferenceRequester
{
public:
  using CommandMsg = trainit_interfaces::msg::RobotTaskCommand;
  using StatusMsg = trainit_interfaces::msg::RobotTaskStatus;

  struct Options
  {
    std::chrono::milliseconds period{100};           // the image and its heartbeat (<= 100 ms)
    std::chrono::milliseconds ack_timeout{1000};
    std::chrono::milliseconds server_timeout{1000};  // the server declared lost after this without a heartbeat change
    std::optional<uint32_t> expected_table_id;       // the export table it was generated for (nullopt: not checked)
    uint32_t session{0};                             // 0: a random one
    bool retry_on_ack_timeout{true};                 // Decision 10a rule 5: one retry, new seq, same job
    std::string command_topic{kCommandTopic};
    std::string status_topic{kStatusTopic};
  };

  enum class Outcome
  {
    ACCEPTED,          // ack ACCEPTED for this command
    ALREADY_ACCEPTED,  // the retry found the first command accepted (DUPLICATE_JOB / INVALID_STATE for its job)
    ALREADY_FINISHED,  // a CANCEL's retry found its job finished (NOT_CURRENT_JOB, the status shows the job)
    REJECTED,          // ack REJECTED; `code` says why
    ACK_TIMEOUT,       // no acknowledgement (after the retry, when enabled)
    REFUSED_NO_SERVER, // no live server status: nothing sent
    REFUSED_MISMATCH,  // interface version or export table differs from the requester's: nothing sent
    SERVER_RESTARTED,  // the server restarted before acknowledging: a START is JOB LOST (it may have been accepted)
  };
  struct Reply
  {
    Outcome outcome{Outcome::ACK_TIMEOUT};
    uint16_t code{0};
    uint32_t seq{0};
    bool retried{false};
    std::string text;
  };

  // The simple PLC-programmer surface (Decision 14), for one job.
  struct View
  {
    bool server_alive{false};
    bool ready{false};
    bool busy{false};          // this job is the current one (RUNNING / STOPPING)
    bool done{false};          // the status reports a result for this job
    Result result{Result::NONE};
    uint16_t result_code{0};
    bool fault{false};
    uint16_t fault_code{0};
    bool job_lost{false};      // the server restarted while this job was in flight
  };

  ReferenceRequester(const rclcpp::Node::SharedPtr& node, Options options);
  ~ReferenceRequester();
  // The status READER's QoS: RELIABLE, TRANSIENT_LOCAL, a KEEP_LAST 20 history -- QoS-compatible with the server's
  // depth-1 writer (the wire unchanged): two statuses that arrive between two executor deliveries are both handled in
  // order, so a RUNNING detail the job leaves within a millisecond (a ready-wait that is ready at once, then the
  // result) is still printed (P0.b O1). The command image stays depth 1 (level-held).
  static rclcpp::QoS statusReaderQos();
  ReferenceRequester(const ReferenceRequester&) = delete;
  ReferenceRequester& operator=(const ReferenceRequester&) = delete;

  // Blocking (call from a thread that is not spinning this node): <= 2 x ack_timeout.
  Reply start(uint16_t task, uint32_t job_id);
  Reply cancel(uint32_t job_id);
  Reply reset();
  // The raw command (tests: an unknown code, NONE with a new seq, ...).
  Reply command(uint8_t cmd, uint32_t job_id, uint16_t task, bool allow_retry = true);
  // Tests: write an arbitrary cmd_seq (P3, P10) with this command, wait for its ack.
  Reply commandWithSeq(uint32_t seq, uint8_t cmd, uint32_t job_id, uint16_t task);

  void setHeartbeatPaused(bool paused);   // the requester "hangs": images keep coming, the heartbeat stops
  void setPublishing(bool publishing);    // the requester "vanishes": no image at all
  uint32_t session() const;
  uint32_t lastSeq() const;
  std::optional<StatusMsg> lastStatus() const;
  bool serverAlive() const;
  View view(uint32_t job_id) const;
  // Waits until `pred` holds for the latest status (true), or `timeout` passes (false).
  bool waitFor(const std::function<bool(const StatusMsg&)>& pred, std::chrono::milliseconds timeout) const;
  // Plain-words transitions (the fake PLC prints them): "Robot needs reset (server started)", "Robot ready", ...;
  // P0.b: every change of a RUNNING job's detail ("Robot busy: job N: <detail>") and a result's detail.
  void setObserver(std::function<void(const std::string&)> observer);

private:
  void onStatus(const StatusMsg& msg);
  void onTimer();
  void publishLocked();
  Reply sendLocked(std::unique_lock<std::mutex>& lock, uint8_t cmd, uint32_t job_id, uint16_t task,
                   std::optional<uint32_t> forced_seq);
  bool serverAliveLocked() const;
  void say(const std::string& line);

  rclcpp::Node::SharedPtr node_;
  Options options_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  CommandMsg image_;
  bool paused_{false};
  uint64_t beats_{0};                     // heartbeat changes published since start
  uint64_t session_beats_mark_{0};        // beats_ when the current server_session's first status arrived
  bool publishing_{true};
  std::optional<StatusMsg> status_;
  std::chrono::steady_clock::time_point status_hb_change_{};
  uint32_t server_session_{0};
  uint32_t inflight_job_{0};              // a job of ours the server accepted and has not finished
  uint32_t lost_job_{0};                  // JOB LOST: the server restarted while it was in flight
  bool was_alive_{false};
  std::function<void(const std::string&)> observer_;
  rclcpp::Publisher<CommandMsg>::SharedPtr pub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr liveness_timer_;
};

}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__REFERENCE_REQUESTER_HPP_
