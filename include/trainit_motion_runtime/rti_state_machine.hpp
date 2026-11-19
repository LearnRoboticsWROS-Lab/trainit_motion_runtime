// =============================================================================
// rti_state_machine.hpp -- the Robot Task Interface v1 state machine (ADR-0032 Decisions 5, 6, 7, 8, 8a, 9, 10),
// PURE: no ROS, no threads, no clock of its own. Every input carries the STEADY time it is applied at; the result
// depends only on the ORDER of the inputs (Decision 8a.1). The ROS binding (robot_task_server) serializes the inputs
// under one lock and executes the effects; the tests drive it directly with a fake clock.
//
// Inputs:
//   onCommand  -- a command image received from the requester (and the command publisher count when the image
//                 carries a session other than the adopted one: Decision 10);
//   onCheck    -- the periodic check: runtime readiness and the command publisher count (every status period);
//   onJobEnded -- the job tree has returned (a natural end, or the halt after a stop): decides the result, once.
//   onActivity -- P0.b (D5): the RUNNING job's activity (a machine step's plain-words state) -> the status `detail`.
// Effects: start a job (StartJob) / stop the current job (stop + halt).
//
// The server starts FAULTED with fault_code SERVER_STARTED (R9, owner-approved): only a RESET returns it to the
// operational state, and a RESET never makes the held state known. NOT a safety function (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__RTI_STATE_MACHINE_HPP_
#define TRAINIT_MOTION_RUNTIME__RTI_STATE_MACHINE_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>

#include "trainit_motion_runtime/rti_contract.hpp"

namespace trainit::rti
{

using SteadyTime = std::chrono::steady_clock::time_point;

// The command image (wire fields only).
struct Command
{
  uint32_t requester_session{0};
  uint16_t heartbeat{0};
  uint32_t cmd_seq{0};
  uint8_t cmd{0};
  uint32_t job_id{0};
  uint16_t task{0};
};

// The status image's state-machine fields (the binding adds the header, the heartbeat and the server session).
struct Status
{
  uint32_t export_table_id{0};
  State state{State::FAULTED};
  bool ready{false};
  uint32_t ack_seq{0};
  Ack ack{Ack::NONE};
  uint16_t ack_code{0};
  uint32_t job_id{0};
  uint16_t task{0};
  Result result{Result::NONE};
  uint16_t result_code{0};
  uint16_t fault_code{0};
  std::string task_name;
  std::string detail;
  bool operator==(const Status& o) const;
  bool operator!=(const Status& o) const { return !(*this == o); }
};

// The stop cause recorded when RUNNING turns STOPPING (set once: Decision 8a.3).
enum class Cause : uint8_t { NONE, CANCEL, REQUESTER_CONFLICT, REQUESTER_LOST, RUNTIME_LOST, TASK_TIMEOUT };
const char* causeName(Cause cause);

// What the job's tree reported when it returned.
struct JobEnd
{
  enum class Tree : uint8_t { SUCCESS, FAILURE, HALTED };
  Tree tree{Tree::FAILURE};
  std::string outcome;            // trainit__outcome at the end ("" = none)
  std::optional<bool> holding;    // trainit__holding at the end (nullopt = absent / not a boolean: UNKNOWN)
  bool eef_fault{false};          // an end-effector fault latched during the job
  std::string reason;             // the fault gate's reason or an exception's text (detail only)
  // P0.b (D3): the job's FIRST machine-step failure, latched by the machine-channel registry whatever the tree's fault
  // gate: its STATION code (305 / 306 / 310 / 313; 0 = none, or a failure that is not a machine condition -- D4, D10)
  // and its plain-words text (detail). A FAILED tree with a station code ends FAILED with THAT code (an end-effector
  // fault latched in the same job outranks it: EEF_FAULT 308); a tree that SUCCEEDED keeps its own result and its
  // detail never carries the text (a hand-written tree recovered the failure).
  uint16_t station_code{0};
  std::string station_text;
};

// The effect "start this job" (executed by the binding outside the lock).
struct StartJob
{
  uint64_t counter{0};            // the server's job counter: the run id's suffix, the stop guard
  uint32_t job_id{0};
  uint16_t number{0};
  std::string task;
  std::string tree_id;
  double timeout_s{0.0};
  std::optional<bool> holding;    // the held-state seed (nullopt = unknown: the key stays absent)
};

struct Effects
{
  std::optional<StartJob> start;
  std::optional<uint64_t> stop;   // stop the job with this counter (MotionRuntime::stop + halt)
  bool stop_reassert{false};      // the periodic re-assertion of a stop already requested (Decision 8)
};

struct Decided
{
  Result result{Result::NONE};
  uint16_t result_code{0};
};

struct Config
{
  std::chrono::milliseconds requester_timeout{1000};
  std::size_t history{64};
  // Decision 8 (P0.a protocol review, M2): STOPPING is bounded. A job whose tree has not returned this long after the
  // stop was requested is decided at once with the held state UNKNOWN (cause CANCEL -> 309, any other cause -> its
  // code), FAULTED; the server is not ready again until that tree has returned (its late end is ignored).
  std::chrono::milliseconds stop_timeout{5000};
};

class StateMachine
{
public:
  StateMachine(Config config, ExportTable table);

  // `publishers`: the command publisher count, when the binding counted it for this input (always on a session
  // change, Decision 10); nullopt keeps the last known count.
  Effects onCommand(const Command& image, SteadyTime now, std::optional<int> publishers = std::nullopt);
  Effects onCheck(bool runtime_ready, const std::string& runtime_why, int publishers, SteadyTime now);
  // The job with `counter` returned. Valid in RUNNING / STOPPING for the current job; anything else is ignored
  // (returns nullopt). Decides the result once (Decision 8a.4).
  std::optional<Decided> onJobEnded(uint64_t counter, const JobEnd& end, SteadyTime now);
  // P0.b (D5): the job `counter`'s activity -- what a machine step shows while it runs ("MACHINE turner BUSY: 2 s of
  // 8.0 s"). Applied ONLY while that job is RUNNING (STOPPING keeps its "stopping: <cause>"; a finished job keeps its
  // result's detail): `detail` becomes `text`. Never touches the state, the result, the cause or a code.
  void onActivity(uint64_t counter, const std::string& text);

  const Status& status() const { return status_; }
  const ExportTable& table() const { return table_; }
  std::optional<bool> heldState() const { return holding_; }
  Cause cause() const { return cause_; }
  // While STOPPING: the job to keep stopping (its counter). A motion that STARTS after the stop request -- the tick
  // was planning, or in a gripper step, when the stop came -- is stopped by the next re-assertion (Decision 8).
  std::optional<uint64_t> stoppingJob() const
  {
    if (status_.state == State::STOPPING && job_) return job_->counter;
    if (stuck_) return stuck_;   // a job decided by the stop timeout: kept stopping until its tree returns
    return std::nullopt;
  }
  bool requesterAdopted() const { return adopted_ != 0; }
  bool requesterAlive() const { return alive_; }
  // A job whose stop timed out and whose tree has not returned yet (the tick thread is still busy with it).
  bool jobStuck() const { return stuck_.has_value(); }

private:
  struct Job
  {
    uint64_t counter{0};
    uint32_t session{0};
    uint32_t job_id{0};
    uint16_t number{0};
    std::string task;
    SteadyTime deadline{};
    SteadyTime stop_deadline{};   // set when STOPPING is entered (Config::stop_timeout)
  };

  void ack(uint32_t seq, Ack a, uint16_t c);
  Effects process(const Command& image, SteadyTime now);   // rules 2-4, the adopted session's command
  Effects settle(SteadyTime now);                          // readiness and the RUNNING stop causes
  Effects stopFor(Cause cause, SteadyTime now);
  void finish(const JobEnd& end, const Decided& d, SteadyTime now);   // the job's result applied (once)
  bool ready() const;
  bool isDuplicate(uint32_t session, uint32_t job_id) const;
  Decided decide(const JobEnd& end) const;
  void setState(State s);
  void noteLater(const std::string& what);

  Config config_;
  ExportTable table_;
  Status status_;

  // the requester
  uint32_t adopted_{0};
  // The adopted session's sequence REFERENCE: the cmd_seq of its last image, whatever the verdict (P0.a protocol
  // review, M1). It is NOT ack_seq: a non-adopted image may write the acknowledgement fields (NOT_SYNCHRONIZED), never
  // this reference -- so an adopted command, once processed (or rejected while not trusted), is never processed again.
  uint32_t adopted_seq_{0};
  uint16_t last_hb_{0};
  SteadyTime last_hb_change_{};
  bool alive_{false};
  uint32_t candidate_{0};
  uint16_t candidate_hb_{0};
  int candidate_changes_{0};
  bool session_lost_{false};      // the adopted session was replaced (a new session) while a job was current
  int publishers_{0};
  bool conflict_{false};

  // the runtime
  bool runtime_ready_{false};
  std::string runtime_why_{"not checked yet"};

  // the job
  std::optional<Job> job_;
  Cause cause_{Cause::NONE};
  std::string later_;             // causes seen after the first, for `detail`
  std::string result_detail_;     // the last finished job's `detail`: shown with its result until the next START
  uint64_t counter_{0};
  std::deque<std::pair<uint32_t, uint32_t>> history_;   // (session, job_id) of the last finished jobs
  std::optional<bool> holding_;   // ROBOT state, carried across jobs; unknown at start (Decision 8)
  std::optional<uint64_t> stuck_; // the counter of a job decided by the stop timeout whose tree has not returned
};

}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__RTI_STATE_MACHINE_HPP_
