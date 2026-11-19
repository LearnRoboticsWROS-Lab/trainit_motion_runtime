// rti_state_machine.cpp -- the Robot Task Interface v1 state machine (ADR-0032). Pure: see the header.
#include "trainit_motion_runtime/rti_state_machine.hpp"

namespace trainit::rti
{

bool Status::operator==(const Status& o) const
{
  return export_table_id == o.export_table_id && state == o.state && ready == o.ready && ack_seq == o.ack_seq &&
         ack == o.ack && ack_code == o.ack_code && job_id == o.job_id && task == o.task && result == o.result &&
         result_code == o.result_code && fault_code == o.fault_code && task_name == o.task_name && detail == o.detail;
}

const char* causeName(Cause cause)
{
  switch (cause)
  {
    case Cause::NONE: return "NONE";
    case Cause::CANCEL: return "CANCEL";
    case Cause::REQUESTER_CONFLICT: return "REQUESTER_CONFLICT";
    case Cause::REQUESTER_LOST: return "REQUESTER_LOST";
    case Cause::RUNTIME_LOST: return "RUNTIME_LOST";
    case Cause::TASK_TIMEOUT: return "TASK_TIMEOUT";
  }
  return "?";
}

namespace
{
Effects merge(Effects a, const Effects& b)
{
  if (b.start) a.start = b.start;
  if (b.stop) a.stop = b.stop;
  return a;
}

constexpr const char* kStartedDetail =
  "server started: it knows nothing of the robot's state (a part may be held); a RESET acknowledges the restart";
}  // namespace

StateMachine::StateMachine(Config config, ExportTable table) : config_(config), table_(std::move(table))
{
  status_.export_table_id = table_.id;
  setState(State::FAULTED);   // R9 (owner-approved): every start, until a RESET is accepted
  status_.fault_code = code::SERVER_STARTED;
  status_.detail = kStartedDetail;
}

void StateMachine::setState(State s)
{
  status_.state = s;
  status_.ready = s == State::IDLE;
}

void StateMachine::ack(uint32_t seq, Ack a, uint16_t c)
{
  status_.ack_seq = seq;
  status_.ack = a;
  status_.ack_code = c;
}

bool StateMachine::ready() const
{
  return runtime_ready_ && alive_ && !conflict_ && !stuck_;
}

bool StateMachine::isDuplicate(uint32_t session, uint32_t job_id) const
{
  if (job_ && job_->session == session && job_->job_id == job_id) return true;
  for (const auto& h : history_)
    if (h.first == session && h.second == job_id) return true;
  return false;
}

void StateMachine::noteLater(const std::string& what)
{
  if (what == causeName(cause_)) return;
  if (later_.find(what) != std::string::npos) return;
  later_ += (later_.empty() ? "" : ", ") + what;
}

Effects StateMachine::stopFor(Cause cause, SteadyTime now)
{
  cause_ = cause;
  later_.clear();
  job_->stop_deadline = now + config_.stop_timeout;
  setState(State::STOPPING);
  status_.detail = std::string("stopping: ") + causeName(cause);
  Effects fx;
  fx.stop = job_->counter;
  return fx;
}

Effects StateMachine::settle(SteadyTime now)
{
  alive_ = adopted_ != 0 && (now - last_hb_change_) <= config_.requester_timeout;
  Effects fx;
  switch (status_.state)
  {
    case State::IDLE:
    case State::UNAVAILABLE:
    {
      setState(ready() ? State::IDLE : State::UNAVAILABLE);
      if (status_.state == State::IDLE)
      {
        status_.detail = result_detail_;   // the last job's, until the next START
      }
      else
      {
        const std::string why = conflict_ ? "TWO_REQUESTERS"
                              : adopted_ == 0 ? "no requester adopted"
                              : !alive_ ? "requester lost"
                              : stuck_ ? "the previous job has not returned yet (its stop did not complete)"
                              : "runtime not ready: " + runtime_why_;
        status_.detail = result_detail_.empty() ? why : result_detail_ + "; " + why;
      }
      break;
    }
    case State::RUNNING:
      // one evaluation, a fixed order (Decision 8a.2)
      if (conflict_) fx = stopFor(Cause::REQUESTER_CONFLICT, now);
      else if (!alive_) fx = stopFor(Cause::REQUESTER_LOST, now);
      else if (!runtime_ready_) fx = stopFor(Cause::RUNTIME_LOST, now);
      else if (now >= job_->deadline) fx = stopFor(Cause::TASK_TIMEOUT, now);
      break;
    case State::STOPPING:
      // the cause is set once (8a.3): a later one is only listed
      if (conflict_) noteLater("REQUESTER_CONFLICT");
      if (!alive_) noteLater("REQUESTER_LOST");
      if (!runtime_ready_) noteLater("RUNTIME_LOST");
      if (job_ && now >= job_->deadline) noteLater("TASK_TIMEOUT");
      status_.detail = std::string("stopping: ") + causeName(cause_) + (later_.empty() ? "" : "; later: " + later_);
      if (job_ && now >= job_->stop_deadline)
      {
        // Decision 8 (review M2): the stop did not complete in time -- the tree is stuck (a motion whose controller
        // never answers, ...). Decided NOW with the held state unknown; the tree's late end is ignored, and the
        // server stays not ready until it has returned.
        JobEnd end;
        end.tree = JobEnd::Tree::HALTED;
        end.holding = std::nullopt;
        end.reason = "the stop did not complete within " + std::to_string(config_.stop_timeout.count()) + " ms";
        stuck_ = job_->counter;
        const Decided d = decide(end);
        finish(end, d, now);
      }
      break;
    case State::FAULTED:
      break;   // a loss never changes a fault (its fault_code stays)
  }
  return fx;
}

Effects StateMachine::onCommand(const Command& image, SteadyTime now, std::optional<int> publishers)
{
  if (publishers)
  {
    publishers_ = *publishers;
    conflict_ = publishers_ > 1;
  }
  const bool from_adopted = adopted_ != 0 && image.requester_session == adopted_;
  // "a new command": against the ADOPTED session's own last cmd_seq for its images (review M1), against the
  // published ack_seq for any other image (only its acknowledgement is at stake: it is never executed)
  const bool changed = from_adopted ? image.cmd_seq != adopted_seq_ : image.cmd_seq != status_.ack_seq;

  if (conflict_)
  {
    // Decision 10: while two publishers are detected the sessions in the images are not trusted -- no adoption, no
    // session change -- and nothing is executed. An adopted image's new command is CONSUMED as rejected: it is never
    // executed later, when the conflict is over (Decision 6: processed once).
    if (from_adopted)
    {
      if (image.heartbeat != last_hb_)
      {
        last_hb_ = image.heartbeat;
        last_hb_change_ = now;
      }
      adopted_seq_ = image.cmd_seq;
    }
    if (changed) ack(image.cmd_seq, Ack::REJECTED, code::NOT_SYNCHRONIZED);
    return settle(now);
  }

  if (from_adopted)
  {
    if (image.heartbeat != last_hb_)
    {
      last_hb_ = image.heartbeat;
      last_hb_change_ = now;   // the same session resuming after a loss is alive again: no re-adoption
    }
    Effects fx = settle(now);
    if (changed) fx = merge(fx, process(image, now));
    adopted_seq_ = image.cmd_seq;   // whatever the verdict: processed once
    return merge(fx, settle(now));
  }

  // A session other than the adopted one (or 0, never adopted).
  if (image.requester_session != 0)
  {
    if (adopted_ != 0)
    {
      // the adopted requester was replaced (a restart): REQUESTER_LOST for a current job (Decision 9)
      adopted_ = 0;
      session_lost_ = job_.has_value();
    }
    if (image.requester_session != candidate_)
    {
      candidate_ = image.requester_session;
      candidate_hb_ = image.heartbeat;
      candidate_changes_ = 0;
    }
    else if (image.heartbeat != candidate_hb_)
    {
      candidate_hb_ = image.heartbeat;
      ++candidate_changes_;
    }
    if (candidate_changes_ >= 2)
    {
      // ADOPTION (Decision 6 rule 1): the image's cmd_seq is the baseline, so a command present in it now -- a
      // level-held START after a restart -- is never executed
      adopted_ = candidate_;
      adopted_seq_ = image.cmd_seq;
      candidate_ = 0;
      candidate_changes_ = 0;
      last_hb_ = image.heartbeat;
      last_hb_change_ = now;
      ack(image.cmd_seq, Ack::NONE, code::NONE);
      return settle(now);
    }
  }
  if (changed) ack(image.cmd_seq, Ack::REJECTED, code::NOT_SYNCHRONIZED);
  return settle(now);
}

Effects StateMachine::process(const Command& image, SteadyTime now)
{
  Effects fx;
  if (image.cmd_seq != nextSeq(adopted_seq_))
  {
    ack(image.cmd_seq, Ack::REJECTED, code::SEQUENCE_ERROR);   // ack_seq takes it: the requester can continue
    return fx;
  }
  if (image.cmd != static_cast<uint8_t>(Cmd::START) && image.cmd != static_cast<uint8_t>(Cmd::CANCEL) &&
      image.cmd != static_cast<uint8_t>(Cmd::RESET))
  {
    ack(image.cmd_seq, Ack::REJECTED, code::INVALID_REQUEST);   // NONE with a changed cmd_seq, or unknown
    return fx;
  }
  const auto cmd = static_cast<Cmd>(image.cmd);
  if ((cmd == Cmd::START || cmd == Cmd::CANCEL) && image.job_id == 0)
  {
    ack(image.cmd_seq, Ack::REJECTED, code::INVALID_REQUEST);
    return fx;
  }
  const State s = status_.state;
  switch (cmd)
  {
    case Cmd::START:
    {
      if (isDuplicate(adopted_, image.job_id))
      {
        ack(image.cmd_seq, Ack::REJECTED, code::DUPLICATE_JOB);
        return fx;
      }
      if (s != State::IDLE)
      {
        ack(image.cmd_seq, Ack::REJECTED,
            s == State::FAULTED ? code::FAULTED : s == State::UNAVAILABLE ? code::NOT_READY : code::BUSY);
        return fx;
      }
      const ExportEntry* entry = table_.byNumber(image.task);
      if (!entry)
      {
        ack(image.cmd_seq, Ack::REJECTED, code::UNKNOWN_TASK);
        return fx;
      }
      ++counter_;
      Job job;
      job.counter = counter_;
      job.session = adopted_;
      job.job_id = image.job_id;
      job.number = entry->number;
      job.task = entry->name;
      job.deadline = now + std::chrono::milliseconds(entry->timeout_ms);
      job_ = job;
      cause_ = Cause::NONE;
      later_.clear();
      session_lost_ = false;
      status_.job_id = job.job_id;
      status_.task = job.number;
      status_.task_name = job.task;
      status_.result = Result::NONE;
      status_.result_code = code::NONE;
      status_.detail.clear();
      result_detail_.clear();
      setState(State::RUNNING);
      ack(image.cmd_seq, Ack::ACCEPTED, code::NONE);   // ACCEPTED and RUNNING in the same image (atomic)
      StartJob start;
      start.counter = job.counter;
      start.job_id = job.job_id;
      start.number = job.number;
      start.task = job.task;
      start.tree_id = entry->treeId();
      start.timeout_s = entry->timeout_s;
      start.holding = holding_;
      fx.start = start;
      return fx;
    }
    case Cmd::CANCEL:
      if (!job_ || job_->job_id != image.job_id)
      {
        ack(image.cmd_seq, Ack::REJECTED, code::NOT_CURRENT_JOB);
        return fx;
      }
      if (s != State::RUNNING)
      {
        ack(image.cmd_seq, Ack::REJECTED, code::INVALID_STATE);   // already STOPPING
        return fx;
      }
      ack(image.cmd_seq, Ack::ACCEPTED, code::NONE);
      return stopFor(Cause::CANCEL, now);
    case Cmd::RESET:
      if (s == State::RUNNING || s == State::STOPPING)
      {
        ack(image.cmd_seq, Ack::REJECTED, code::INVALID_STATE);
        return fx;
      }
      ack(image.cmd_seq, Ack::ACCEPTED, code::NONE);
      if (s == State::FAULTED)
      {
        // the restart / fault is ACKNOWLEDGED: the operational state returns once the runtime preconditions hold.
        // The held state is NOT made known by a RESET (ADR-0032 Decision 9).
        status_.fault_code = code::NONE;
        status_.detail.clear();
        result_detail_.clear();   // the fault (and the result that caused it) is acknowledged
        setState(State::UNAVAILABLE);
        return settle(now);
      }
      return fx;   // IDLE / UNAVAILABLE: nothing to reset
    case Cmd::NONE:
      break;
  }
  return fx;
}

Effects StateMachine::onCheck(bool runtime_ready, const std::string& runtime_why, int publishers, SteadyTime now)
{
  runtime_ready_ = runtime_ready;
  runtime_why_ = runtime_ready ? "" : runtime_why;
  publishers_ = publishers;
  conflict_ = publishers_ > 1;
  return settle(now);
}

Decided StateMachine::decide(const JobEnd& end) const
{
  auto natural = [&]() -> Decided {
    if (end.tree == JobEnd::Tree::SUCCESS)
    {
      if (end.eef_fault) return {Result::FAILED, code::EEF_FAULT};
      // a failure outcome on a tree that SUCCEEDED (only a hand-written tree can do this): inconsistent, fail closed
      if (end.outcome == "TERMINAL_FAULT" || end.outcome == "PICKABLE_EXHAUSTED" || end.outcome == "RECOVERY_EXHAUSTED")
        return {Result::FAILED, code::TASK_FAILED};
      if (end.outcome == "BIN_EMPTY") return {Result::SUCCEEDED, code::BIN_EMPTY};
      if (end.outcome == "NOTHING_PICKABLE") return {Result::SUCCEEDED, code::NOTHING_PICKABLE};
      return {Result::SUCCEEDED, code::OK};
    }
    // P0.b (D3): a machine step's STATION code -- latched whatever the tree's fault gate -- decides a tree that did not
    // succeed, before TERMINAL_FAULT (a gated program's outcome) and the 301 fallback: never TASK_FAILED, never 300.
    // An end-effector fault latched in the same job is checked BEFORE it (C++ review M4): the tool's physical state is
    // then unconfirmed -- the more severe uncertainty (ADR-0032 8a.4) -- so the result is EEF_FAULT 308 and the
    // machine's words stay in the detail.
    if (end.station_code != 0 && end.eef_fault) return {Result::FAILED, code::EEF_FAULT};
    if (end.station_code != 0) return {Result::FAILED, end.station_code};
    if (end.outcome == "TERMINAL_FAULT") return {Result::FAILED, code::TERMINAL_FAULT};
    if (end.outcome == "PICKABLE_EXHAUSTED" || end.outcome == "RECOVERY_EXHAUSTED")
    {
      // ENDED requires a healthy robot: an end-effector fault latched makes it a fault
      if (end.eef_fault) return {Result::FAILED, code::EEF_FAULT};
      return {Result::ENDED,
              end.outcome == "PICKABLE_EXHAUSTED" ? code::PICKABLE_EXHAUSTED : code::RECOVERY_EXHAUSTED};
    }
    return {Result::FAILED, code::TASK_FAILED};   // no canonical outcome: fail closed
  };
  switch (cause_)
  {
    case Cause::NONE:
      return natural();
    case Cause::CANCEL:
      // Decision 8a.4: the tree finished SUCCESS before the halt -> its own result; otherwise fail closed
      if (end.tree == JobEnd::Tree::SUCCESS) return natural();
      if (end.holding && *end.holding) return {Result::FAILED, code::CANCELLED_WHILE_HOLDING};
      if (!end.holding) return {Result::FAILED, code::CANCELLED_HOLD_UNKNOWN};
      if (end.eef_fault) return {Result::FAILED, code::EEF_FAULT};
      if (!runtime_ready_) return {Result::FAILED, code::RUNTIME_LOST};   // a strictly more severe uncertainty
      return {Result::CANCELLED, code::CANCELLED};
    case Cause::REQUESTER_CONFLICT:
      return {Result::FAILED, code::REQUESTER_CONFLICT};
    case Cause::REQUESTER_LOST:
      return {Result::FAILED, code::REQUESTER_LOST};
    case Cause::RUNTIME_LOST:
      return {Result::FAILED, code::RUNTIME_LOST};
    case Cause::TASK_TIMEOUT:
      return {Result::FAILED, code::TASK_TIMEOUT};
  }
  return {Result::FAILED, code::TASK_FAILED};
}

std::optional<Decided> StateMachine::onJobEnded(uint64_t counter, const JobEnd& end, SteadyTime now)
{
  if (stuck_ && *stuck_ == counter)
  {
    // the tree of a job the stop timeout already decided has returned: its result stands (decided once); the tick
    // thread is free again, so the server may become ready (no RESET skipped: the job's fault is still FAULTED)
    stuck_.reset();
    settle(now);
    return std::nullopt;
  }
  if (!job_ || job_->counter != counter ||
      (status_.state != State::RUNNING && status_.state != State::STOPPING))
    return std::nullopt;
  const Decided d = decide(end);
  finish(end, d, now);
  return d;
}

void StateMachine::onActivity(uint64_t counter, const std::string& text)
{
  if (status_.state != State::RUNNING || !job_ || job_->counter != counter) return;
  status_.detail = text;
}

void StateMachine::finish(const JobEnd& end, const Decided& d, SteadyTime now)
{
  // ROBOT state: handed to the next job (Decision 8) -- UNKNOWN when the end leaves the tool physically unconfirmed
  // (the runtime lost, an end-effector fault: the key may be what the tree last WROTE, not what the tool holds)
  const bool unconfirmed = d.result_code == code::RUNTIME_LOST || d.result_code == code::EEF_FAULT || end.eef_fault;
  holding_ = unconfirmed ? std::nullopt : end.holding;
  history_.emplace_back(job_->session, job_->job_id);
  while (history_.size() > config_.history) history_.pop_front();

  std::string detail;
  if (cause_ != Cause::NONE)
  {
    detail = std::string("stopped: ") + causeName(cause_);
    if (session_lost_ && cause_ == Cause::REQUESTER_LOST) detail += " (the requester's session changed)";
    if (cause_ == Cause::CANCEL && end.tree == JobEnd::Tree::SUCCESS) detail += " (too late: the task had completed)";
    if (cause_ == Cause::CANCEL && !end.outcome.empty() && end.tree != JobEnd::Tree::SUCCESS)
      detail += " (the task's own outcome " + end.outcome + " is masked by the cancel)";
    if (cause_ == Cause::CANCEL && end.station_code != 0 && end.tree != JobEnd::Tree::SUCCESS)
      detail += std::string(" (the machine failure ") + codeName(end.station_code) + " is masked by the cancel)";
    if (!later_.empty()) detail += "; later: " + later_;
  }
  // P0.b: the machine step's own words (once: the fault gate's reason is usually the same text) -- only for a tree
  // that did NOT succeed (C++ review M3): a hand-written tree that recovered a machine failure (a retry, a fallback)
  // and then succeeded never reports that stale failure. Otherwise D3 holds: the job's FIRST machine failure stands.
  // TSA-generated trees cannot recover a machine failure (TSA refuses request / wait inside a fallback or a retry,
  // R-P0B-06), so the latch only matters for hand-written trees.
  if (end.tree != JobEnd::Tree::SUCCESS && !end.station_text.empty() &&
      end.reason.find(end.station_text) == std::string::npos)
    detail += (detail.empty() ? "" : "; ") + end.station_text;
  if (!end.reason.empty()) detail += (detail.empty() ? "" : "; ") + end.reason;

  status_.result = d.result;
  status_.result_code = d.result_code;
  status_.detail = detail;
  result_detail_ = detail;   // kept with the result until the next START (a late TRANSIENT_LOCAL reader sees it)
  job_.reset();
  cause_ = Cause::NONE;
  later_.clear();
  session_lost_ = false;
  if (d.result == Result::FAILED)
  {
    status_.fault_code = d.result_code;
    setState(State::FAULTED);
  }
  else
  {
    setState(State::IDLE);
    settle(now);   // IDLE, or UNAVAILABLE when the requester or the runtime is gone (F12)
  }
}

}  // namespace trainit::rti
