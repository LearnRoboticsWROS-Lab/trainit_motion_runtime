// reference_requester.cpp -- the Robot Task Interface's reference requester (ADR-0032 Decision 10a). See the header.
#include "trainit_motion_runtime/reference_requester.hpp"

#include <random>
#include <utility>
#include <vector>

namespace trainit::rti
{

namespace
{
uint32_t randomSession()
{
  std::random_device rd;
  std::uniform_int_distribution<uint32_t> d(1u, UINT32_MAX);
  return d(rd);
}

rclcpp::QoS rtiQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

std::string codeText(uint16_t c)
{
  const std::string n = codeName(c);
  return n.empty() ? std::to_string(c) : n + " (" + std::to_string(c) + ")";
}
}  // namespace

ReferenceRequester::ReferenceRequester(const rclcpp::Node::SharedPtr& node, Options options)
: node_(node), options_(std::move(options))
{
  image_.requester_session = options_.session != 0 ? options_.session : randomSession();
  pub_ = node_->create_publisher<CommandMsg>(options_.command_topic, rtiQos());
  sub_ = node_->create_subscription<StatusMsg>(options_.status_topic, statusReaderQos(),
                                                [this](const StatusMsg::SharedPtr m) { onStatus(*m); });
  timer_ = node_->create_wall_timer(options_.period, [this]() { onTimer(); });
}

rclcpp::QoS ReferenceRequester::statusReaderQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local();
}

ReferenceRequester::~ReferenceRequester()
{
  if (timer_) timer_->cancel();
}

void ReferenceRequester::setObserver(std::function<void(const std::string&)> observer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  observer_ = std::move(observer);
}

void ReferenceRequester::say(const std::string& line)
{
  std::function<void(const std::string&)> obs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obs = observer_;
  }
  if (obs) obs(line);
}

uint32_t ReferenceRequester::session() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return image_.requester_session;
}

uint32_t ReferenceRequester::lastSeq() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return image_.cmd_seq;
}

void ReferenceRequester::setHeartbeatPaused(bool paused)
{
  std::lock_guard<std::mutex> lock(mutex_);
  paused_ = paused;
}

void ReferenceRequester::setPublishing(bool publishing)
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishing_ = publishing;
}

std::optional<ReferenceRequester::StatusMsg> ReferenceRequester::lastStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool ReferenceRequester::serverAliveLocked() const
{
  return status_.has_value() && std::chrono::steady_clock::now() - status_hb_change_ <= options_.server_timeout;
}

bool ReferenceRequester::serverAlive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return serverAliveLocked();
}

void ReferenceRequester::publishLocked()
{
  if (!publishing_) return;
  image_.header.stamp = node_->now();
  pub_->publish(image_);
}

void ReferenceRequester::onTimer()
{
  std::vector<std::string> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!paused_)
    {
      ++image_.heartbeat;
      if (publishing_) ++beats_;
    }
    publishLocked();
    const bool alive = serverAliveLocked();
    if (was_alive_ && !alive)
      lines.push_back("Robot LOST: no status heartbeat for " +
                      std::to_string(options_.server_timeout.count()) + " ms");
    was_alive_ = alive;
  }
  cv_.notify_all();
  for (const auto& l : lines) say(l);
}

void ReferenceRequester::onStatus(const StatusMsg& m)
{
  std::vector<std::string> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const std::optional<StatusMsg> prev = status_;
    const bool alive_before = serverAliveLocked();
    if (!prev || prev->heartbeat != m.heartbeat || prev->server_session != m.server_session) status_hb_change_ = now;
    // Decision 10a rule 2: a (new) server adopts this requester after two heartbeat changes it RECEIVES -- counted
    // from the first status of each server session (a restarted server has seen none of the earlier ones)
    if (m.server_session != server_session_) session_beats_mark_ = beats_;
    if (server_session_ != 0 && m.server_session != server_session_)
    {
      if (inflight_job_ != 0)
      {
        lost_job_ = inflight_job_;
        lines.push_back("JOB LOST: the robot task server restarted while job " + std::to_string(inflight_job_) +
                        " was in flight (never resumed): recovery is the machine master's");
        inflight_job_ = 0;
      }
      lines.push_back("Robot task server restarted (new session)");
    }
    server_session_ = m.server_session;
    status_ = m;
    if (!alive_before && prev) lines.push_back("Robot status back");
    if (inflight_job_ != 0 && m.job_id == inflight_job_ && m.result != StatusMsg::RESULT_NONE) inflight_job_ = 0;

    const bool state_changed = !prev || prev->state != m.state || prev->server_session != m.server_session;
    const bool result_changed = !prev || prev->job_id != m.job_id || prev->result != m.result ||
                                prev->result_code != m.result_code;
    // P0.b (D5): the result's detail (a machine step's plain words: MACHINE turner TIMEOUT -- ...) on the same line
    if (result_changed && m.result != StatusMsg::RESULT_NONE)
      lines.push_back("Job " + std::to_string(m.job_id) + " (" + m.task_name + ") finished: " +
                      resultName(static_cast<Result>(m.result)) + " " + codeText(m.result_code) +
                      (m.detail.empty() ? std::string() : " -- " + m.detail));
    if (state_changed)
    {
      switch (m.state)
      {
        case StatusMsg::STATE_FAULTED:
          lines.push_back(m.fault_code == StatusMsg::CODE_SERVER_STARTED
                            ? std::string("Robot needs reset (server started)")
                            : "Robot FAULTED: " + codeText(m.fault_code) + " -- reset once the cause is cleared");
          break;
        case StatusMsg::STATE_IDLE: lines.push_back("Robot ready"); break;
        case StatusMsg::STATE_RUNNING:
          lines.push_back("Robot busy: job " + std::to_string(m.job_id) + " (" + m.task_name + ", task #" +
                          std::to_string(m.task) + ")");
          break;
        case StatusMsg::STATE_STOPPING: lines.push_back("Robot stopping job " + std::to_string(m.job_id) + ": " + m.detail); break;
        case StatusMsg::STATE_UNAVAILABLE: lines.push_back("Robot not ready: " + m.detail); break;
        default: break;
      }
    }
    // P0.b (D5): what the RUNNING job shows (a machine step: WAITING FOR MACHINE ..., MACHINE turner BUSY ...), printed
    // on every change of the detail -- the job never looks hung
    if (m.state == StatusMsg::STATE_RUNNING && !m.detail.empty() &&
        (!prev || prev->state != StatusMsg::STATE_RUNNING || prev->server_session != m.server_session ||
         prev->job_id != m.job_id || prev->detail != m.detail))
      lines.push_back("Robot busy: job " + std::to_string(m.job_id) + ": " + m.detail);
  }
  cv_.notify_all();
  for (const auto& l : lines) say(l);
}

ReferenceRequester::Reply ReferenceRequester::sendLocked(std::unique_lock<std::mutex>& lock, uint8_t cmd,
                                                         uint32_t job_id, uint16_t task,
                                                         std::optional<uint32_t> forced_seq)
{
  Reply r;
  r.seq = forced_seq ? *forced_seq : nextSeq(image_.cmd_seq);
  image_.cmd_seq = r.seq;
  image_.cmd = cmd;
  image_.job_id = job_id;
  image_.task = task;
  publishLocked();
  const uint32_t session = server_session_;
  const bool got = cv_.wait_for(lock, options_.ack_timeout, [&] {
    return status_ && (status_->server_session != session ||
                       (status_->ack_seq == r.seq && status_->ack != StatusMsg::ACK_NONE));
  });
  if (got && status_->server_session != session)
  {
    // the server restarted before acknowledging (review, minor 6): the old one may have ACCEPTED it -- a START is
    // reported JOB LOST (never resumed: Decision 9), never silently retried on the new server
    r.outcome = Outcome::SERVER_RESTARTED;
    r.text = "the robot task server restarted before acknowledging seq " + std::to_string(r.seq);
    if (cmd == CommandMsg::CMD_START) lost_job_ = job_id;
    return r;
  }
  if (!got)
  {
    r.outcome = Outcome::ACK_TIMEOUT;
    r.text = "ACK_TIMEOUT: no acknowledgement of seq " + std::to_string(r.seq);
    return r;
  }
  r.code = status_->ack_code;
  r.outcome = status_->ack == StatusMsg::ACK_ACCEPTED ? Outcome::ACCEPTED : Outcome::REJECTED;
  return r;
}

ReferenceRequester::Reply ReferenceRequester::command(uint8_t cmd, uint32_t job_id, uint16_t task, bool allow_retry)
{
  Reply r;
  std::vector<std::string> lines;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Decision 10a rule 2: a command only once the heartbeat has changed three times since the CURRENT server
    // session's first status (the server adopts a new session after two changes): at start, and after a server
    // restart. Bounded; a command sent anyway is NOT_SYNCHRONIZED and retried below.
    auto synchronized = [&] { return status_ && beats_ >= session_beats_mark_ + 3; };
    cv_.wait_for(lock, options_.period * 8, synchronized);
    if (!serverAliveLocked())
    {
      r.outcome = Outcome::REFUSED_NO_SERVER;
      r.text = "REFUSED: no live robot task server status (nothing sent)";
    }
    else if (status_->interface_version != kInterfaceVersion ||
             (options_.expected_table_id && status_->export_table_id != *options_.expected_table_id))
    {
      r.outcome = Outcome::REFUSED_MISMATCH;
      r.text = "REFUSED: the server serves RTI v" + std::to_string(status_->interface_version) + ", export table " +
               std::to_string(status_->export_table_id) + "; this requester was generated for RTI v" +
               std::to_string(kInterfaceVersion) +
               (options_.expected_table_id ? ", table " + std::to_string(*options_.expected_table_id) : "") +
               " (nothing sent: regenerate the PLC interface)";
    }
    else
    {
      r = sendLocked(lock, cmd, job_id, task, std::nullopt);
      if (r.outcome == Outcome::REJECTED && r.code == StatusMsg::REJECT_NOT_SYNCHRONIZED && allow_retry)
      {
        // Decision 10a rule 2: not adopted (yet) -- it was never executed. Three more heartbeats, then ONE retry
        // with a new cmd_seq (a server that adopts on this image takes it as the baseline, never executes it)
        lines.push_back(r.text.empty() ? "REJECTED NOT_SYNCHRONIZED: waiting to be adopted, then retrying once"
                                       : r.text + ": waiting to be adopted, then retrying once");
        const uint64_t mark = beats_;
        cv_.wait_for(lock, options_.period * 8, [&] { return beats_ >= mark + 3; });
        r = sendLocked(lock, cmd, job_id, task, std::nullopt);
        r.retried = true;
      }
      if (r.outcome == Outcome::ACK_TIMEOUT && allow_retry && options_.retry_on_ack_timeout &&
          (cmd == CommandMsg::CMD_START || cmd == CommandMsg::CMD_CANCEL || cmd == CommandMsg::CMD_RESET))
      {
        lines.push_back(r.text + ": retrying once with a new sequence number and the same job");
        Reply again = sendLocked(lock, cmd, job_id, task, std::nullopt);
        if (again.outcome == Outcome::REJECTED && again.code == StatusMsg::REJECT_SEQUENCE_ERROR)
          again = sendLocked(lock, cmd, job_id, task, std::nullopt);   // an image was skipped: continue after it
        again.retried = true;
        if (again.outcome == Outcome::REJECTED && status_)
        {
          if (cmd == CommandMsg::CMD_START && again.code == StatusMsg::REJECT_DUPLICATE_JOB && status_->job_id == job_id)
            again.outcome = Outcome::ALREADY_ACCEPTED;
          else if (cmd == CommandMsg::CMD_CANCEL && again.code == StatusMsg::REJECT_INVALID_STATE &&
                   status_->job_id == job_id && status_->state == StatusMsg::STATE_STOPPING)
            again.outcome = Outcome::ALREADY_ACCEPTED;
          else if (cmd == CommandMsg::CMD_CANCEL && again.code == StatusMsg::REJECT_NOT_CURRENT_JOB &&
                   status_->job_id == job_id && status_->result != StatusMsg::RESULT_NONE)
            again.outcome = Outcome::ALREADY_FINISHED;
        }
        r = again;
      }
      if (cmd == CommandMsg::CMD_START && (r.outcome == Outcome::ACCEPTED || r.outcome == Outcome::ALREADY_ACCEPTED))
      {
        inflight_job_ = job_id;
        lost_job_ = 0;
      }
      switch (r.outcome)
      {
        case Outcome::ACCEPTED: r.text = std::string("ACCEPTED: ") + cmdName(cmd); break;
        case Outcome::ALREADY_ACCEPTED: r.text = std::string("ACCEPTED (found on retry): ") + cmdName(cmd); break;
        case Outcome::ALREADY_FINISHED: r.text = "the job had already finished"; break;
        case Outcome::REJECTED: r.text = std::string("REJECTED: ") + cmdName(cmd) + " -- " + codeText(r.code); break;
        case Outcome::SERVER_RESTARTED:
          r.text += cmd == CommandMsg::CMD_START ? ": job " + std::to_string(job_id) + " LOST (never resumed)" : "";
          break;
        default: break;
      }
    }
  }
  for (const auto& l : lines) say(l);
  return r;
}

ReferenceRequester::Reply ReferenceRequester::commandWithSeq(uint32_t seq, uint8_t cmd, uint32_t job_id, uint16_t task)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return sendLocked(lock, cmd, job_id, task, seq);
}

ReferenceRequester::Reply ReferenceRequester::start(uint16_t task, uint32_t job_id)
{
  return command(CommandMsg::CMD_START, job_id, task);
}

ReferenceRequester::Reply ReferenceRequester::cancel(uint32_t job_id)
{
  return command(CommandMsg::CMD_CANCEL, job_id, 0);
}

ReferenceRequester::Reply ReferenceRequester::reset()
{
  return command(CommandMsg::CMD_RESET, 0, 0);
}

ReferenceRequester::View ReferenceRequester::view(uint32_t job_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  View v;
  v.server_alive = serverAliveLocked();
  v.job_lost = lost_job_ != 0 && lost_job_ == job_id;
  if (!status_) return v;
  const auto& s = *status_;
  v.ready = s.ready && v.server_alive;
  v.busy = s.job_id == job_id && (s.state == StatusMsg::STATE_RUNNING || s.state == StatusMsg::STATE_STOPPING);
  v.done = s.job_id == job_id && s.result != StatusMsg::RESULT_NONE;
  if (v.done)
  {
    v.result = static_cast<Result>(s.result);
    v.result_code = s.result_code;
  }
  v.fault = s.state == StatusMsg::STATE_FAULTED;
  v.fault_code = s.fault_code;
  return v;
}

bool ReferenceRequester::waitFor(const std::function<bool(const StatusMsg&)>& pred,
                                 std::chrono::milliseconds timeout) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(lock, timeout, [&] { return status_ && pred(*status_); });
}

}  // namespace trainit::rti
