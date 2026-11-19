// =============================================================================
// p0b/scripted_source.hpp -- a SCRIPTED machine-service status source for TMR's P0.b tests: P0B_DESIGN.md §3.3, the
// source's rules (a stand-in for trainit_cell_io's ProcessDevice; TMR's tests never depend on that repository).
//   1. a new source_session, READY, no cycle, no rejection;  2. a request acted on only when it names this session and
//   its identity (requester_session, seq) differs from the last consumed one (a re-delivered image is a no-op) -- and,
//   as trainit_cell_io's refinement (protocol review MAJOR-1), from the last consumed identity OF ITS REQUESTER (a
//   small bounded map): a re-delivered image stays a no-op after another requester's identity was consumed in between;
//   3. READY / DONE + a new request -> ACCEPTED (BUSY, the cycle starts);  4. BUSY -> REJECTED BUSY (the running cycle
//   continues);  5. FAULT -> REJECTED FAULTED, HOMING -> REJECTED BUSY;  6. more than one request publisher (counted
//   every period) -> REJECTED TWO_REQUESTERS;  7. completion -> DONE (cycle_done, a level);  8. a device fault -> FAULT,
//   the cycle stops; RESET -> HOMING -> READY, the session stays;  9. the status every period (heartbeat +1, minted by
//   the scan) and at once on every change.
// Knobs: a timed cycle, a stuck actuator, a station fault, a cycle that never completes, a frozen image, silence (a
// killed source whose publisher lingers), a restart (a NEW session on the same publisher), a request consumed late, a
// request injected without a second publisher, a displaced rejection, two requesters forced, the latest request image
// re-delivered (as a re-forwarding bridge does, PB-P21). Its scan polls its own
// clock every 5 ms (wall time, or /clock with sim_time). Used by test_p0b_machine_nodes (S) and the P0.b runner
// cases of test_p0a_task_server (R). A TEST tool: NOT a machine, NOT a safety function (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME_TEST__P0B_SCRIPTED_SOURCE_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST__P0B_SCRIPTED_SOURCE_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trainit_interfaces/msg/channel_request.hpp>
#include <trainit_interfaces/msg/channel_status.hpp>

#include "trainit_motion_runtime/machine_channel.hpp"

namespace p0b_test
{
using StatusMsg = trainit_interfaces::msg::ChannelStatus;
using RequestMsg = trainit_interfaces::msg::ChannelRequest;
using trainit::machine::Identity;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

inline uint32_t randomSourceSession(uint32_t not_this)
{
  static std::mt19937 gen{std::random_device{}()};   // a test helper's generator
  std::uniform_int_distribution<uint32_t> d(1u, UINT32_MAX);
  uint32_t s = d(gen);
  while (s == not_this) s = d(gen);
  return s;
}

inline rclcpp::QoS sourceQos()   // the channel's QoS (§3.2): RELIABLE, TRANSIENT_LOCAL, depth 1
{
  return rclcpp::QoS(1).reliable().transient_local();
}

class ScriptedSource
{
public:
  struct Knobs
  {
    int period_ms = 100;          // the status period on the source's clock; the heartbeat +1 each period
    int cycle_ms = 400;           // an accepted request's cycle, then DONE
    int consume_delay_ms = 0;     // a request image is consumed this long after it arrived
    int homing_ms = 300;          // after a RESET: HOMING, then READY
    bool sim_time = false;        // the source's clock is /clock (its scan on sim time)
  };

  ScriptedSource(const std::string& service, Knobs k, const std::string& ns = "/p0b")
  : k_(k), cycle_ms_(k.cycle_ms), consume_delay_ms_(k.consume_delay_ms)
  {
    rclcpp::NodeOptions no;
    no.use_global_arguments(false);
    if (k.sim_time) no.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node_ = std::make_shared<rclcpp::Node>("p0b_src_" + service, ns, no);
    pub_ = node_->create_publisher<StatusMsg>(service + "/status", sourceQos());
    sub_ = node_->create_subscription<RequestMsg>(service + "/request", sourceQos(),
                                                  [this](RequestMsg::ConstSharedPtr m) { onRequest(*m); });
    request_topic_ = sub_->get_topic_name();
    img_.source_session = randomSourceSession(0);
    img_.state = StatusMsg::STATE_READY;
    timer_ = node_->create_wall_timer(5ms, [this] { poll(); });
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spinner_ = std::thread([this] { exec_->spin(); });
  }
  ~ScriptedSource()
  {
    exec_->cancel();
    if (spinner_.joinable()) spinner_.join();
    exec_.reset();
    timer_.reset();
    sub_.reset();
    pub_.reset();
    node_.reset();
  }

  // --- knobs -------------------------------------------------------------------------------------------------------
  void faultNextCycleAfter(int ms, uint16_t code)   // a stuck actuator: the next cycle faults after `ms`
  {
    std::lock_guard<std::mutex> l(m_);
    fault_next_ = std::make_pair(code, ms);
  }
  void neverComplete(bool on)
  {
    std::lock_guard<std::mutex> l(m_);
    never_complete_ = on;
  }
  void setCycleMs(int ms)
  {
    std::lock_guard<std::mutex> l(m_);
    cycle_ms_ = ms;
  }
  void setConsumeDelay(int ms)
  {
    std::lock_guard<std::mutex> l(m_);
    consume_delay_ms_ = ms;
  }
  void faultNow(uint16_t code)   // a device fault now: FAULT + its code; a running cycle stops where it is
  {
    std::lock_guard<std::mutex> l(m_);
    img_.state = StatusMsg::STATE_FAULT;
    img_.fault_code = code;
    img_.detail = "a scripted station fault";
    fault_at_.reset();
    force_publish_ = true;
  }
  void reset()   // the operator / machine master's RESET: HOMING, then READY (the session stays)
  {
    std::lock_guard<std::mutex> l(m_);
    if (img_.state != StatusMsg::STATE_FAULT) return;
    img_.state = StatusMsg::STATE_HOMING;
    img_.fault_code = 0;
    homing_end_ = nowMs() + k_.homing_ms;
    force_publish_ = true;
  }
  void freeze(bool on)   // a wedged source: the image keeps coming, its heartbeat stops
  {
    std::lock_guard<std::mutex> l(m_);
    frozen_ = on;
  }
  void silence(bool on)   // a killed source whose publisher lingers: no image at all
  {
    std::lock_guard<std::mutex> l(m_);
    silenced_ = on;
  }
  void restart()   // a NEW session on the same publisher: READY, no cycle, no rejection
  {
    std::lock_guard<std::mutex> l(m_);
    img_.source_session = randomSourceSession(img_.source_session);
    img_.heartbeat = 0;
    img_.state = StatusMsg::STATE_READY;
    img_.cycle_requester = img_.cycle_seq = 0;
    img_.cycle_done = false;
    img_.reject_requester = img_.reject_seq = 0;
    img_.reject_code = StatusMsg::REJECT_NONE;
    img_.fault_code = 0;
    last_consumed_.reset();
    last_by_requester_.clear();   // a new session: nothing consumed yet (rule 1)
    fault_at_.reset();
    force_publish_ = true;
  }
  void injectRequest(Identity id)   // a request that reaches the source WITHOUT a second publisher (scripted)
  {
    std::lock_guard<std::mutex> l(m_);
    injected_.push_back(id);
  }
  void displaceNextRejection(Identity other)   // PB-P20: the next rejection is displaced by another at once
  {
    std::lock_guard<std::mutex> l(m_);
    displace_ = other;
  }
  void forceTwoRequesters(bool on)   // rule 6, as if a second requester were counted
  {
    std::lock_guard<std::mutex> l(m_);
    force_two_ = on;
  }
  void redeliverLastRequest()   // the latest request image delivered AGAIN (level-held: re-forwarded, PB-P21)
  {
    std::lock_guard<std::mutex> l(m_);
    if (!last_request_) return;
    pending_ = *last_request_;
    pending_due_ = nowMs();
  }

  // --- observation -------------------------------------------------------------------------------------------------
  uint32_t session() const
  {
    std::lock_guard<std::mutex> l(m_);
    return img_.source_session;
  }
  int accepted() const
  {
    std::lock_guard<std::mutex> l(m_);
    return accepted_;
  }
  int completed() const
  {
    std::lock_guard<std::mutex> l(m_);
    return completed_;
  }
  int requestImages() const
  {
    std::lock_guard<std::mutex> l(m_);
    return request_images_;
  }
  std::vector<Identity> consumed() const
  {
    std::lock_guard<std::mutex> l(m_);
    return consumed_;
  }
  StatusMsg image() const
  {
    std::lock_guard<std::mutex> l(m_);
    return img_;
  }
  std::optional<Clock::time_point> firstPublished(const std::function<bool(const StatusMsg&)>& pred,
                                                  Clock::time_point since = {}) const
  {
    std::lock_guard<std::mutex> l(m_);
    for (const auto& [t, m] : published_)
      if (t >= since && pred(m)) return t;
    return std::nullopt;
  }
  std::optional<StatusMsg> lastPublishedImage() const
  {
    std::lock_guard<std::mutex> l(m_);
    if (published_.empty()) return std::nullopt;
    return published_.back().second;
  }
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  double nowMs() const { return static_cast<double>(node_->now().nanoseconds()) / 1e6; }

  void onRequest(const RequestMsg& m)
  {
    std::lock_guard<std::mutex> l(m_);
    ++request_images_;
    pending_ = m;   // level-held: a newer image replaces it
    pending_due_ = nowMs() + consume_delay_ms_;
    last_request_ = m;
  }

  void poll()
  {
    std::lock_guard<std::mutex> l(m_);
    const int64_t ns = node_->now().nanoseconds();
    if (ns == 0) return;   // a simulation clock not started yet
    const double now = static_cast<double>(ns) / 1e6;
    if (!started_)
    {
      started_ = true;
      next_beat_ = now;
    }
    bool publish = force_publish_;
    force_publish_ = false;
    if (now >= next_beat_)
    {
      if (!frozen_) ++img_.heartbeat;   // minted by the scan itself, every period
      next_beat_ += k_.period_ms;
      if (next_beat_ <= now) next_beat_ = now + k_.period_ms;
      publish = true;
      try
      {
        request_publishers_ = static_cast<int>(node_->count_publishers(request_topic_));   // rule 6, every period
      }
      catch (const std::exception&)
      {
      }
    }
    if (pending_ && now >= pending_due_)
    {
      const RequestMsg r = *pending_;
      pending_.reset();
      if (r.source_session == img_.source_session)   // rule 2: another session's image is never executed
        publish = consume(Identity{r.requester_session, r.seq}, now) || publish;
    }
    while (!injected_.empty())
    {
      const Identity id = injected_.front();
      injected_.pop_front();
      publish = consume(id, now) || publish;
    }
    if (img_.state == StatusMsg::STATE_BUSY)
    {
      if (fault_at_ && now >= *fault_at_)
      {
        img_.state = StatusMsg::STATE_FAULT;   // rule 8: the cycle stops where it is
        img_.fault_code = fault_code_;
        img_.detail = "the clamp did not reach its limit in time";
        fault_at_.reset();
        publish = true;
      }
      else if (!never_complete_ && now >= cycle_end_)
      {
        img_.cycle_done = true;   // rule 7: DONE, a level
        img_.state = StatusMsg::STATE_DONE;
        ++completed_;
        publish = true;
      }
    }
    if (img_.state == StatusMsg::STATE_HOMING && now >= homing_end_)
    {
      img_.state = StatusMsg::STATE_READY;
      publish = true;
    }
    if (publish && !silenced_)
    {
      img_.header.stamp = node_->now();
      pub_->publish(img_);
      published_.emplace_back(Clock::now(), img_);
    }
  }

  // rules 2-6: a NEW identity is consumed exactly once -- ACCEPTED (READY / DONE) or REJECTED with its reason
  bool consume(const Identity& id, double now)
  {
    if (last_consumed_ && *last_consumed_ == id) return false;   // re-delivered: a no-op
    // that requester's last consumed one, or an OLDER one (a replay: 32-bit serial order), is a no-op too -- the
    // cell's rule (trainit_cell_io seq_newer)
    if (const uint32_t last = lastOfRequester(id.requester); last != 0 && !seqNewer(id.seq, last)) return false;
    noteConsumed(id);
    consumed_.push_back(id);
    const uint8_t s = img_.state;
    if (force_two_ || request_publishers_ > 1) reject(id, StatusMsg::REJECT_TWO_REQUESTERS);
    else if (s == StatusMsg::STATE_FAULT) reject(id, StatusMsg::REJECT_FAULTED);
    else if (s == StatusMsg::STATE_BUSY || s == StatusMsg::STATE_HOMING) reject(id, StatusMsg::REJECT_BUSY);
    else
    {
      img_.cycle_requester = id.requester;
      img_.cycle_seq = id.seq;
      img_.cycle_done = false;
      img_.state = StatusMsg::STATE_BUSY;
      img_.fault_code = 0;
      ++accepted_;
      cycle_end_ = now + cycle_ms_;
      if (fault_next_)
      {
        fault_at_ = now + fault_next_->second;
        fault_code_ = fault_next_->first;
        fault_next_.reset();
      }
    }
    return true;
  }
  void reject(const Identity& id, uint8_t code)
  {
    img_.reject_requester = id.requester;
    img_.reject_seq = id.seq;
    img_.reject_code = code;
    if (displace_)
    {
      const Identity y = *displace_;   // another request rejected in the same scan, before anyone read the first
      displace_.reset();
      noteConsumed(y);
      consumed_.push_back(y);
      img_.reject_requester = y.requester;
      img_.reject_seq = y.seq;
    }
  }

  static bool seqNewer(uint32_t seq, uint32_t last)
  {
    const uint32_t ahead = seq - last;   // modulo 2^32
    return ahead != 0 && ahead < 0x80000000u;
  }
  // Rule 2's refinement: the last consumed seq of each requester session, the least recently consumed one evicted
  // beyond kRequesters -- trainit_cell_io's REMEMBERED_REQUESTERS (0: none -- a seq is never 0).
  static constexpr std::size_t kRequesters = 16;
  uint32_t lastOfRequester(uint32_t requester) const
  {
    for (const auto& [r, seq] : last_by_requester_)
      if (r == requester) return seq;
    return 0;
  }
  void noteConsumed(const Identity& id)
  {
    last_consumed_ = id;
    for (auto it = last_by_requester_.begin(); it != last_by_requester_.end(); ++it)
      if (it->first == id.requester)
      {
        last_by_requester_.erase(it);
        break;
      }
    last_by_requester_.emplace_back(id.requester, id.seq);
    if (last_by_requester_.size() > kRequesters) last_by_requester_.pop_front();
  }

  Knobs k_;
  mutable std::mutex m_;
  StatusMsg img_;
  int cycle_ms_;
  int consume_delay_ms_;
  bool started_{false};
  double next_beat_{0};
  bool force_publish_{false};
  std::optional<RequestMsg> pending_;
  double pending_due_{0};
  std::deque<Identity> injected_;
  std::optional<Identity> last_consumed_;
  std::deque<std::pair<uint32_t, uint32_t>> last_by_requester_;   // (requester_session, its last consumed seq)
  std::optional<RequestMsg> last_request_;                       // the latest request image received
  std::vector<Identity> consumed_;
  double cycle_end_{0};
  std::optional<double> fault_at_;
  uint16_t fault_code_{0};
  std::optional<std::pair<uint16_t, int>> fault_next_;
  double homing_end_{0};
  bool never_complete_{false};
  bool frozen_{false};
  bool silenced_{false};
  bool force_two_{false};
  std::optional<Identity> displace_;
  int request_publishers_{0};
  int accepted_{0};
  int completed_{0};
  int request_images_{0};
  std::vector<std::pair<Clock::time_point, StatusMsg>> published_;
  std::string request_topic_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<StatusMsg>::SharedPtr pub_;
  rclcpp::Subscription<RequestMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spinner_;
};

}  // namespace p0b_test

#endif  // TRAINIT_MOTION_RUNTIME_TEST__P0B_SCRIPTED_SOURCE_HPP_
