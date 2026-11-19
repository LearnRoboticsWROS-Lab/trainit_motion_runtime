// machine_channels.cpp -- the per-process machine-channel registry (P0.b). See the header.
#include "trainit_motion_runtime/machine_channels.hpp"

#include <random>
#include <utility>

namespace trainit
{

// The pure vocabulary IS the message's (one source of truth: the type; checked here at compile time).
using StatusM = trainit_interfaces::msg::ChannelStatus;
static_assert(StatusM::STATE_NONE == static_cast<uint8_t>(machine::State::NONE));
static_assert(StatusM::STATE_READY == static_cast<uint8_t>(machine::State::READY));
static_assert(StatusM::STATE_BUSY == static_cast<uint8_t>(machine::State::BUSY));
static_assert(StatusM::STATE_DONE == static_cast<uint8_t>(machine::State::DONE));
static_assert(StatusM::STATE_FAULT == static_cast<uint8_t>(machine::State::FAULT));
static_assert(StatusM::STATE_HOMING == static_cast<uint8_t>(machine::State::HOMING));
static_assert(StatusM::REJECT_NONE == static_cast<uint8_t>(machine::Reject::NONE));
static_assert(StatusM::REJECT_BUSY == static_cast<uint8_t>(machine::Reject::BUSY));
static_assert(StatusM::REJECT_FAULTED == static_cast<uint8_t>(machine::Reject::FAULTED));
static_assert(StatusM::REJECT_TWO_REQUESTERS == static_cast<uint8_t>(machine::Reject::TWO_REQUESTERS));
static_assert(StatusM::FAULT_NONE == static_cast<uint16_t>(machine::FaultCode::NONE));
static_assert(StatusM::FAULT_ACTUATOR_TIMEOUT == static_cast<uint16_t>(machine::FaultCode::ACTUATOR_TIMEOUT));
static_assert(StatusM::FAULT_SENSOR_FAULT == static_cast<uint16_t>(machine::FaultCode::SENSOR_FAULT));
static_assert(StatusM::FAULT_STATION_FAULT == static_cast<uint16_t>(machine::FaultCode::STATION_FAULT));

// --- the shared slots (the callbacks hold them, never the registry: a callback may outlive it) -----------------------
struct MachineChannels::Slot
{
  explicit Slot(std::string s, std::chrono::milliseconds max_age) : service(std::move(s)), feed(max_age) {}
  const std::string service;
  std::string status_topic;    // resolved (remapped): the names the graph is counted on
  std::string request_topic;
  mutable std::mutex m;
  machine::Feed feed;
};

struct MachineChannels::Shared
{
  std::mutex m;
  std::vector<std::shared_ptr<Slot>> slots;
};

namespace
{
uint32_t randomSession()
{
  std::random_device rd;
  std::uniform_int_distribution<uint32_t> d(1u, UINT32_MAX);
  return d(rd);
}
}  // namespace

rclcpp::QoS MachineChannels::qos()
{
  // the REQUEST writer: the wire's profile (§3.2) -- RELIABLE, TRANSIENT_LOCAL, depth 1: level-held, the latest request
  // is the image a (re)joining source receives
  return rclcpp::QoS(1).reliable().transient_local();
}

rclcpp::QoS MachineChannels::statusQos()
{
  // the STATUS reader (protocol review fix 3): the wire's RELIABLE + TRANSIENT_LOCAL with a deeper READER history,
  // KEEP_LAST 20 -- two images that arrive between two deliveries of the registry's callback group are BOTH applied, in
  // order (with depth 1 the intermediate one was dropped: the image naming this process's request could be missed by
  // the consumed latch). QoS-compatible with the depth-1 writers (a history depth is not part of the requested /
  // offered match); the wire is unchanged; a late joiner still receives each writer's ONE latched image.
  return rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local();
}

machine::Image MachineChannels::imageOf(const StatusMsg& m)
{
  machine::Image img;
  img.source_session = m.source_session;
  img.heartbeat = m.heartbeat;
  img.state = m.state;
  img.cycle = {m.cycle_requester, m.cycle_seq};
  img.cycle_done = m.cycle_done;
  img.reject = {m.reject_requester, m.reject_seq};
  img.reject_code = m.reject_code;
  img.fault_code = m.fault_code;
  return img;
}

MachineChannels::MachineChannels(const rclcpp::Node::SharedPtr& node, Options options)
: node_(node),
  options_(options),
  requester_session_(options.requester_session != 0 ? options.requester_session : randomSession()),
  group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  shared_(std::make_shared<Shared>())
{
}

MachineChannels::~MachineChannels()
{
  if (timer_) timer_->cancel();
  std::lock_guard<std::mutex> lock(m_);
  endpoints_.clear();
}

void MachineChannels::prepare(const std::string& service)
{
  std::lock_guard<std::mutex> lock(m_);
  if (endpoints_.count(service)) return;
  auto slot = std::make_shared<Slot>(service, options_.max_age);
  {
    std::lock_guard<std::mutex> g(slot->m);
    slot->feed.subscribed(std::chrono::steady_clock::now());
  }
  rclcpp::SubscriptionOptions so;
  so.callback_group = group_;
  Endpoint e;
  e.sub = node_->create_subscription<StatusMsg>(
    machine::statusTopic(service), statusQos(),
    [slot, log = node_->get_logger()](StatusMsg::ConstSharedPtr msg) {
      const auto at = std::chrono::steady_clock::now();
      const machine::Image img = imageOf(*msg);
      std::string arriving;   // a source's status starts to reach this robot: told once, outside the lock
      {
        std::lock_guard<std::mutex> g(slot->m);
        arriving = slot->feed.arrival(img, at);
        slot->feed.image(img, at);
      }
      if (!arriving.empty()) RCLCPP_INFO(log, "machine service '%s': %s", slot->service.c_str(), arriving.c_str());
    },
    so);
  e.pub = node_->create_publisher<RequestMsg>(machine::requestTopic(service), qos());
  slot->status_topic = e.sub->get_topic_name();
  slot->request_topic = e.pub->get_topic_name();
  endpoints_[service] = e;
  {
    std::lock_guard<std::mutex> g(shared_->m);
    shared_->slots.push_back(slot);
  }
  if (!timer_)
  {
    // the publisher counts, every period, in the registry's group (the steady clock: never sim time). A count that
    // cannot be taken (the context is shutting down, or the query threw) keeps the last one.
    const std::weak_ptr<rclcpp::Node> weak = node_;
    const std::shared_ptr<Shared> shared = shared_;
    timer_ = node_->create_wall_timer(
      options_.count_period,
      [weak, shared]() {
        if (!rclcpp::ok()) return;   // shutting down: the graph is going away
        const auto node = weak.lock();
        if (!node) return;
        std::vector<std::shared_ptr<Slot>> slots;
        {
          std::lock_guard<std::mutex> g(shared->m);
          slots = shared->slots;
        }
        for (const auto& s : slots)
        {
          try
          {
            const int status = static_cast<int>(node->count_publishers(s->status_topic));
            const int request = static_cast<int>(node->count_publishers(s->request_topic));
            std::lock_guard<std::mutex> g(s->m);
            s->feed.publishers(status, request);
          }
          catch (const std::exception& ex)
          {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                 "machine service '%s': counting its publishers failed (%s); the last count stands",
                                 s->service.c_str(), ex.what());
          }
        }
      },
      group_);
  }
  RCLCPP_INFO(node_->get_logger(), "machine service '%s': status on %s, requests on %s (requester session %08x, "
              "status max age %lld ms)", service.c_str(), slot->status_topic.c_str(), slot->request_topic.c_str(),
              requester_session_, static_cast<long long>(options_.max_age.count()));
}

bool MachineChannels::prepared(const std::string& service) const
{
  std::lock_guard<std::mutex> lock(m_);
  return endpoints_.count(service) > 0;
}

std::vector<std::string> MachineChannels::services() const
{
  std::lock_guard<std::mutex> lock(m_);
  std::vector<std::string> out;
  for (const auto& [service, e] : endpoints_) out.push_back(service);
  return out;
}

void MachineChannels::setProcessObserver(std::shared_ptr<ObservedProcessController::Observer> observer)
{
  std::lock_guard<std::mutex> lock(m_);
  observer_ = std::move(observer);
}

void MachineChannels::reportStart() const
{
  const auto now = std::chrono::steady_clock::now();
  for (const auto& service : services())
  {
    const auto f = feed(service);
    if (!f) continue;
    const auto d = machine::startDiagnostic(service, *f, now);
    if (d.warning) RCLCPP_WARN(node_->get_logger(), "%s", d.text.c_str());
    else RCLCPP_INFO(node_->get_logger(), "%s", d.text.c_str());
  }
}

std::optional<machine::Feed> MachineChannels::feed(const std::string& service) const
{
  std::shared_ptr<Slot> slot;
  {
    std::lock_guard<std::mutex> g(shared_->m);
    for (const auto& s : shared_->slots)
      if (s->service == service) slot = s;
  }
  if (!slot) return std::nullopt;
  std::lock_guard<std::mutex> g(slot->m);
  return slot->feed;
}

std::optional<machine::Written> MachineChannels::lastWritten(const std::string& service) const
{
  std::lock_guard<std::mutex> lock(process_m_);
  const auto it = last_written_.find(service);
  if (it == last_written_.end()) return std::nullopt;
  return it->second;
}

std::optional<machine::Identity> MachineChannels::write(const std::string& service, uint32_t source_session,
                                                        machine::SteadyTime now, std::optional<int64_t> runner_ns)
{
  rclcpp::Publisher<RequestMsg>::SharedPtr pub;
  std::shared_ptr<ObservedProcessController::Observer> observer;
  {
    std::lock_guard<std::mutex> lock(m_);
    const auto it = endpoints_.find(service);
    if (it == endpoints_.end()) return std::nullopt;
    pub = it->second.pub;
    observer = observer_;
  }
  machine::Identity id;
  id.requester = requester_session_;
  {
    std::lock_guard<std::mutex> lock(process_m_);
    auto it = seq_.find(service);
    if (it == seq_.end()) it = seq_.emplace(service, machine::SeqCounter(options_.initial_seq)).first;
    id.seq = it->second.next(source_session);
  }
  RequestMsg msg;
  msg.header.stamp = node_->now();   // the runner's clock: diagnosis only
  msg.source_session = source_session;
  msg.requester_session = id.requester;
  msg.seq = id.seq;
  const machine::Written written{source_session, id, now};   // `now`: the 313 "not yet taken" says how long ago
  // the feed watches for the image that names it (its consumption, latched) -- BEFORE the publish: it cannot be missed
  std::shared_ptr<Slot> slot;
  {
    std::lock_guard<std::mutex> g(shared_->m);
    for (const auto& s : shared_->slots)
      if (s->service == service) slot = s;
  }
  if (slot)
  {
    std::lock_guard<std::mutex> g(slot->m);
    slot->feed.wrote(written);
  }
  try
  {
    pub->publish(msg);   // level-held: the image stays until the next request
  }
  catch (...)
  {
    // C++ review M2: the request never reached the wire and nothing below records it -- the feed watches the PREVIOUS
    // written request again (still this process's last written one), never a lock-out on an identity no image can
    // name; the node fails as before ("could not be written", no station code)
    if (slot)
    {
      std::lock_guard<std::mutex> g(slot->m);
      slot->feed.unwrote();
    }
    throw;
  }
  if (slot)
  {
    std::lock_guard<std::mutex> g(slot->m);
    slot->feed.kept();   // on the wire: nothing to restore (an unwrote() now changes nothing)
  }
  {
    std::lock_guard<std::mutex> lock(process_m_);
    last_written_[service] = written;
  }
  {
    std::lock_guard<std::mutex> lock(job_m_);
    machine::Outstanding o;
    o.written = written;
    o.at = now;
    o.runner_ns = runner_ns;
    outstanding_[service] = o;
  }
  // ADR-0030: a machine request actually written is an execution event (kind `request`), on the tick thread. It is
  // OBSERVATION (C++ review M1): the request is on the wire and recorded -- an observer that throws is logged, never a
  // failed request step (as activity())
  if (observer && *observer)
  {
    ProcessEvent e;
    e.type = ProcessEventType::REQUEST;
    e.channel = service;
    e.value = 0.0;
    try
    {
      (*observer)(e);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(node_->get_logger(), "machine service '%s': the request was written; its execution event was not "
                  "reported (%s)", service.c_str(), ex.what());
    }
    catch (...)
    {
      RCLCPP_WARN(node_->get_logger(), "machine service '%s': the request was written; its execution event was not "
                  "reported (an unknown error)", service.c_str());
    }
  }
  return id;
}

void MachineChannels::beginJob(ActivitySink sink)
{
  std::lock_guard<std::mutex> lock(job_m_);
  outstanding_.clear();
  failure_code_ = 0;
  failure_text_.clear();
  failed_ = false;
  sink_ = std::move(sink);
}

void MachineChannels::endJob()
{
  std::lock_guard<std::mutex> lock(job_m_);
  sink_ = nullptr;
}

void MachineChannels::activity(const std::string& text) const
{
  ActivitySink sink;
  {
    std::lock_guard<std::mutex> lock(job_m_);
    sink = sink_;
  }
  if (!sink) return;
  try
  {
    sink(text);   // the RTI detail (D5): observation, never a reason for the tick to fail
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(node_->get_logger(), "machine activity not reported (%s): %s", e.what(), text.c_str());
  }
}

std::optional<machine::Outstanding> MachineChannels::outstanding(const std::string& service) const
{
  std::lock_guard<std::mutex> lock(job_m_);
  const auto it = outstanding_.find(service);
  if (it == outstanding_.end()) return std::nullopt;
  return it->second;
}

void MachineChannels::markCompleted(const std::string& service)
{
  std::lock_guard<std::mutex> lock(job_m_);
  const auto it = outstanding_.find(service);
  if (it != outstanding_.end()) it->second.completed = true;
}

void MachineChannels::latchFailure(uint16_t code, const std::string& text)
{
  std::lock_guard<std::mutex> lock(job_m_);
  // the FIRST failure stands; a STATION code replaces an earlier failure that was not a machine condition (D3/D4)
  if (failed_ && !(failure_code_ == 0 && code != 0)) return;
  failed_ = true;
  failure_code_ = code;
  failure_text_ = text;
}

uint16_t MachineChannels::jobStationCode() const
{
  std::lock_guard<std::mutex> lock(job_m_);
  return failure_code_;
}

std::string MachineChannels::jobStationText() const
{
  std::lock_guard<std::mutex> lock(job_m_);
  return failure_text_;
}

}  // namespace trainit
