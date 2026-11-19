// robot_task_server.cpp -- the RobotTaskServer's ROS binding (ADR-0032). See the header.
#include "trainit_motion_runtime/robot_task_server.hpp"

#include <cstdio>
#include <random>
#include <sstream>

namespace trainit::rti
{

// The pure vocabulary IS the message's (one source of truth: the type; checked here at compile time).
using StatusM = trainit_interfaces::msg::RobotTaskStatus;
using CommandM = trainit_interfaces::msg::RobotTaskCommand;
static_assert(StatusM::INTERFACE_VERSION == kInterfaceVersion);
static_assert(CommandM::CMD_START == static_cast<uint8_t>(Cmd::START));
static_assert(CommandM::CMD_CANCEL == static_cast<uint8_t>(Cmd::CANCEL));
static_assert(CommandM::CMD_RESET == static_cast<uint8_t>(Cmd::RESET));
static_assert(StatusM::STATE_UNAVAILABLE == static_cast<uint8_t>(State::UNAVAILABLE));
static_assert(StatusM::STATE_IDLE == static_cast<uint8_t>(State::IDLE));
static_assert(StatusM::STATE_RUNNING == static_cast<uint8_t>(State::RUNNING));
static_assert(StatusM::STATE_STOPPING == static_cast<uint8_t>(State::STOPPING));
static_assert(StatusM::STATE_FAULTED == static_cast<uint8_t>(State::FAULTED));
static_assert(StatusM::ACK_ACCEPTED == static_cast<uint8_t>(Ack::ACCEPTED));
static_assert(StatusM::ACK_REJECTED == static_cast<uint8_t>(Ack::REJECTED));
static_assert(StatusM::RESULT_SUCCEEDED == static_cast<uint8_t>(Result::SUCCEEDED));
static_assert(StatusM::RESULT_ENDED == static_cast<uint8_t>(Result::ENDED));
static_assert(StatusM::RESULT_CANCELLED == static_cast<uint8_t>(Result::CANCELLED));
static_assert(StatusM::RESULT_FAILED == static_cast<uint8_t>(Result::FAILED));
static_assert(StatusM::CODE_OK == code::OK && StatusM::CODE_BIN_EMPTY == code::BIN_EMPTY &&
              StatusM::CODE_NOTHING_PICKABLE == code::NOTHING_PICKABLE &&
              StatusM::CODE_PICKABLE_EXHAUSTED == code::PICKABLE_EXHAUSTED &&
              StatusM::CODE_RECOVERY_EXHAUSTED == code::RECOVERY_EXHAUSTED && StatusM::CODE_CANCELLED == code::CANCELLED &&
              StatusM::CODE_TERMINAL_FAULT == code::TERMINAL_FAULT && StatusM::CODE_TASK_FAILED == code::TASK_FAILED &&
              StatusM::CODE_TASK_TIMEOUT == code::TASK_TIMEOUT && StatusM::CODE_REQUESTER_LOST == code::REQUESTER_LOST &&
              StatusM::CODE_RUNTIME_LOST == code::RUNTIME_LOST && StatusM::CODE_STATION_TIMEOUT == code::STATION_TIMEOUT &&
              StatusM::CODE_STATION_FAULT == code::STATION_FAULT &&
              StatusM::CODE_CANCELLED_WHILE_HOLDING == code::CANCELLED_WHILE_HOLDING &&
              StatusM::CODE_EEF_FAULT == code::EEF_FAULT &&
              StatusM::CODE_CANCELLED_HOLD_UNKNOWN == code::CANCELLED_HOLD_UNKNOWN &&
              StatusM::CODE_STATION_UNAVAILABLE == code::STATION_UNAVAILABLE &&
              StatusM::CODE_SERVER_STARTED == code::SERVER_STARTED &&
              StatusM::CODE_REQUESTER_CONFLICT == code::REQUESTER_CONFLICT &&
              StatusM::CODE_STATION_BUSY == code::STATION_BUSY);   // P0.b: appended (trainit_interfaces 0.5.0)
static_assert(StatusM::REJECT_INVALID_REQUEST == code::INVALID_REQUEST &&
              StatusM::REJECT_UNKNOWN_TASK == code::UNKNOWN_TASK && StatusM::REJECT_BUSY == code::BUSY &&
              StatusM::REJECT_NOT_READY == code::NOT_READY && StatusM::REJECT_FAULTED == code::FAULTED &&
              StatusM::REJECT_DUPLICATE_JOB == code::DUPLICATE_JOB &&
              StatusM::REJECT_NOT_CURRENT_JOB == code::NOT_CURRENT_JOB &&
              StatusM::REJECT_INVALID_STATE == code::INVALID_STATE &&
              StatusM::REJECT_SEQUENCE_ERROR == code::SEQUENCE_ERROR &&
              StatusM::REJECT_NOT_SYNCHRONIZED == code::NOT_SYNCHRONIZED);

uint32_t newServerSession()
{
  std::random_device rd;
  std::uniform_int_distribution<uint32_t> d(1u, UINT32_MAX);
  return d(rd);
}

rclcpp::QoS RobotTaskServer::qos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

template <class F>
void RobotTaskServer::guarded(const char* what, F&& body)
{
  try
  {
    body();
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(node_->get_logger(), "RTI: internal error in the %s callback: %s -- serving ends: the motion is "
                 "stopped and the server shuts down", what, e.what());
  }
  catch (...)
  {
    RCLCPP_FATAL(node_->get_logger(), "RTI: internal error in the %s callback -- serving ends: the motion is "
                 "stopped and the server shuts down", what);
  }
  failed_ = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    if (timer_) timer_->cancel();
  }
  // ORDERLY: the pre-shutdown callback (StopOnShutdown) stops the motion in flight, the tick thread halts the tree
  // and returns, the process exits (trainit_task_server: exit 3). The requester sees the heartbeat stop (F9).
  rclcpp::shutdown(nullptr, "RTI internal error");
}

std::optional<RobotTaskServer::Params> RobotTaskServer::readParams(const rclcpp::Node::SharedPtr& node,
                                                                   std::string& refused)
{
  auto get = [&](const char* name, int64_t def) {
    if (!node->has_parameter(name)) node->declare_parameter<int64_t>(name, def);
    return node->get_parameter(name).as_int();
  };
  const int64_t period = get("rti_status_period_ms", 100);
  const int64_t timeout = get("rti_requester_timeout_ms", 1000);
  const int64_t stop = get("rti_stop_timeout_ms", 5000);
  auto bounded = [](int64_t v) { return v >= 50 && v <= 10000; };
  if (!bounded(period))
  {
    refused = "rti_status_period_ms " + std::to_string(period) + " (bounds 50..10000 ms)";
    return std::nullopt;
  }
  if (!bounded(timeout) || timeout < 2 * period)
  {
    refused = "rti_requester_timeout_ms " + std::to_string(timeout) +
              " (bounds 50..10000 ms, and at least two status periods)";
    return std::nullopt;
  }
  if (stop < 50 || stop > 60000 || stop < 2 * period)
  {
    refused = "rti_stop_timeout_ms " + std::to_string(stop) + " (bounds 50..60000 ms, and at least two status periods)";
    return std::nullopt;
  }
  Params p;
  p.period = std::chrono::milliseconds(period);
  p.requester_timeout = std::chrono::milliseconds(timeout);
  p.stop_timeout = std::chrono::milliseconds(stop);
  return p;
}

RobotTaskServer::RobotTaskServer(const rclcpp::Node::SharedPtr& node, const rclcpp::CallbackGroup::SharedPtr& group,
                                 Params params, ExportTable table, JobExecutor& executor, RuntimeProbe& probe)
: node_(node),
  params_(params),
  executor_(executor),
  probe_(probe),
  session_(newServerSession()),
  machine_(Config{params.requester_timeout, 64, params.stop_timeout}, std::move(table))
{
  status_pub_ = node_->create_publisher<StatusMsg>(kStatusTopic, qos());
  rclcpp::SubscriptionOptions so;
  so.callback_group = group;
  command_sub_ = node_->create_subscription<CommandMsg>(
    kCommandTopic, qos(), [this](const CommandMsg::SharedPtr msg) { guarded("command", [&] { onCommand(*msg); }); },
    so);
  command_topic_ = command_sub_->get_topic_name();
  timer_ = node_->create_wall_timer(params_.period, [this]() { guarded("status period", [&] { onTimer(); }); },
                                    group);   // steady: never sim time
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishLocked();
  }
  RCLCPP_INFO(node_->get_logger(),
              "RTI v%u: server session %08x, %zu exported task(s), export table id %u, on %s / %s; status every "
              "%lld ms, requester timeout %lld ms, stop timeout %lld ms. FAULTED / SERVER_STARTED until a RESET "
              "(ADR-0032)",
              kInterfaceVersion, session_, machine_.table().entries.size(), machine_.table().id,
              command_topic_.c_str(), status_pub_->get_topic_name(),
              static_cast<long long>(params_.period.count()), static_cast<long long>(params_.requester_timeout.count()),
              static_cast<long long>(params_.stop_timeout.count()));
  for (const auto& e : machine_.table().entries)
    RCLCPP_INFO(node_->get_logger(), "RTI exported task #%u %s (timeout %.3f s, tree %s)", e.number, e.name.c_str(),
                e.timeout_s, e.treeId().c_str());
}

RobotTaskServer::~RobotTaskServer()
{
  shutdown();
}

void RobotTaskServer::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
  if (timer_) timer_->cancel();
}

std::string RobotTaskServer::runId(uint64_t counter) const
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%08x-%llu", session_, static_cast<unsigned long long>(counter));
  return buf;
}

Status RobotTaskServer::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return machine_.status();
}

std::optional<int> RobotTaskServer::countPublishers() const
{
  if (!rclcpp::ok()) return std::nullopt;   // shutting down: the graph is going away
  try
  {
    const int n = static_cast<int>(node_->count_publishers(command_topic_));
    last_publishers_ = n;
    return n;
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "RTI: counting the command publishers failed (%s); the last count stands", e.what());
    return std::nullopt;
  }
}

bool RobotTaskServer::probeReady(std::string& why)
{
  try
  {
    return probe_.ready(why);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "RTI: the runtime check threw: %s",
                         e.what());
  }
  catch (...)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "RTI: the runtime check threw");
  }
  why = "the runtime check failed";
  return false;
}


void RobotTaskServer::publishLocked()
{
  const Status& s = machine_.status();
  StatusMsg m;
  m.header.stamp = node_->now();   // the node clock (sim time under use_sim_time): ADR-0030 timing only
  m.interface_version = kInterfaceVersion;
  m.export_table_id = s.export_table_id;
  m.server_session = session_;
  m.heartbeat = heartbeat_;
  m.state = static_cast<uint8_t>(s.state);
  m.ready = s.ready;
  m.ack_seq = s.ack_seq;
  m.ack = static_cast<uint8_t>(s.ack);
  m.ack_code = s.ack_code;
  m.job_id = s.job_id;
  m.task = s.task;
  m.result = static_cast<uint8_t>(s.result);
  m.result_code = s.result_code;
  m.fault_code = s.fault_code;
  m.task_name = s.task_name;
  m.detail = s.detail;
  status_pub_->publish(m);
}

void RobotTaskServer::logTransition(const Status& b, const Status& a, const char* why)
{
  const auto log = node_->get_logger();
  if (a.ack_seq != b.ack_seq || a.ack != b.ack || a.ack_code != b.ack_code)
  {
    if (a.ack == Ack::NONE)
      RCLCPP_INFO(log, "RTI: requester adopted (baseline seq %u)", a.ack_seq);
    else
      RCLCPP_INFO(log, "RTI: %s seq %u: %s%s%s", why, a.ack_seq, ackName(a.ack), a.ack_code ? " " : "",
                  codeName(a.ack_code));
  }
  if (a.result != b.result || a.result_code != b.result_code || (a.job_id != b.job_id && a.result != Result::NONE))
    if (a.result != Result::NONE)
      RCLCPP_INFO(log, "RTI: job %u (%s #%u) ended: %s %s", a.job_id, a.task_name.c_str(), a.task,
                  resultName(a.result), codeName(a.result_code));
  if (a.state != b.state)
  {
    if (a.state == State::FAULTED || a.state == State::UNAVAILABLE || a.state == State::STOPPING)
      RCLCPP_WARN(log, "RTI: state %s -> %s%s%s%s", stateName(b.state), stateName(a.state),
                  a.fault_code ? " " : "", a.fault_code ? codeName(a.fault_code) : "",
                  a.detail.empty() ? "" : (" -- " + a.detail).c_str());
    else
      RCLCPP_INFO(log, "RTI: state %s -> %s%s", stateName(b.state), stateName(a.state),
                  a.state == State::RUNNING ? (" (job " + std::to_string(a.job_id) + ", " + a.task_name + ")").c_str()
                                            : "");
  }
}

void RobotTaskServer::onCommand(const CommandMsg& msg)
{
  Effects fx;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) return;
    Command img;
    img.requester_session = msg.requester_session;
    img.heartbeat = msg.heartbeat;
    img.cmd_seq = msg.cmd_seq;
    img.cmd = msg.cmd;
    img.job_id = msg.job_id;
    img.task = msg.task;
    std::optional<int> pubs;
    if (msg.requester_session != last_image_session_)
    {
      pubs = countPublishers();   // Decision 10: counted again whenever a session change is seen (nullopt: the last)
      last_image_session_ = msg.requester_session;
    }
    const Status before = machine_.status();
    fx = machine_.onCommand(img, std::chrono::steady_clock::now(), pubs);
    const Status& after = machine_.status();
    if (after != before)
    {
      logTransition(before, after, cmdName(msg.cmd));
      publishLocked();
    }
  }
  execute(fx);
}

void RobotTaskServer::onTimer()
{
  // the facts first, outside the lock (graph queries)
  if (!rclcpp::ok()) return;   // shutting down: no check (the graph is going away)
  std::string why;
  const bool ready = probeReady(why);
  const int pubs = countPublishers().value_or(last_publishers_.load());
  Effects fx;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) return;
    ++heartbeat_;
    const Status before = machine_.status();
    fx = machine_.onCheck(ready, why, pubs, std::chrono::steady_clock::now());
    const Status& after = machine_.status();
    if (after != before) logTransition(before, after, "check");
    // ADR-0032 Decision 8: while STOPPING, the stop is RE-ASSERTED every period -- a motion that started after the
    // request (the tick was planning, or in a gripper step) is stopped within one period, never run to its end
    if (!fx.stop)
      if (const auto job = machine_.stoppingJob())
      {
        fx.stop = *job;
        fx.stop_reassert = true;
      }
    publishLocked();   // every period: the heartbeat
  }
  execute(fx);
}

std::optional<Decided> RobotTaskServer::jobEnded(uint64_t counter, const JobEnd& end)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const Status before = machine_.status();
  const auto d = machine_.onJobEnded(counter, end, std::chrono::steady_clock::now());
  const Status& after = machine_.status();
  if (!d && after == before) return d;   // not the current job's report (ignored)
  if (!d)
    RCLCPP_WARN(node_->get_logger(), "RTI: job run %s returned after its stop timed out (its result stands)",
                runId(counter).c_str());
  logTransition(before, after, "end");
  if (!stopped_) publishLocked();
  return d;
}

void RobotTaskServer::jobActivity(uint64_t counter, const std::string& text)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_) return;
  const Status before = machine_.status();
  machine_.onActivity(counter, text);
  const Status& after = machine_.status();
  if (after == before) return;   // not the RUNNING job's, or the same text
  RCLCPP_INFO(node_->get_logger(), "RTI: job %u: %s", after.job_id, after.detail.c_str());
  publishLocked();
}

void RobotTaskServer::execute(const Effects& fx)
{
  // the start first: a stop for the job just started (an input that starts and stops it at once) must find it
  // pending, or it would be dropped until the next re-assertion (P0.a protocol review, minor 1)
  if (fx.start) executor_.start(*fx.start);
  if (fx.stop)
  {
    if (!fx.stop_reassert)
      RCLCPP_WARN(node_->get_logger(), "RTI: stopping job run %s (MotionRuntime::stop, then the halt; re-asserted "
                  "every period until the job ends)", runId(*fx.stop).c_str());
    executor_.requestStop(*fx.stop);
  }
}

}  // namespace trainit::rti
