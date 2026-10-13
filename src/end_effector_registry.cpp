// =============================================================================
// end_effector_registry.cpp — the runner's end-effector devices (ADR-0022).
//
// Every wait runs on the steady clock, in short slices, and ends COMMAND_ABORTED ("interrupted:
// the ROS context shut down") once the context shuts down; every stamp compared with an
// adapter's is the runner's ROS clock (node->now()). Every endpoint name is RELATIVE to the
// runner's node, so a runner in /r1 reaches /r1's adapter and /r1's controllers only (D7).
// =============================================================================
#include "trainit_motion_runtime/end_effector_registry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <initializer_list>
#include <limits>
#include <set>
#include <sstream>
#include <thread>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace trainit
{

namespace
{
using Act = trainit_interfaces::msg::EndEffectorActuatorState;
using SteadyClock = std::chrono::steady_clock;
using std::chrono::milliseconds;

constexpr char kSep = '/';
constexpr int64_t kNsPerMs = 1000000;
// The longest wait or tolerance, in ms (24.8 days): the parser accepts any positive int64, as the
// reference does, so every ms value is saturated before a chrono or nanosecond conversion.
constexpr int64_t kMaxMs = std::numeric_limits<int32_t>::max();
constexpr milliseconds kSlice{10};   // a wait's slice: a shut-down context ends it at once
constexpr const char* kInterrupted = "interrupted: the ROS context shut down";

int64_t saturatedMs(int64_t ms) { return std::clamp<int64_t>(ms, 0, kMaxMs); }

enum class Wait : uint8_t
{
  READY,
  TIMEOUT,
  INTERRUPTED,   // the ROS context shut down
};

// Waits for `future` up to `timeout_ms` on the steady clock, in slices.
template <typename Future>
Wait waitReady(const Future& future, int64_t timeout_ms, const rclcpp::Context::SharedPtr& context)
{
  const auto deadline = SteadyClock::now() + milliseconds(saturatedMs(timeout_ms));
  for (;;)
  {
    if (future.wait_for(milliseconds(0)) == std::future_status::ready) return Wait::READY;
    if (!rclcpp::ok(context)) return Wait::INTERRUPTED;
    const auto now = SteadyClock::now();
    if (now >= deadline) return Wait::TIMEOUT;
    (void)future.wait_for(std::min<SteadyClock::duration>(deadline - now, kSlice));
  }
}

// Asks the server to cancel `handle`. False when the client no longer knows the goal -- it reached a
// terminal state meanwhile (rclcpp_action throws UnknownGoalHandleError) -- or the request could
// not be sent: never an exception out of a tick or an executor callback.
template <typename ActionT>
bool cancelGoal(rclcpp_action::Client<ActionT>& client,
                const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& handle)
{
  try
  {
    client.async_cancel_goal(handle);
    return true;
  }
  catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
  {
    return false;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

// "a/b/c": a relative ROS name from tokens.
std::string rel(std::initializer_list<std::string> parts)
{
  std::string s;
  for (const auto& p : parts)
  {
    if (!s.empty()) s += kSep;
    s += p;
  }
  return s;
}

int64_t toNs(const builtin_interfaces::msg::Time& t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

std::string joinComma(const std::vector<std::string>& v)
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + v[i];
  return s;
}

eef::GoalStatus statusOf(rclcpp_action::ResultCode code)
{
  switch (code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED: return eef::GoalStatus::SUCCEEDED;
    case rclcpp_action::ResultCode::ABORTED: return eef::GoalStatus::ABORTED;
    case rclcpp_action::ResultCode::CANCELED: return eef::GoalStatus::CANCELED;
    default: return eef::GoalStatus::UNKNOWN;
  }
}

EefResult ok()
{
  EefResult r;
  r.ok = true;
  return r;
}

// "sensor;sim_ground_truth" or "sensor, sim_ground_truth" -> tokens
std::vector<std::string> splitList(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  auto flush = [&] {
    const auto a = cur.find_first_not_of(' ');
    const auto b = cur.find_last_not_of(' ');
    if (a != std::string::npos) out.push_back(cur.substr(a, b - a + 1));
    cur.clear();
  };
  for (const char c : s)
  {
    if (c == ';' || c == ',') flush();
    else cur += c;
  }
  flush();
  return out;
}

// A literal an int port reads as a positive int (BT.CPP converts the literal at tick time).
bool isPositiveIntText(const std::string& s)
{
  if (s.empty() || s.size() > 9 || !std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; }))
    return false;
  return std::stol(s) > 0;
}
}  // namespace

// --- the shared slots callbacks write into (never `this`: a callback may outlive a tick) ---------
struct EndEffectorDevice::Feed
{
  mutable std::mutex m;
  std::shared_ptr<const State> latest;
  SteadyClock::time_point arrival{};
  uint64_t seq{0};
};

struct EndEffectorDevice::GoalWatch
{
  std::atomic<bool> abandoned{false};    // the client stopped waiting for the goal response
  std::atomic<bool> cancel_sent{false};
  bool may_cancel{true};
  std::mutex m;
  bool has_feedback{false};
  std::vector<std::string> joints;       // FollowJointTrajectory: the newest feedback
  std::vector<double> error;
};

// =================================================================================================
// EndEffectorDevice
// =================================================================================================
EndEffectorDevice::EndEffectorDevice(const rclcpp::Node::SharedPtr& node, const eef::DeviceSpec& spec)
: node_(node), spec_(spec), feed_(std::make_shared<Feed>())
{
  identity_ = rclcpp::expand_topic_or_service_name(rel({"eef", spec_.id}), node_->get_name(),
                                                   node_->get_namespace(), false);
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions so;
  so.callback_group = group_;
  const std::shared_ptr<Feed> feed = feed_;
  state_sub_ = node_->create_subscription<State>(
    rel({"eef", spec_.id, "state"}), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [feed](std::shared_ptr<const State> msg) {
      std::lock_guard<std::mutex> g(feed->m);
      feed->latest = std::move(msg);
      feed->arrival = SteadyClock::now();
      ++feed->seq;
    },
    so);
  state_topic_ = state_sub_->get_topic_name();
  if (spec_.scene_signal == eef::SceneSignal::TMR)
    scene_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      rel({"eef", spec_.id, "engaged"}), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  for (const auto& a : spec_.actuators)
  {
    switch (a.command_interface)
    {
      case eef::CommandInterface::SET_BOOL:
        set_bool_[a.id] = node_->create_client<SetBool>(rel({"eef", spec_.id, "actuators", a.id, "set_engaged"}),
                                                        rmw_qos_profile_services_default, group_);
        break;
      case eef::CommandInterface::GRIPPER_COMMAND:
        gripper_[a.id] = rclcpp_action::create_client<GripperCommand>(node_, a.action, group_);
        break;
      case eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY:
        trajectory_[a.id] = rclcpp_action::create_client<FollowJointTrajectory>(node_, a.action, group_);
        break;
      case eef::CommandInterface::UNKNOWN:
        break;
    }
  }
}

std::string EndEffectorDevice::stateTopic() const { return state_topic_; }

std::optional<std::size_t> EndEffectorDevice::statePublishers() const
{
  // rcl_count_publishers fails once the context is shut down, and rclcpp then throws
  if (!rosOk()) return std::nullopt;
  try
  {
    return node_->count_publishers(state_topic_);
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}

bool EndEffectorDevice::rosOk() const { return rclcpp::ok(node_->get_node_base_interface()->get_context()); }

EefResult EndEffectorDevice::interrupted(const std::string& subject) const
{
  return failure(eef::Reason::COMMAND_ABORTED, subject, kInterrupted);
}

EndEffectorDevice::Snapshot EndEffectorDevice::latest() const
{
  std::lock_guard<std::mutex> g(feed_->m);
  Snapshot s;
  s.sample = feed_->latest;
  s.arrival = feed_->arrival;
  s.seq = feed_->seq;
  return s;
}

std::map<std::string, eef::SentCommand> EndEffectorDevice::sent() const
{
  std::lock_guard<std::mutex> g(sent_m_);
  return sent_;
}

void EndEffectorDevice::recordSent(const std::string& actuator, uint8_t kind, int64_t sent_ns, int64_t prior_ns)
{
  std::lock_guard<std::mutex> g(sent_m_);
  eef::SentCommand c;
  c.command = kind;
  c.stamp_ns = sent_ns;
  c.prior_ns = prior_ns;
  sent_[actuator] = c;
}

EndEffectorDevice::Prior EndEffectorDevice::priorOf(const std::string& actuator) const
{
  const Snapshot snap = latest();
  Prior p;
  p.seq = snap.seq;
  if (!snap.sample || snap.sample->end_effector != identity_) return p;
  for (const auto& e : snap.sample->actuators)
    if (e.actuator == actuator)
    {
      p.stamp_ns = toNs(e.last_command_stamp);
      break;
    }
  return p;
}

void EndEffectorDevice::publishSceneSignal(bool engaged)
{
  if (!scene_pub_) return;
  std_msgs::msg::Bool m;
  m.data = engaged;
  scene_pub_->publish(m);
}

int64_t EndEffectorDevice::nowNs() const { return node_->now().nanoseconds(); }

uint8_t EndEffectorDevice::commandKind(const eef::ActuatorSpec& a, const std::string& target) const
{
  if (a.command_interface == eef::CommandInterface::SET_BOOL)
    return target == "engage" ? Act::COMMAND_ENGAGE : Act::COMMAND_RELEASE;
  const eef::SiteSpec* s = spec_.siteGrippedBy(a.id);
  if (s && target == s->engage) return Act::COMMAND_ENGAGE;
  if (s && target == s->release) return Act::COMMAND_RELEASE;
  return Act::COMMAND_TARGET;
}

EefResult EndEffectorDevice::failure(eef::Reason r, std::string subject, std::string detail) const
{
  EefResult out;
  out.reason = r;
  out.subject = std::move(subject);
  out.detail = std::move(detail);
  return out;
}

EefResult EndEffectorDevice::command(const std::string& actuator, const std::string& target,
                                     std::optional<int64_t> timeout_ms)
{
  const eef::ActuatorSpec* a = spec_.actuator(actuator);
  if (a == nullptr)
    return failure(eef::Reason::UNKNOWN_ACTUATOR, eef::actuatorIdentity(identity_, actuator), "no such actuator in the contract");
  if (!a->hasTarget(target))
    return failure(eef::Reason::UNKNOWN_TARGET, eef::actuatorIdentity(identity_, actuator),
                   "'" + target + "' is not a named target of " + actuator);
  const int64_t cto = timeout_ms.value_or(spec_.command_timeout_ms);
  switch (a->command_interface)
  {
    case eef::CommandInterface::SET_BOOL: return commandSetBool(*a, target, cto);
    case eef::CommandInterface::GRIPPER_COMMAND: return commandGripper(*a, target, cto);
    case eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY: return commandTrajectory(*a, target, cto);
    case eef::CommandInterface::UNKNOWN: break;
  }
  return failure(eef::Reason::CONTRACT_MISMATCH, eef::actuatorIdentity(identity_, actuator), "no command interface");
}

EefResult EndEffectorDevice::commandSetBool(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms)
{
  const auto& client = set_bool_.at(a.id);
  const std::string subject = eef::actuatorIdentity(identity_, a.id);
  eef::ObservedCommand oc;
  oc.command_interface = eef::CommandInterface::SET_BOOL;
  if (!rosOk()) return interrupted(subject);
  if (!client->wait_for_service(milliseconds(eef::kServerWaitMs)))
  {
    if (!rosOk()) return interrupted(subject);   // the wait ends early once the context shuts down
    oc.phase = eef::Phase::NO_SERVER;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail + " on '" + client->get_service_name() + "'");
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = target == "engage";   // data = (target == engage)
  const uint8_t kind = commandKind(a, target);
  const Prior prior = priorOf(a.id);
  const int64_t sent_ns = nowNs();
  auto future = client->async_send_request(request);
  const Wait answered = waitReady(future, timeout_ms, node_->get_node_base_interface()->get_context());
  if (answered != Wait::READY)
  {
    client->remove_pending_request(future);
    if (answered == Wait::INTERRUPTED) return interrupted(subject);
    oc.phase = eef::Phase::NO_RESPONSE;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail + " (" + std::to_string(timeout_ms) + " ms); the state shows whether it applied");
  }
  const auto response = future.get();
  oc.phase = eef::Phase::RESULT;
  oc.success = response->success;
  oc.message = response->message;
  const auto o = eef::mapOutcome(oc);
  if (!o.done) return failure(o.reason, subject, o.detail);
  recordSent(a.id, kind, sent_ns, prior.stamp_ns);
  return confirm(a, kind, sent_ns, prior);
}

EefResult EndEffectorDevice::commandGripper(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;
  const auto& client = gripper_.at(a.id);
  const std::string subject = eef::actuatorIdentity(identity_, a.id);
  const eef::SiteSpec* site = spec_.siteGrippedBy(a.id);
  const auto context = node_->get_node_base_interface()->get_context();
  eef::ObservedCommand oc;
  oc.command_interface = eef::CommandInterface::GRIPPER_COMMAND;
  oc.result_policy = a.result_policy;
  oc.engage_target = site != nullptr && target == site->engage;
  if (!rosOk()) return interrupted(subject);
  if (!client->wait_for_action_server(milliseconds(eef::kServerWaitMs)))
  {
    if (!rosOk()) return interrupted(subject);
    oc.phase = eef::Phase::NO_SERVER;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail + " on '" + a.action + "'");
  }
  // No TrainIt state on this device: its server is a third-party (ros2_control) controller.
  const auto publishers = statePublishers();
  if (!publishers) return interrupted(subject);
  oc.ros2_control = *publishers == 0;
  auto watch = std::make_shared<GoalWatch>();
  watch->may_cancel = eef::mayCancel(oc.command_interface, oc.ros2_control);

  GripperCommand::Goal goal;
  goal.command.position = a.targets.at(target).front();
  goal.command.max_effort = a.max_effort.value_or(0.0);
  rclcpp_action::Client<GripperCommand>::SendGoalOptions options;
  const std::weak_ptr<rclcpp_action::Client<GripperCommand>> weak = client;
  options.goal_response_callback = [watch, weak](GoalHandle::SharedPtr handle) {
    if (handle && watch->abandoned.load() && watch->may_cancel && !watch->cancel_sent.exchange(true))
      if (auto c = weak.lock()) cancelGoal(*c, handle);   // false: the goal already ended
  };
  const uint8_t kind = commandKind(a, target);
  const Prior prior = priorOf(a.id);
  const int64_t sent_ns = nowNs();
  auto handle_future = client->async_send_goal(goal, options);
  const Wait responded = waitReady(handle_future, eef::kGoalResponseMs, context);
  if (responded == Wait::INTERRUPTED) return interrupted(subject);   // HOLD: nothing is cancelled
  if (responded == Wait::TIMEOUT)
  {
    watch->abandoned = true;
    if (handle_future.wait_for(milliseconds(0)) == std::future_status::ready)
    {
      auto late = handle_future.get();
      if (late && watch->may_cancel && !watch->cancel_sent.exchange(true)) cancelGoal(*client, late);
    }
    oc.phase = eef::Phase::NO_RESPONSE;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail);
  }
  const auto handle = handle_future.get();
  if (!handle)
  {
    oc.phase = eef::Phase::REJECTED;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail);
  }
  recordSent(a.id, kind, sent_ns, prior.stamp_ns);   // an accepted goal is APPLIED
  auto result_future = client->async_get_result(handle);
  const int64_t deadline_ms = eef::resultDeadlineMs(a, timeout_ms);
  const Wait finished = waitReady(result_future, deadline_ms, context);
  if (finished == Wait::INTERRUPTED) return interrupted(subject);
  if (finished == Wait::TIMEOUT)
  {
    // a cancel that throws means the goal reached a terminal state meanwhile: its result, when it
    // is already here, is mapped below; otherwise the deadline stands
    const bool cancel_failed = watch->may_cancel && !cancelGoal(*client, handle);
    if (!cancel_failed || result_future.wait_for(milliseconds(0)) != std::future_status::ready)
    {
      oc.phase = eef::Phase::NO_RESULT;
      const auto o = eef::mapOutcome(oc);
      return failure(o.reason, subject, o.detail + " (" + std::to_string(deadline_ms) + " ms)");
    }
  }
  const auto wrapped = result_future.get();
  oc.phase = eef::Phase::RESULT;
  oc.status = statusOf(wrapped.code);
  if (wrapped.result)
  {
    oc.reached_goal = wrapped.result->reached_goal;
    oc.stalled = wrapped.result->stalled;
  }
  const auto o = eef::mapOutcome(oc);
  if (!o.done) return failure(o.reason, subject, o.detail);
  return confirm(a, kind, sent_ns, prior);
}

EefResult EndEffectorDevice::commandTrajectory(const eef::ActuatorSpec& a, const std::string& target, int64_t timeout_ms)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  const auto& client = trajectory_.at(a.id);
  const std::string subject = eef::actuatorIdentity(identity_, a.id);
  const auto context = node_->get_node_base_interface()->get_context();
  eef::ObservedCommand oc;
  oc.command_interface = eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY;
  if (!rosOk()) return interrupted(subject);
  if (!client->wait_for_action_server(milliseconds(eef::kServerWaitMs)))
  {
    if (!rosOk()) return interrupted(subject);
    oc.phase = eef::Phase::NO_SERVER;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail + " on '" + a.action + "'");
  }
  const auto publishers = statePublishers();
  if (!publishers) return interrupted(subject);
  oc.ros2_control = *publishers == 0;
  auto watch = std::make_shared<GoalWatch>();
  watch->may_cancel = eef::mayCancel(oc.command_interface, oc.ros2_control);

  // ONE point at duration_s, started now (stamp 0), the tolerances CARRIED in the goal: an
  // omitted or 0 tolerance means the server's default, and the JTC's default goal tolerance is
  // never checked (ADR-0022 D2). No velocities, accelerations or effort.
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = a.joints;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = a.targets.at(target);
  point.time_from_start = rclcpp::Duration::from_seconds(a.duration_s);
  goal.trajectory.points.push_back(point);
  for (std::size_t i = 0; i < a.joints.size(); ++i)
  {
    control_msgs::msg::JointTolerance t;
    t.name = a.joints[i];
    t.position = a.goal_tolerance[i];
    goal.goal_tolerance.push_back(t);
    if (!a.path_tolerance.empty())
    {
      control_msgs::msg::JointTolerance p;
      p.name = a.joints[i];
      p.position = a.path_tolerance[i];
      goal.path_tolerance.push_back(p);
    }
  }
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(a.goal_time_tolerance_s);

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
  const std::weak_ptr<rclcpp_action::Client<FollowJointTrajectory>> weak = client;
  options.goal_response_callback = [watch, weak](GoalHandle::SharedPtr handle) {
    if (handle && watch->abandoned.load() && watch->may_cancel && !watch->cancel_sent.exchange(true))
      if (auto c = weak.lock()) cancelGoal(*c, handle);   // false: the goal already ended
  };
  options.feedback_callback = [watch](GoalHandle::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> f) {
    std::lock_guard<std::mutex> g(watch->m);
    watch->has_feedback = true;
    watch->joints = f->joint_names;
    watch->error = f->error.positions;
  };
  const uint8_t kind = commandKind(a, target);
  const Prior prior = priorOf(a.id);
  const int64_t sent_ns = nowNs();
  auto handle_future = client->async_send_goal(goal, options);
  const Wait responded = waitReady(handle_future, eef::kGoalResponseMs, context);
  if (responded == Wait::INTERRUPTED) return interrupted(subject);   // HOLD: nothing is cancelled
  if (responded == Wait::TIMEOUT)
  {
    watch->abandoned = true;
    if (handle_future.wait_for(milliseconds(0)) == std::future_status::ready)
    {
      auto late = handle_future.get();
      if (late && watch->may_cancel && !watch->cancel_sent.exchange(true)) cancelGoal(*client, late);
    }
    oc.phase = eef::Phase::NO_RESPONSE;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail);
  }
  const auto handle = handle_future.get();
  if (!handle)
  {
    oc.phase = eef::Phase::REJECTED;
    const auto o = eef::mapOutcome(oc);
    return failure(o.reason, subject, o.detail + " (joints [" + joinComma(a.joints) + "])");
  }
  recordSent(a.id, kind, sent_ns, prior.stamp_ns);
  auto result_future = client->async_get_result(handle);
  const int64_t deadline_ms = eef::resultDeadlineMs(a, timeout_ms);
  const Wait finished = waitReady(result_future, deadline_ms, context);
  if (finished == Wait::INTERRUPTED) return interrupted(subject);
  if (finished == Wait::TIMEOUT)
  {
    // a cancel that throws means the goal reached a terminal state meanwhile (see commandGripper)
    const bool cancel_failed = watch->may_cancel && !cancelGoal(*client, handle);
    if (!cancel_failed || result_future.wait_for(milliseconds(0)) != std::future_status::ready)
    {
      oc.phase = eef::Phase::NO_RESULT;
      const auto o = eef::mapOutcome(oc);
      return failure(o.reason, subject, o.detail + " (" + std::to_string(deadline_ms) + " ms)");
    }
  }
  const auto wrapped = result_future.get();
  oc.phase = eef::Phase::RESULT;
  oc.status = statusOf(wrapped.code);
  if (wrapped.result)
  {
    oc.error_code = wrapped.result->error_code;
    oc.error_string = wrapped.result->error_string;
  }
  const auto o = eef::mapOutcome(oc);
  if (!o.done)
  {
    std::string detail = o.detail;
    if (o.reason == eef::Reason::GOAL_TOLERANCE_VIOLATED || o.reason == eef::Reason::PATH_TOLERANCE_VIOLATED)
    {
      // which joint: from the server's newest feedback (its actual-vs-desired error)
      const auto& tol = o.reason == eef::Reason::GOAL_TOLERANCE_VIOLATED ? a.goal_tolerance : a.path_tolerance;
      std::lock_guard<std::mutex> g(watch->m);
      std::vector<std::string> off;
      for (std::size_t i = 0; i < watch->joints.size() && i < watch->error.size(); ++i)
      {
        const auto j = std::find(a.joints.begin(), a.joints.end(), watch->joints[i]);
        if (j == a.joints.end()) continue;
        const double limit = tol.empty() ? 0.0 : tol[static_cast<std::size_t>(j - a.joints.begin())];
        if (std::fabs(watch->error[i]) > limit)
        {
          std::ostringstream os;
          os << watch->joints[i] << " (error " << watch->error[i] << ", tolerance " << limit << ")";
          off.push_back(os.str());
        }
      }
      detail += watch->has_feedback ? (off.empty() ? std::string("; the last feedback shows no joint out of tolerance")
                                                   : "; out of tolerance in the last feedback: " + joinComma(off))
                                    : std::string("; no feedback was received");
    }
    return failure(o.reason, subject, detail);
  }
  return confirm(a, kind, sent_ns, prior);
}

EefResult EndEffectorDevice::confirm(const eef::ActuatorSpec& a, uint8_t kind, int64_t sent_ns, const Prior& prior)
{
  const std::string subject = eef::actuatorIdentity(identity_, a.id);
  const bool set_bool = a.command_interface == eef::CommandInterface::SET_BOOL;
  // The rule applies to every actuator the device's own adapter serves: EVERY set_bool actuator
  // (only a TrainIt adapter serves one, and it always publishes the state: ADR-0022 D2/D6), and a
  // GripperCommand or FollowJointTrajectory actuator while a state publisher is visible. With no
  // publisher, such a server is a third-party (ros2_control) controller, which has no state in
  // M2.6: the rule does not apply (the implementation notes' run-time recognition).
  if (!set_bool)
  {
    const auto publishers = statePublishers();
    if (!publishers) return interrupted(subject);
    if (*publishers == 0) return ok();
  }
  const int64_t window_ms = set_bool ? a.confirm_timeout_ms : eef::kConfirmDefaultMs;
  const int64_t tol = saturatedMs(spec_.verification.clock_tolerance_ms) * kNsPerMs;
  const auto index = static_cast<std::size_t>(
    std::find(spec_.actuator_ids.begin(), spec_.actuator_ids.end(), a.id) - spec_.actuator_ids.begin());
  const uint8_t want_binary = kind == Act::COMMAND_ENGAGE ? Act::BINARY_ENGAGED : Act::BINARY_RELEASED;
  const auto deadline = SteadyClock::now() + milliseconds(saturatedMs(window_ms));
  bool reflected = false;   // some sample in the window showed THIS command
  Snapshot snap;
  for (;;)
  {
    if (!rosOk()) return interrupted(subject);
    snap = latest();
    if (snap.sample)
    {
      const State& s = *snap.sample;
      if (s.end_effector != identity_)
        return failure(eef::Reason::WRONG_EEF, identity_,
                       "the state read for " + a.id + " identifies '" + s.end_effector + "'");
      if (index >= s.actuators.size() || s.actuators[index].actuator != a.id)
        return failure(eef::Reason::CONTRACT_MISMATCH, identity_,
                       "the state does not list " + a.id + " where the contract does");
      const Act& e = s.actuators[index];
      // THIS command: applied at or after the send (within the tolerance), and newer than the
      // entry this runner saw when it sent -- an earlier identical command is not a reflection
      const int64_t stamp = toNs(e.last_command_stamp);
      if (e.last_command == kind && stamp >= sent_ns - tol && stamp > prior.stamp_ns)
      {
        reflected = true;
        // with actuation_feedback the switch confirms next: binary == target, binary_confirmed
        if (!a.actuation_feedback) return ok();
        if (e.binary == want_binary && e.binary_confirmed && toNs(e.stamp) >= sent_ns - tol) return ok();
      }
    }
    if (SteadyClock::now() >= deadline) break;
    std::this_thread::sleep_for(kSlice);
  }
  const std::string what = std::string(eef::commandToken(kind)) + " sent to " + a.id;
  const std::string within = " within " + std::to_string(window_ms) + " ms";
  if (!reflected)
  {
    // a silent adapter is told apart from a command that went elsewhere
    if (!snap.sample)
      return failure(eef::Reason::STATE_MISSING, identity_,
                     "no EndEffectorState sample arrived on '" + state_topic_ + "'" + within + " of the " + what);
    if (snap.seq == prior.seq)
      return failure(eef::Reason::STATE_STALE, subject,
                     "the state publisher sent nothing since the command (the " + what + ";" + within + ")");
    return failure(eef::Reason::CONTRACT_MISMATCH, subject,
                   "the device's own state never showed the " + what + within +
                     ": the command reached another device (a remapped client?)");
  }
  return failure(eef::Reason::ACTUATION_NOT_CONFIRMED, subject,
                 std::string("no switch on ") + a.id + " confirmed " + eef::binaryToken(want_binary) + within +
                   " of the command");
}

EefResult EndEffectorDevice::verify(const std::string& site, eef::Expect expect, int64_t timeout_ms,
                                    int64_t max_age_ms, const std::vector<eef::EvidenceClass>& accepted)
{
  const std::string where = site.empty() ? identity_ : eef::siteIdentity(identity_, site);
  eef::JudgeInput in;
  in.device = &spec_;
  in.identity = identity_;
  in.site = site;
  in.expect = expect;
  in.max_age_ms = max_age_ms;
  in.clock_tolerance_ms = spec_.verification.clock_tolerance_ms;
  in.accepted = accepted;
  in.sent = sent();
  const auto publishers = statePublishers();   // one writer: at every VerifyGrasp
  if (!publishers) return interrupted(where);
  in.publishers = *publishers;
  const auto deadline = SteadyClock::now() + milliseconds(saturatedMs(timeout_ms));
  for (;;)
  {
    if (!rosOk()) return interrupted(where);
    const Snapshot snap = latest();
    in.sample = snap.sample.get();
    in.at_deadline = SteadyClock::now() >= deadline;
    in.now_ns = nowNs();
    const eef::JudgeResult j = eef::judge(in);
    if (j.verdict != eef::Verdict::WAIT || in.at_deadline)
    {
      EefResult r;
      r.ok = j.verdict == eef::Verdict::SUCCESS;
      r.reason = j.verdict == eef::Verdict::WAIT ? eef::Reason::STATE_STALE : j.reason;
      r.subject = j.subject;
      r.detail = j.detail;
      r.inferred = j.inferred;
      return r;
    }
    std::this_thread::sleep_for(kSlice);
  }
}

// =================================================================================================
// EndEffectorRegistry
// =================================================================================================
EndEffectorRegistry::EndEffectorRegistry(const rclcpp::Node::SharedPtr& node, eef::Contract contract)
: node_(node), contract_(std::move(contract))
{
  for (const auto& d : contract_.devices) devices_.push_back(std::make_unique<EndEffectorDevice>(node_, d));
}

EndEffectorDevice* EndEffectorRegistry::device(const std::string& id) const
{
  for (const auto& d : devices_)
    if (d->spec().id == id) return d.get();
  return nullptr;
}

EndEffectorDevice* EndEffectorRegistry::resolve(const std::string& eef, EefResult& out) const
{
  if (eef.empty())
  {
    if (devices_.size() == 1) return devices_.front().get();
    out.reason = eef::Reason::UNKNOWN_EEF;
    out.subject = rclcpp::expand_topic_or_service_name("eef", node_->get_name(), node_->get_namespace(), false);
    out.detail = "the contract has " + std::to_string(devices_.size()) + " devices [" + joinComma(contract_.ids()) +
                 "]: name one with the eef port";
    return nullptr;
  }
  if (EndEffectorDevice* d = device(eef)) return d;
  out.reason = eef::Reason::UNKNOWN_EEF;
  out.subject = std::string(node_->get_namespace()) + (std::string(node_->get_namespace()) == "/" ? "" : std::string(1, kSep)) +
                rel({"eef", eef});
  out.detail = "no device '" + eef + "' in the contract [" + joinComma(contract_.ids()) + "]";
  return nullptr;
}

EefResult EndEffectorRegistry::finish(const std::string& who, const std::string& what, EefResult r)
{
  const auto log = node_->get_logger();
  if (r.ok)
  {
    // the judge's own words on a SUCCESS, when it has any ("held through <site>")
    RCLCPP_INFO(log, "%s %s: SUCCESS%s%s%s", who.c_str(), what.c_str(), r.inferred ? " (confirmed by inference)" : "",
                r.detail.empty() ? "" : ", ", r.detail.c_str());
    return r;
  }
  const char* reason = eef::toToken(r.reason);
  const char* cls = eef::toToken(eef::classOf(r.reason));
  if (eef::isFault(r.reason))
  {
    RCLCPP_ERROR(log, "%s %s: %s (%s) %s: %s", who.c_str(), what.c_str(), reason, cls, r.subject.c_str(), r.detail.c_str());
    latch(r.reason, r.subject, who + ": " + r.detail);
  }
  else
  {
    RCLCPP_WARN(log, "%s %s: %s (%s) %s: %s", who.c_str(), what.c_str(), reason, cls, r.subject.c_str(), r.detail.c_str());
  }
  return r;
}

EefResult EndEffectorRegistry::grip(const std::string& who, const std::string& eef, const std::string& site, bool engage)
{
  EefResult r;
  EndEffectorDevice* d = resolve(eef, r);
  if (d == nullptr) return finish(who, eef, r);
  const eef::DeviceSpec& spec = d->spec();
  const eef::SiteSpec* s = nullptr;
  if (site.empty())
  {
    if (spec.sites.size() != 1)
    {
      r.reason = eef::Reason::UNKNOWN_SITE;
      r.subject = d->identity();
      r.detail = spec.id + " has " + std::to_string(spec.sites.size()) + " sites: name one with the site port";
      return finish(who, d->identity(), r);
    }
    s = &spec.sites.front();
  }
  else if ((s = spec.site(site)) == nullptr)
  {
    r.reason = eef::Reason::UNKNOWN_SITE;
    r.subject = eef::siteIdentity(d->identity(), site);
    r.detail = "no site '" + site + "' in " + spec.id;
    return finish(who, r.subject, r);
  }
  const std::string& target = engage ? s->engage : s->release;
  const std::string what = eef::siteIdentity(d->identity(), s->id) + " (" + s->grip + " -> " + target + ")";
  r = d->command(s->grip, target, std::nullopt);
  if (r.ok && d->tmrPublishesSceneSignal()) d->publishSceneSignal(engage);   // on DONE only (ADR-0012 D3)
  return finish(who, what, r);
}

EefResult EndEffectorRegistry::actuate(const std::string& who, const std::string& eef, const std::string& actuator,
                                       const std::string& target, std::optional<int64_t> timeout_ms)
{
  EefResult r;
  EndEffectorDevice* d = resolve(eef, r);
  if (d == nullptr) return finish(who, eef, r);
  const eef::DeviceSpec& spec = d->spec();
  const std::string subject = eef::actuatorIdentity(d->identity(), actuator);
  const std::string what = subject + " -> " + target;
  const eef::ActuatorSpec* a = spec.actuator(actuator);
  if (a == nullptr)
  {
    r.reason = eef::Reason::UNKNOWN_ACTUATOR;
    r.subject = subject;
    r.detail = "no actuator '" + actuator + "' in " + spec.id;
    return finish(who, what, r);
  }
  const eef::SiteSpec* gripped = spec.siteGrippedBy(actuator);
  if (!a->hasTarget(target) || (gripped && (target == gripped->engage || target == gripped->release)))
  {
    r.reason = eef::Reason::UNKNOWN_TARGET;
    r.subject = subject;
    r.detail = !a->hasTarget(target) ? "'" + target + "' is not a named target of " + actuator
                                     : "'" + target + "' is the grip target of site " + gripped->id +
                                         ": use CloseGripper / OpenGripper";
    return finish(who, what, r);
  }
  if (timeout_ms && *timeout_ms <= 0)
  {
    r.reason = eef::Reason::CONTRACT_MISMATCH;
    r.subject = subject;
    r.detail = "timeout_ms must be > 0";
    return finish(who, what, r);
  }
  return finish(who, what, d->command(actuator, target, timeout_ms));
}

EefResult EndEffectorRegistry::verifyGrasp(const std::string& who, const std::string& eef, const std::string& site,
                                           const std::string& expect, std::optional<int64_t> timeout_ms,
                                           std::optional<int64_t> max_age_ms, const std::string& accept)
{
  EefResult r;
  EndEffectorDevice* d = resolve(eef, r);
  if (d == nullptr) return finish(who, eef, r);
  const eef::DeviceSpec& spec = d->spec();
  const std::string where = site.empty() ? d->identity() : eef::siteIdentity(d->identity(), site);
  const std::string what = where + " " + (expect.empty() ? std::string("?") : expect);
  auto bad = [&](eef::Reason reason, const std::string& subject, const std::string& detail) {
    r.reason = reason;
    r.subject = subject;
    r.detail = detail;
    return finish(who, what, r);
  };
  if (!site.empty() && spec.site(site) == nullptr) return bad(eef::Reason::UNKNOWN_SITE, where, "no site '" + site + "' in " + spec.id);
  if (expect != "held" && expect != "released")
    return bad(eef::Reason::CONTRACT_MISMATCH, where, "expect='" + expect + "': held | released");
  std::vector<eef::EvidenceClass> accepted = spec.verification.accepted_evidence;
  if (!accept.empty())
  {
    accepted.clear();
    for (const auto& token : splitList(accept))
    {
      const auto e = eef::acceptedEvidenceFromToken(token);
      if (!e || !spec.verification.accepts(*e))
        return bad(eef::Reason::CONTRACT_MISMATCH, where,
                   "accept='" + accept + "': '" + token + "' is not in the project's accepted_evidence (accept may only narrow it)");
      if (std::find(accepted.begin(), accepted.end(), *e) == accepted.end()) accepted.push_back(*e);
    }
    if (accepted.empty()) return bad(eef::Reason::CONTRACT_MISMATCH, where, "accept='" + accept + "' names no evidence class");
  }
  const int64_t timeout = timeout_ms.value_or(spec.verification.timeout_ms);
  const int64_t max_age = max_age_ms.value_or(spec.verification.max_age_ms);
  if (timeout <= 0 || max_age <= 0) return bad(eef::Reason::CONTRACT_MISMATCH, where, "timeout_ms and max_age_ms must be > 0");
  return finish(who, what, d->verify(site, expect == "held" ? eef::Expect::HELD : eef::Expect::RELEASED, timeout, max_age, accepted));
}

EefResult EndEffectorRegistry::portError(const std::string& who, const std::string& path, const std::string& detail)
{
  EefResult r;
  r.reason = eef::Reason::CONTRACT_MISMATCH;
  r.subject = path;
  r.detail = detail;
  return finish(who, path, r);
}

// --- the start-up checks ----------------------------------------------------------------------------
std::vector<std::string> EndEffectorRegistry::checkTree(const BT::Tree& tree) const
{
  std::vector<std::string> problems;
  enum class Kind { ABSENT, LITERAL, BLACKBOARD };
  struct Port
  {
    Kind kind{Kind::ABSENT};
    std::string value{};
  };
  const std::string ids = "[" + joinComma(contract_.ids()) + "]";
  tree.applyVisitor([&](const BT::TreeNode* n) {
    const std::string& id = n->registrationName();
    const std::string where = n->fullPath() + " (" + id + ")";
    if (id == "SetGripper")
    {
      problems.push_back(where + ": SetGripper is not part of the canonical end-effector contract; use "
                                 "CloseGripper/OpenGripper or CommandActuator");
      return;
    }
    const bool grip = id == "CloseGripper" || id == "OpenGripper";
    const bool act = id == "CommandActuator";
    const bool ver = id == "VerifyGrasp";
    if (!grip && !act && !ver) return;
    auto port = [n](const char* name) {
      Port p;
      const auto& ports = n->config().input_ports;
      const auto it = ports.find(name);
      if (it == ports.end() || it->second.empty()) return p;
      p.value = it->second;
      p.kind = BT::TreeNode::isBlackboardPointer(it->second) ? Kind::BLACKBOARD : Kind::LITERAL;
      return p;
    };
    auto bad = [&](const char* name, const std::string& value, const std::string& why) {
      problems.push_back(where + ": " + name + "=\"" + value + "\": " + why);
    };
    auto positiveInt = [&](const char* name) {
      const Port p = port(name);
      if (p.kind == Kind::LITERAL && !isPositiveIntText(p.value)) bad(name, p.value, "a positive integer");
    };

    const Port e = port("eef");
    const eef::DeviceSpec* dev = nullptr;
    if (e.kind == Kind::LITERAL && (dev = contract_.device(e.value)) == nullptr)
      return bad("eef", e.value, "no such device in the contract " + ids);
    if (e.kind == Kind::ABSENT)
    {
      if (contract_.devices.size() != 1)
      {
        problems.push_back(where + ": no eef port, and the contract has several devices " + ids);
        return;
      }
      dev = &contract_.devices.front();
    }
    const Port s = port("site");
    if (dev && s.kind == Kind::LITERAL && dev->site(s.value) == nullptr) bad("site", s.value, "no such site of " + dev->id);
    if (dev && grip && s.kind == Kind::ABSENT && dev->sites.size() != 1)
      problems.push_back(where + ": " + dev->id + " has several sites: name one with the site port");
    if (act)
    {
      const Port a = port("actuator");
      const Port t = port("target");
      if (a.kind == Kind::ABSENT) problems.push_back(where + ": the actuator port is required");
      if (t.kind == Kind::ABSENT) problems.push_back(where + ": the target port is required");
      const eef::ActuatorSpec* spec = (dev && a.kind == Kind::LITERAL) ? dev->actuator(a.value) : nullptr;
      if (dev && a.kind == Kind::LITERAL && spec == nullptr) bad("actuator", a.value, "no such actuator of " + dev->id);
      if (spec && t.kind == Kind::LITERAL)
      {
        const eef::SiteSpec* gripped = dev->siteGrippedBy(spec->id);
        if (!spec->hasTarget(t.value)) bad("target", t.value, "not a named target of " + spec->id);
        else if (gripped && (t.value == gripped->engage || t.value == gripped->release))
          bad("target", t.value, "the grip target of site " + gripped->id + ": use CloseGripper/OpenGripper");
      }
      positiveInt("timeout_ms");
    }
    if (ver)
    {
      const Port x = port("expect");
      if (x.kind == Kind::ABSENT) problems.push_back(where + ": the expect port is required (held | released)");
      if (x.kind == Kind::LITERAL && x.value != "held" && x.value != "released") bad("expect", x.value, "held | released");
      const Port acc = port("accept");
      if (dev && acc.kind == Kind::LITERAL)
      {
        const auto tokens = splitList(acc.value);
        if (tokens.empty()) bad("accept", acc.value, "names no evidence class");
        for (const auto& token : tokens)
        {
          const auto ev = eef::acceptedEvidenceFromToken(token);
          if (!ev || !dev->verification.accepts(*ev))
            bad("accept", acc.value, "'" + token + "' is not in " + dev->id + "'s accepted_evidence (accept may only narrow it)");
        }
      }
      positiveInt("timeout_ms");
      positiveInt("max_age_ms");
    }
  });
  return problems;
}

std::vector<std::string> EndEffectorRegistry::checkStateWriters() const
{
  std::vector<std::string> problems;
  for (const auto& d : devices_)
  {
    const auto n = d->statePublishers();   // nullopt: the context shut down, and no run follows
    if (n && *n > 1)
      problems.push_back(d->identity() + ": " + std::to_string(*n) + " publishers on '" + d->stateTopic() +
                         "'; ONE adapter writes a device's state");
  }
  return problems;
}

// --- the fault latch ------------------------------------------------------------------------------
void EndEffectorRegistry::latch(eef::Reason reason, const std::string& subject, const std::string& detail)
{
  {
    std::lock_guard<std::mutex> g(latch_m_);
    latched_.push_back(std::string(eef::toToken(reason)) + " " + subject + ": " + detail);
  }
  RCLCPP_ERROR(node_->get_logger(), "end-effector FAULT latched: %s %s (the run ends with exit code 3 if the tree "
               "still succeeds, ADR-0022)", eef::toToken(reason), subject.c_str());
}

bool EndEffectorRegistry::faultLatched() const
{
  std::lock_guard<std::mutex> g(latch_m_);
  return !latched_.empty();
}

std::vector<std::string> EndEffectorRegistry::latchedFaults() const
{
  std::lock_guard<std::mutex> g(latch_m_);
  return latched_;
}

std::string contractLine(const eef::Contract& contract)
{
  if (!contract.canonical()) return "end-effector contract: LEGACY (ADR-0022)";
  return "end-effector contract: canonical, devices [" + joinComma(contract.ids()) + "] (ADR-0022)";
}

std::vector<EefEndpoint> eefEndpoints(const eef::Contract& contract)
{
  std::vector<EefEndpoint> out;
  for (const auto& d : contract.devices)
  {
    out.push_back({EefEndpoint::Kind::TOPIC, rel({"eef", d.id, "state"})});
    if (d.scene_signal == eef::SceneSignal::TMR) out.push_back({EefEndpoint::Kind::TOPIC, rel({"eef", d.id, "engaged"})});
    for (const auto& a : d.actuators)
    {
      switch (a.command_interface)
      {
        case eef::CommandInterface::SET_BOOL:
          out.push_back({EefEndpoint::Kind::SERVICE, rel({"eef", d.id, "actuators", a.id, "set_engaged"})});
          break;
        case eef::CommandInterface::GRIPPER_COMMAND:
        case eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY:
          out.push_back({EefEndpoint::Kind::ACTION, a.action});
          break;
        case eef::CommandInterface::UNKNOWN:
          break;
      }
    }
  }
  return out;
}

}  // namespace trainit
