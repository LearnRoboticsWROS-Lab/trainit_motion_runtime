// vNext M2.6 AFTER test harness -- in-process fakes of what a CANONICAL end-effector device talks
// to (ADR-0022), for the runner's registry (trainit::EndEffectorRegistry):
//
//   ScriptedSetBool           a std_srvs/SetBool server whose reply to EACH call is scripted: APPLY
//                             (success:true), REJECT (success:false + message), NO_ANSWER (never
//                             answers, not applied), APPLIED_UNANSWERED (applied, never answers)
//   ScriptedTrajectoryServer  a control_msgs/FollowJointTrajectory server with the visible JTC rules
//                             (the EXACT joint set or REJECT), scripted SUCCEED / ABORT(code, text) /
//                             REJECT / WITHHOLD, a motion of run_ms, an optional feedback, CANCELED
//                             on a cancel request; it records every goal whole (tolerances included)
//   FakeEefAdapter            ONE device's TrainIt adapter, minimally: it serves every actuator on its
//                             standard interface (the BEFORE harness's ScriptedGripper for a
//                             GripperCommand actuator), and publishes eef/<id>/state by the adapter's
//                             state rules -- last_command from ANY commander (APPLIED commands only;
//                             a rejected one goes to `detail`), per-site PENDING -> decided after an
//                             applied engage, the LOST latch (HELD -> not held with no applied release
//                             since; cleared by the next applied engage or release), the status
//                             precedence FAULT > UNAVAILABLE > UNKNOWN > BUSY > READY, stamps that are
//                             observations (a silent source stops advancing) and never re-stamped; a
//                             site with no deciding source is UNAVAILABLE / NONE stamped 0, as the
//                             real adapters publish it (it was never observed).
//
// Every server answers from a MutuallyExclusive group on a dedicated executor (the BEFORE harness's
// rclcpp_action facts hold: a cancel needs a real cancel request; a withheld goal is finished by the
// harness). Synthetic names only.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_FAKES_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_FAKES_HPP_

#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trainit_interfaces/msg/end_effector_actuator_state.hpp>
#include <trainit_interfaces/msg/end_effector_state.hpp>

#include "m2_6/capture26.hpp"
#include "m2_6/eef_fakes.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"

namespace m26
{
namespace after
{

using State = trainit_interfaces::msg::EndEffectorState;
using Act = trainit_interfaces::msg::EndEffectorActuatorState;
using SetBool = std_srvs::srv::SetBool;
using Fjt = control_msgs::action::FollowJointTrajectory;

inline builtin_interfaces::msg::Time toTime(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

inline int64_t toNs(const builtin_interfaces::msg::Time& t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

// ===================================================================================================
// ScriptedSetBool
// ===================================================================================================
struct SetBoolReply
{
  enum class Kind { APPLY, REJECT, NO_ANSWER, APPLIED_UNANSWERED };
  Kind kind{Kind::APPLY};
  std::string message{};

  static SetBoolReply apply() { return SetBoolReply{}; }
  static SetBoolReply reject(std::string why)
  {
    SetBoolReply r;
    r.kind = Kind::REJECT;
    r.message = std::move(why);
    return r;
  }
  static SetBoolReply noAnswer()
  {
    SetBoolReply r;
    r.kind = Kind::NO_ANSWER;
    return r;
  }
  static SetBoolReply appliedUnanswered()
  {
    SetBoolReply r;
    r.kind = Kind::APPLIED_UNANSWERED;
    return r;
  }
};

struct SetBoolCall
{
  int ordinal{0};
  bool data{false};
  SetBoolReply reply{};
};

class ScriptedSetBool
{
public:
  // called inside the service callback: (data, applied, rejection message)
  using Hook = std::function<void(bool, bool, const std::string&)>;

  ScriptedSetBool(Log& log, const std::string& ns, const std::string& service, Hook hook)
  : log_(log), hook_(std::move(hook))
  {
    node_ = std::make_shared<rclcpp::Node>("m26_setbool" + tagOf(ns + "/" + service), ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = node_->create_service<SetBool>(
      service,
      [this](std::shared_ptr<rclcpp::Service<SetBool>> srv, std::shared_ptr<rmw_request_id_t> header,
             std::shared_ptr<SetBool::Request> req) { onCall(srv, header, *req); },
      rmw_qos_profile_services_default, group_);
    name_ = service_->get_service_name();
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~ScriptedSetBool()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  ScriptedSetBool(const ScriptedSetBool&) = delete;
  ScriptedSetBool& operator=(const ScriptedSetBool&) = delete;

  const std::string& name() const { return name_; }
  // the replies to the next calls; after them every call is APPLY
  void script(std::vector<SetBoolReply> replies)
  {
    std::lock_guard<std::mutex> g(m_);
    script_.assign(replies.begin(), replies.end());
  }
  std::vector<SetBoolCall> calls() const
  {
    std::lock_guard<std::mutex> g(m_);
    return calls_;
  }

private:
  void onCall(const std::shared_ptr<rclcpp::Service<SetBool>>& srv, const std::shared_ptr<rmw_request_id_t>& header,
              const SetBool::Request& req)
  {
    SetBoolReply r;
    int k = 0;
    {
      std::lock_guard<std::mutex> g(m_);
      if (!script_.empty())
      {
        r = script_.front();
        script_.pop_front();
      }
      k = static_cast<int>(calls_.size()) + 1;
      calls_.push_back({k, req.data, r});
    }
    const bool applied = r.kind == SetBoolReply::Kind::APPLY || r.kind == SetBoolReply::Kind::APPLIED_UNANSWERED;
    log_.event("SERVICE", name_, "SetBool.call #" + std::to_string(k) + " data=" + boolText(req.data) +
                                   (applied ? " APPLIED" : " not applied"));
    if (hook_) hook_(req.data, applied, r.kind == SetBoolReply::Kind::REJECT ? r.message : std::string());
    if (r.kind == SetBoolReply::Kind::NO_ANSWER || r.kind == SetBoolReply::Kind::APPLIED_UNANSWERED) return;
    SetBool::Response res;
    res.success = r.kind == SetBoolReply::Kind::APPLY;
    res.message = r.kind == SetBoolReply::Kind::REJECT ? r.message : std::string("applied");
    srv->send_response(*header, res);
  }

  Log& log_;
  Hook hook_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<SetBool>::SharedPtr service_;
  std::string name_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  mutable std::mutex m_;
  std::deque<SetBoolReply> script_;
  std::vector<SetBoolCall> calls_;
};

// ===================================================================================================
// ScriptedTrajectoryServer
// ===================================================================================================
struct TrajectoryReply
{
  enum class Kind { SUCCEED, ABORT, REJECT, WITHHOLD };
  Kind kind{Kind::SUCCEED};
  int32_t error_code{0};
  std::string error_string{};
  int run_ms{30};                        // the "motion" before the result
  std::vector<double> feedback_error{};  // non-empty: ONE feedback with this error before the result

  static TrajectoryReply succeed(int run_ms = 30)
  {
    TrajectoryReply r;
    r.run_ms = run_ms;
    return r;
  }
  static TrajectoryReply abort(int32_t code, std::string text, int run_ms = 30)
  {
    TrajectoryReply r;
    r.kind = Kind::ABORT;
    r.error_code = code;
    r.error_string = std::move(text);
    r.run_ms = run_ms;
    return r;
  }
  static TrajectoryReply reject()
  {
    TrajectoryReply r;
    r.kind = Kind::REJECT;
    return r;
  }
  static TrajectoryReply withhold()
  {
    TrajectoryReply r;
    r.kind = Kind::WITHHOLD;
    return r;
  }
  TrajectoryReply withFeedback(std::vector<double> error) const
  {
    TrajectoryReply r = *this;
    r.feedback_error = std::move(error);
    return r;
  }
};

struct TrajectoryGoal
{
  int ordinal{0};
  std::vector<std::string> joints{};
  std::vector<double> positions{};
  std::size_t points{0};
  std::size_t velocities{0}, accelerations{0}, effort{0};
  double time_from_start_s{0.0};
  bool start_now{false};   // header.stamp == 0
  std::vector<std::pair<std::string, double>> goal_tolerance{};
  std::vector<std::pair<std::string, double>> path_tolerance{};
  double goal_time_tolerance_s{0.0};
  std::string response{};   // "ACCEPT" | "REJECT"
  std::string terminal{};   // "SUCCEEDED" | "ABORTED" | "CANCELED"
  int cancel_requests{0};
};

class ScriptedTrajectoryServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Fjt>;
  // (the goal, true when accepted / false when it ended)
  using Hook = std::function<void(const TrajectoryGoal&, bool)>;

  ScriptedTrajectoryServer(Log& log, const std::string& ns, const std::string& action,
                           std::vector<std::string> controller_joints, Hook hook)
  : log_(log), joints_(std::move(controller_joints)), hook_(std::move(hook))
  {
    node_ = std::make_shared<rclcpp::Node>("m26_fjt" + tagOf(ns + "/" + action), ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    name_ = resolved(node_, action, false);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_ = rclcpp_action::create_server<Fjt>(
      node_, action,
      [this](const rclcpp_action::GoalUUID& id, std::shared_ptr<const Fjt::Goal> goal) { return onGoal(id, *goal); },
      [this](std::shared_ptr<Handle> h) { return onCancel(h); },
      [this](std::shared_ptr<Handle> h) { onAccepted(h); }, rcl_action_server_get_default_options(), group_);
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~ScriptedTrajectoryServer()
  {
    joinWorkers();
    releaseWithheld();
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  ScriptedTrajectoryServer(const ScriptedTrajectoryServer&) = delete;
  ScriptedTrajectoryServer& operator=(const ScriptedTrajectoryServer&) = delete;

  const std::string& name() const { return name_; }
  void script(std::vector<TrajectoryReply> replies)
  {
    std::lock_guard<std::mutex> g(m_);
    script_.assign(replies.begin(), replies.end());
  }
  std::vector<TrajectoryGoal> goals() const
  {
    std::lock_guard<std::mutex> g(m_);
    return goals_;
  }
  void joinWorkers()
  {
    std::vector<std::future<void>> w;
    {
      std::lock_guard<std::mutex> g(m_);
      w.swap(workers_);
    }
    for (auto& f : w) f.wait();
  }
  void releaseWithheld()
  {
    std::map<int, std::shared_ptr<Handle>> held;
    {
      std::lock_guard<std::mutex> g(m_);
      held.swap(held_);
    }
    for (auto& [k, h] : held)
    {
      if (!h->is_active()) continue;
      auto res = std::make_shared<Fjt::Result>();
      finish(k, h->is_canceling() ? "CANCELED" : "ABORTED");
      if (h->is_canceling()) h->canceled(res);
      else h->abort(res);
    }
  }

private:
  rclcpp_action::GoalResponse onGoal(const rclcpp_action::GoalUUID& id, const Fjt::Goal& goal)
  {
    TrajectoryReply r;
    TrajectoryGoal rec;
    {
      std::lock_guard<std::mutex> g(m_);
      if (!script_.empty())
      {
        r = script_.front();
        script_.pop_front();
      }
      rec.ordinal = static_cast<int>(goals_.size()) + 1;
    }
    rec.joints = goal.trajectory.joint_names;
    rec.points = goal.trajectory.points.size();
    if (!goal.trajectory.points.empty())
    {
      const auto& p = goal.trajectory.points.back();
      rec.positions = p.positions;
      rec.velocities = p.velocities.size();
      rec.accelerations = p.accelerations.size();
      rec.effort = p.effort.size();
      rec.time_from_start_s = rclcpp::Duration(p.time_from_start).seconds();
    }
    rec.start_now = goal.trajectory.header.stamp.sec == 0 && goal.trajectory.header.stamp.nanosec == 0;
    for (const auto& t : goal.goal_tolerance) rec.goal_tolerance.emplace_back(t.name, t.position);
    for (const auto& t : goal.path_tolerance) rec.path_tolerance.emplace_back(t.name, t.position);
    rec.goal_time_tolerance_s = rclcpp::Duration(goal.goal_time_tolerance).seconds();
    // the JTC: a goal must name exactly the controller's joints (allow_partial_joints_goal false)
    const bool exact = rec.joints == joints_;
    const bool reject = !exact || r.kind == TrajectoryReply::Kind::REJECT;
    rec.response = reject ? "REJECT" : "ACCEPT";
    log_.event("ACTION", name_, "FollowJointTrajectory.goal #" + std::to_string(rec.ordinal) + " -> " +
                                  (reject ? (exact ? "REJECT (script)" : "REJECT (not the controller's joint set)") : "ACCEPT"));
    {
      std::lock_guard<std::mutex> g(m_);
      goals_.push_back(rec);
      ordinals_[id] = rec.ordinal;
      replies_[rec.ordinal] = r;
    }
    return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<Handle>& h)
  {
    std::lock_guard<std::mutex> g(m_);
    const int k = ordinals_.at(h->get_goal_id());
    ++goals_[static_cast<std::size_t>(k - 1)].cancel_requests;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void finish(int k, const std::string& terminal)
  {
    TrajectoryGoal rec;
    {
      std::lock_guard<std::mutex> g(m_);
      goals_[static_cast<std::size_t>(k - 1)].terminal = terminal;
      rec = goals_[static_cast<std::size_t>(k - 1)];
    }
    log_.event("ACTION", name_, "goal #" + std::to_string(k) + " result " + terminal);
    if (hook_) hook_(rec, false);
  }

  void onAccepted(const std::shared_ptr<Handle>& h)
  {
    int k = 0;
    TrajectoryReply r;
    TrajectoryGoal rec;
    {
      std::lock_guard<std::mutex> g(m_);
      k = ordinals_.at(h->get_goal_id());
      r = replies_.at(k);
      rec = goals_[static_cast<std::size_t>(k - 1)];
    }
    if (hook_) hook_(rec, true);
    if (r.kind == TrajectoryReply::Kind::WITHHOLD)
    {
      std::lock_guard<std::mutex> g(m_);
      held_[k] = h;
      return;
    }
    std::lock_guard<std::mutex> g(m_);
    workers_.push_back(std::async(std::launch::async, [this, h, k, r, rec] {
      const auto until = Clock::now() + milliseconds(r.run_ms);
      while (Clock::now() < until && !h->is_canceling()) std::this_thread::sleep_for(milliseconds(5));
      auto res = std::make_shared<Fjt::Result>();
      if (h->is_canceling())
      {
        res->error_code = 0;
        finish(k, "CANCELED");
        h->canceled(res);
        return;
      }
      if (!r.feedback_error.empty())
      {
        auto fb = std::make_shared<Fjt::Feedback>();
        fb->joint_names = rec.joints;
        fb->error.positions = r.feedback_error;
        fb->actual.positions = rec.positions;
        h->publish_feedback(fb);
        std::this_thread::sleep_for(milliseconds(30));   // the feedback travels before the result
      }
      res->error_code = r.kind == TrajectoryReply::Kind::SUCCEED ? 0 : r.error_code;
      res->error_string = r.error_string;
      finish(k, r.kind == TrajectoryReply::Kind::SUCCEED ? "SUCCEEDED" : "ABORTED");
      if (r.kind == TrajectoryReply::Kind::SUCCEED) h->succeed(res);
      else h->abort(res);
    }));
  }

  Log& log_;
  std::vector<std::string> joints_;
  Hook hook_;
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<rclcpp_action::Server<Fjt>> server_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  mutable std::mutex m_;
  std::deque<TrajectoryReply> script_;
  std::vector<TrajectoryGoal> goals_;
  std::map<rclcpp_action::GoalUUID, int> ordinals_;
  std::map<int, TrajectoryReply> replies_;
  std::map<int, std::shared_ptr<Handle>> held_;
  std::vector<std::future<void>> workers_;
};

// ===================================================================================================
// FakeEefAdapter
// ===================================================================================================
struct SiteScript
{
  uint8_t on_engage{State::GRASP_HELD};       // what the deciding source decides after an applied engage
  uint8_t on_release{State::GRASP_NOT_HELD};  // ... after an applied release
  uint8_t evidence{State::EVIDENCE_SENSOR};   // the deciding source's class
  int decide_after_ms{40};                    // PENDING this long after an applied engage
  bool observable{true};                      // false: no source on this backend (UNAVAILABLE / NONE)
};

struct AdapterOptions
{
  bool publish_state{true};                    // false: the servers answer, no state is published
  bool serve{true};                            // false: no command server at all
  std::string identity{};                      // non-empty: published as end_effector (a wrong identity)
  std::vector<std::string> published_actuators{};   // non-empty: published actuator ids (a mismatch)
  std::string node_name{"m26_fake_eef_adapter"};
  std::string adapter_row{"m26_fake"};
};

class FakeEefAdapter
{
public:
  FakeEefAdapter(Log& log, const std::string& ns, const trainit::eef::DeviceSpec& spec, AdapterOptions opt = {})
  : log_(log), ns_(ns), spec_(spec), opt_(std::move(opt))
  {
    node_ = std::make_shared<rclcpp::Node>(opt_.node_name, ns, rclcpp::NodeOptions().use_global_arguments(false));
    const std::string id_rel = "eef/" + spec_.id;
    identity_ = opt_.identity.empty()
                  ? rclcpp::expand_topic_or_service_name(id_rel, node_->get_name(), node_->get_namespace(), false)
                  : opt_.identity;
    const int64_t now = nowNs();
    for (const auto& a : spec_.actuators)
    {
      ActModel m;
      m.binary = a.command_interface == trainit::eef::CommandInterface::SET_BOOL ? Act::BINARY_RELEASED : Act::BINARY_UNKNOWN;
      m.stamp = now;
      acts_[a.id] = m;
    }
    for (const auto& s : spec_.sites)
    {
      SiteModel m;
      m.stamp = now;
      sites_[s.id] = m;
    }
    if (opt_.serve)
      for (const auto& a : spec_.actuators) makeServer(a);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (opt_.publish_state)
      pub_ = node_->create_publisher<State>(id_rel + "/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    timer_ = node_->create_wall_timer(milliseconds(20), [this] { tick(); }, group_);
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakeEefAdapter()
  {
    // the servers first: their callbacks and workers call back into this adapter's state
    set_bool_.clear();
    trajectory_.clear();
    {
      std::lock_guard<std::mutex> g(servers_m_);
      gripper_.clear();
    }
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  FakeEefAdapter(const FakeEefAdapter&) = delete;
  FakeEefAdapter& operator=(const FakeEefAdapter&) = delete;

  const std::string& identity() const { return identity_; }
  bool serves() const { return opt_.serve; }
  std::string stateTopic() const { return pub_ ? pub_->get_topic_name() : std::string(); }
  ScriptedSetBool& setBool(const std::string& a) { return *set_bool_.at(a); }
  ScriptedTrajectoryServer& trajectory(const std::string& a) { return *trajectory_.at(a); }
  ScriptedGripper& gripper(const std::string& a)
  {
    std::lock_guard<std::mutex> g(servers_m_);
    return *gripper_.at(a);
  }
  // A fresh GripperCommand server for `a` with `script` (ScriptedGripper takes its script whole).
  void resetGripper(const std::string& a, std::vector<GripperReply> script)
  {
    const auto* spec = spec_.actuator(a);
    std::lock_guard<std::mutex> g(servers_m_);
    gripper_[a].reset();
    gripper_seen_[a] = 0;
    gripper_[a] = std::make_unique<ScriptedGripper>(log_, ns_, spec->action, std::move(script));
  }

  void setSite(const std::string& site, SiteScript s)
  {
    std::lock_guard<std::mutex> g(m_);
    sites_.at(site).script = s;
    if (!s.observable)
    {
      sites_.at(site).grasp = State::GRASP_UNAVAILABLE;
      sites_.at(site).evidence = State::EVIDENCE_NONE;
    }
  }
  // the deciding source observes `grasp` now (a test's direct stimulus)
  void observe(const std::string& site, uint8_t grasp, uint8_t evidence)
  {
    std::lock_guard<std::mutex> g(m_);
    auto& s = sites_.at(site);
    s.grasp = grasp;
    s.evidence = evidence;
    s.pending.reset();
    s.stamp = nowNs();
  }
  // the object falls: HELD -> not held with no applied release since -> LOST (latched)
  void drop(const std::string& site)
  {
    std::lock_guard<std::mutex> g(m_);
    auto& s = sites_.at(site);
    if (s.grasp == State::GRASP_HELD) s.grasp = State::GRASP_LOST;
    s.pending.reset();
    s.stamp = nowNs();
  }
  // the site's deciding source goes silent: its stamp stops advancing
  void silence(const std::string& site, bool silent)
  {
    std::lock_guard<std::mutex> g(m_);
    sites_.at(site).silent = silent;
  }
  void setActuatorStatus(const std::string& a, uint8_t status, const std::string& why = "")
  {
    std::lock_guard<std::mutex> g(m_);
    acts_.at(a).status = status;
    acts_.at(a).stamp = nowNs();
    if (!why.empty()) notes_.push_back(a + ": " + why);
  }
  void setSwitchConfirms(const std::string& a, bool confirms)
  {
    std::lock_guard<std::mutex> g(m_);
    acts_.at(a).switch_confirms = confirms;
  }
  // stop / resume publishing (the latched last sample stays for late joiners)
  void pause(bool paused) { paused_ = paused; }
  // back to the start: every site NOT_HELD (observed), every actuator IDLE, no notes, all scripts default
  void reset()
  {
    {
      std::lock_guard<std::mutex> g(m_);
      for (auto& [id, s] : sites_)
      {
        s = SiteModel{};
        s.stamp = nowNs();
      }
      for (auto& [id, a] : acts_)
      {
        a.status = Act::STATUS_IDLE;
        a.switch_confirms = true;
        a.stamp = nowNs();
      }
      notes_.clear();
    }
    for (auto& [a, s] : set_bool_) s->script({});
    for (auto& [a, s] : trajectory_) s->script({});
    paused_ = false;
  }

  State latestPublished() const
  {
    std::lock_guard<std::mutex> g(m_);
    return last_;
  }
  uint8_t lastCommand(const std::string& a) const
  {
    std::lock_guard<std::mutex> g(m_);
    return acts_.at(a).last_command;
  }
  uint8_t siteGrasp(const std::string& site) const
  {
    std::lock_guard<std::mutex> g(m_);
    return sites_.at(site).grasp;
  }

private:
  struct ActModel
  {
    uint8_t status{Act::STATUS_IDLE};
    uint8_t binary{Act::BINARY_UNKNOWN};
    bool confirmed{false};
    bool switch_confirms{true};
    int64_t stamp{0};
    uint8_t last_command{Act::COMMAND_NONE};
    int64_t last_command_stamp{0};
  };
  struct Pending
  {
    int64_t at{0};
    uint8_t to{State::GRASP_UNKNOWN};
  };
  struct SiteModel
  {
    SiteScript script{};
    uint8_t grasp{State::GRASP_NOT_HELD};
    uint8_t evidence{State::EVIDENCE_SENSOR};
    int64_t stamp{0};
    bool silent{false};
    std::optional<Pending> pending{};
  };

  int64_t nowNs() const { return node_->now().nanoseconds(); }

  void makeServer(const trainit::eef::ActuatorSpec& a)
  {
    using CI = trainit::eef::CommandInterface;
    if (a.command_interface == CI::SET_BOOL)
    {
      const std::string id = a.id;
      set_bool_[a.id] = std::make_unique<ScriptedSetBool>(
        log_, ns_, "eef/" + spec_.id + "/actuators/" + a.id + "/set_engaged",
        [this, id](bool data, bool applied, const std::string& why) {
          const uint8_t kind = data ? Act::COMMAND_ENGAGE : Act::COMMAND_RELEASE;
          if (applied) appliedCommand(id, kind, data);
          else note(id + ": " + (data ? "ENGAGE" : "RELEASE") + " rejected" + (why.empty() ? "" : ": " + why));
        });
    }
    else if (a.command_interface == CI::GRIPPER_COMMAND)
    {
      gripper_[a.id] = std::make_unique<ScriptedGripper>(log_, ns_, a.action);
      gripper_seen_[a.id] = 0;
    }
    else if (a.command_interface == CI::FOLLOW_JOINT_TRAJECTORY)
    {
      const std::string id = a.id;
      trajectory_[a.id] = std::make_unique<ScriptedTrajectoryServer>(
        log_, ns_, a.action, a.joints, [this, id](const TrajectoryGoal&, bool started) {
          std::lock_guard<std::mutex> g(m_);
          auto& m = acts_.at(id);
          const int64_t now = nowNs();
          if (started)
          {
            m.last_command = Act::COMMAND_TARGET;   // an axis never grips: always a named target
            m.last_command_stamp = now;
            if (m.status == Act::STATUS_IDLE) m.status = Act::STATUS_MOVING;
          }
          else if (m.status == Act::STATUS_MOVING)
          {
            m.status = Act::STATUS_IDLE;
          }
          m.stamp = now;
        });
    }
  }

  void note(const std::string& n)
  {
    std::lock_guard<std::mutex> g(m_);
    notes_.push_back(n);
  }

  // The adapter's state rules for an APPLIED command, from ANY commander.
  void appliedCommand(const std::string& a, uint8_t kind, std::optional<bool> data = std::nullopt)
  {
    std::lock_guard<std::mutex> g(m_);
    const int64_t now = nowNs();
    auto& m = acts_.at(a);
    m.last_command = kind;
    m.last_command_stamp = now;
    m.stamp = now;
    if (data)
    {
      m.binary = *data ? Act::BINARY_ENGAGED : Act::BINARY_RELEASED;
      const auto* spec = spec_.actuator(a);
      m.confirmed = spec->actuation_feedback && m.switch_confirms;
    }
    const auto* site = spec_.siteGrippedBy(a);
    if (site == nullptr || (kind != Act::COMMAND_ENGAGE && kind != Act::COMMAND_RELEASE)) return;
    auto& s = sites_.at(site->id);
    if (!s.script.observable) return;
    if (kind == Act::COMMAND_ENGAGE)
    {
      // an applied engage clears LOST; the deciding source has not decided yet
      s.grasp = State::GRASP_PENDING;
      s.evidence = s.script.evidence;
      s.pending = Pending{now + static_cast<int64_t>(s.script.decide_after_ms) * 1000000, s.script.on_engage};
    }
    else
    {
      // an applied release clears LOST too; the source decides after the same delay
      if (s.grasp == State::GRASP_LOST) s.grasp = State::GRASP_NOT_HELD;
      s.pending = Pending{now + static_cast<int64_t>(s.script.decide_after_ms) * 1000000, s.script.on_release};
    }
    // PENDING carries the deciding SOURCE's newest stamp: a silent source gives the apply no
    // fresh observation (ADR-0022 D3)
    if (!s.silent) s.stamp = now;
  }

  // GripperCommand goals, read from the BEFORE harness's ScriptedGripper records.
  void pollGrippers()
  {
    std::lock_guard<std::mutex> sg(servers_m_);
    for (auto& [a, server] : gripper_)
    {
      if (!server) continue;
      const auto goals = server->goals();
      std::size_t& seen = gripper_seen_[a];
      while (seen < goals.size() && !goals[seen].response.empty())
      {
        const auto& gr = goals[seen++];
        const auto* site = spec_.siteGrippedBy(a);
        const auto* spec = spec_.actuator(a);
        uint8_t kind = Act::COMMAND_TARGET;
        if (site && spec->targets.at(site->engage).front() == gr.position) kind = Act::COMMAND_ENGAGE;
        else if (site && spec->targets.at(site->release).front() == gr.position) kind = Act::COMMAND_RELEASE;
        if (gr.response == "REJECT")
          note(a + ": " + std::string(kind == Act::COMMAND_ENGAGE ? "ENGAGE" : kind == Act::COMMAND_RELEASE ? "RELEASE" : "TARGET") + " rejected");
        else
          appliedCommand(a, kind);
      }
    }
  }

  void tick()
  {
    pollGrippers();
    State s;
    {
      std::lock_guard<std::mutex> g(m_);
      const int64_t now = nowNs();
      for (auto& [id, site] : sites_)
      {
        if (!site.script.observable) continue;
        if (site.pending && now >= site.pending->at)
        {
          site.grasp = site.pending->to;
          // evidence is NONE if and only if the grasp is UNAVAILABLE or UNKNOWN
          site.evidence = (site.grasp == State::GRASP_UNKNOWN || site.grasp == State::GRASP_UNAVAILABLE)
                            ? State::EVIDENCE_NONE : site.script.evidence;
          site.pending.reset();
        }
        if (!site.silent) site.stamp = now;   // a periodic source re-observes
      }
      s = build();
      last_ = s;
    }
    if (pub_ && !paused_) pub_->publish(s);
  }

  State build() const
  {
    State s;
    s.end_effector = identity_;
    s.adapter = opt_.adapter_row;
    int64_t newest = 0;
    for (const auto& site : spec_.sites)
    {
      const auto& m = sites_.at(site.id);
      s.sites.push_back(site.id);
      s.site_grip.push_back(site.grip);
      s.site_grasp.push_back(m.script.observable ? m.grasp : State::GRASP_UNAVAILABLE);
      s.site_evidence.push_back(m.script.observable ? m.evidence : State::EVIDENCE_NONE);
      const int64_t stamp = m.script.observable ? m.stamp : 0;   // never observed
      s.site_stamp.push_back(toTime(stamp));
      newest = std::max(newest, stamp);
    }
    bool fault = false, unavailable = false, unknown = false, busy = false;
    const auto& ids = opt_.published_actuators.empty() ? spec_.actuator_ids : opt_.published_actuators;
    for (const auto& id : ids)
    {
      Act e;
      e.actuator = id;
      const auto* spec = spec_.actuator(id);
      e.command_interface = spec ? static_cast<uint8_t>(spec->command_interface) : Act::INTERFACE_SET_BOOL;
      if (spec) e.joints = spec->joints;
      const auto it = acts_.find(id);
      if (it != acts_.end())
      {
        const auto& m = it->second;
        e.status = m.status;
        e.binary = m.binary;
        e.binary_confirmed = m.confirmed;
        e.stamp = toTime(m.stamp);
        e.last_command = m.last_command;
        e.last_command_stamp = toTime(m.last_command_stamp);
        newest = std::max(newest, m.stamp);
      }
      fault = fault || e.status == Act::STATUS_FAULT;
      unavailable = unavailable || e.status == Act::STATUS_UNAVAILABLE;
      unknown = unknown || e.status == Act::STATUS_UNKNOWN;
      busy = busy || e.status == Act::STATUS_MOVING;
      s.actuators.push_back(e);
    }
    s.status = fault ? State::STATUS_FAULT : unavailable ? State::STATUS_UNAVAILABLE : unknown ? State::STATUS_UNKNOWN
               : busy ? State::STATUS_BUSY : State::STATUS_READY;
    s.header.stamp = toTime(newest);
    std::string detail;
    for (const auto& n : notes_) detail += (detail.empty() ? "" : "; ") + n;
    s.detail = detail;
    return s;
  }

  Log& log_;
  std::string ns_;
  trainit::eef::DeviceSpec spec_;
  AdapterOptions opt_;
  rclcpp::Node::SharedPtr node_;
  std::string identity_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<State>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, std::unique_ptr<ScriptedSetBool>> set_bool_;
  std::map<std::string, std::unique_ptr<ScriptedTrajectoryServer>> trajectory_;
  std::mutex servers_m_;
  std::map<std::string, std::unique_ptr<ScriptedGripper>> gripper_;
  std::map<std::string, std::size_t> gripper_seen_;
  std::atomic<bool> paused_{false};
  mutable std::mutex m_;
  std::map<std::string, ActModel> acts_;
  std::map<std::string, SiteModel> sites_;
  std::vector<std::string> notes_;
  State last_{};
  Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
};

}  // namespace after
}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_FAKES_HPP_
