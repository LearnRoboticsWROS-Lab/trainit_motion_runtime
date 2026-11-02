// vNext M2.6 test harness -- in-process fakes of everything a legacy end-effector path talks to.
//
//   ScriptedGripper   a control_msgs/GripperCommand action server whose reply to EACH goal is
//                     scripted: SUCCEED{reached_goal, stalled, position, effort}, ABORT{result or
//                     empty}, PREEMPT (CANCELED with an empty result, after a REAL cancel request
//                     sent by a second client), REJECT, WITHHOLD (accepted, no result), and any of
//                     them with a late goal response (DELAY_ACCEPT). It records every goal, the
//                     response, the terminal status and every cancel request.
//   BoolRecorder      a std_msgs/Bool subscriber, VOLATILE or LATCHED (TRANSIENT_LOCAL; created
//                     late, it is a late joiner); each sample is attributed to the node that
//                     published it (the publisher gid, matched against the graph).
//   BoolProvider      a std_msgs/Bool publisher: once, or periodic; latched or volatile.
//   FakePolicyRuntime trainit_policy_runtime's ROS contract as RunPolicy uses it: ~/run (Trigger)
//                     and ~/status (String JSON, RELIABLE + TRANSIENT_LOCAL like the real node).
//   FakeSceneServices get_planning_scene + apply_planning_scene, for scene_manager_node; it can
//                     EMULATE move_group refusing an attach diff (labelled wherever it shows).
//   RunnerKeys / makeRunnerContext  the BtContext built EXACTLY the way trainit_run_bt builds it.
//
// rclcpp_action facts the fakes respect (Humble): a server in a REENTRANT group answers
// STATUS_UNKNOWN under load, so every server here is in a MutuallyExclusive group on a dedicated
// executor; rcl_action forbids EXECUTING -> CANCELED from the server, so a CANCELED outcome needs a
// real cancel request first; a goal handle dropped while active is cancelled by its destructor, so
// a withheld goal is kept and finished by the harness. A fake takes a production node's name only
// where that name is the default being pinned (RunPolicy's /trainit_policy_runtime/run).
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_FAKES_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_FAKES_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "capture26.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace m26
{

inline std::string resolved(const rclcpp::Node::SharedPtr& node, const std::string& name, bool is_service)
{
  return rclcpp::expand_topic_or_service_name(name, node->get_name(), node->get_namespace(), is_service);
}

// A node name made from a resource name: "/r1/x/y" -> "_r1_x_y".
inline std::string tagOf(const std::string& s)
{
  std::string t;
  for (char c : s) t.push_back((std::isalnum(static_cast<unsigned char>(c)) != 0) ? c : '_');
  return t;
}

// ===================================================================================================
// ScriptedGripper
// ===================================================================================================
struct GripperReply
{
  enum class Kind { SUCCEED, ABORT, PREEMPT, REJECT, WITHHOLD };
  Kind kind{Kind::SUCCEED};
  bool reached_goal{true};
  bool stalled{false};
  enum class Position { GOAL, FIXED, SHIM } position_rule{Position::GOAL};
  double position{0.0};        // FIXED: the reported position (e.g. where the jaws stalled)
  double effort{0.0};
  bool empty_result{false};    // an all-default Result
  int accept_delay_ms{0};      // DELAY_ACCEPT: the goal response is sent this late

  static GripperReply succeed(bool reached = true, bool stalled = false)
  { GripperReply r; r.reached_goal = reached; r.stalled = stalled; return r; }
  static GripperReply abort(bool reached, bool stalled)
  { GripperReply r = succeed(reached, stalled); r.kind = Kind::ABORT; return r; }
  static GripperReply abortEmpty()
  { GripperReply r; r.kind = Kind::ABORT; r.empty_result = true; return r; }
  // the FAIRINO suction shim (fairino_gripper/gripper_suction_action_server.py): every goal
  // ABORTED, reached_goal=false, position 1.0 when the goal is >= its 0.5 threshold, else 0.0
  static GripperReply shimAbort()
  { GripperReply r = abort(false, false); r.position_rule = Position::SHIM; return r; }
  static GripperReply preempt() { GripperReply r; r.kind = Kind::PREEMPT; r.empty_result = true; return r; }
  static GripperReply reject() { GripperReply r; r.kind = Kind::REJECT; return r; }
  static GripperReply withhold() { GripperReply r; r.kind = Kind::WITHHOLD; return r; }
  GripperReply at(double p, double e) const
  { GripperReply r = *this; r.position_rule = Position::FIXED; r.position = p; r.effort = e; return r; }
  GripperReply acceptAfter(int ms) const { GripperReply r = *this; r.accept_delay_ms = ms; return r; }

  std::string describe() const
  {
    std::string s;
    auto fields = [&] {
      if (empty_result) return std::string("(empty result)");
      std::string p = position_rule == Position::GOAL ? "goal"
                      : position_rule == Position::SHIM ? "shim(1 if goal>=0.5 else 0)" : num17(position);
      return "(reached_goal=" + std::to_string(reached_goal) + ",stalled=" + std::to_string(stalled) +
             ",position=" + p + ",effort=" + num17(effort) + ")";
    };
    switch (kind)
    {
      case Kind::SUCCEED: s = "SUCCEED" + fields(); break;
      case Kind::ABORT: s = "ABORT" + fields(); break;
      case Kind::PREEMPT: s = "PREEMPT(a second client sends a real cancel request; CANCELED, empty result)"; break;
      case Kind::REJECT: s = "REJECT"; break;
      case Kind::WITHHOLD: s = "WITHHOLD(accepted; no result until the harness releases it)"; break;
    }
    if (accept_delay_ms > 0) s += " accept_after=" + std::to_string(accept_delay_ms) + "ms";
    return s;
  }
};

struct GoalRecord
{
  int ordinal{0};
  double position{0.0};
  double max_effort{0.0};
  GripperReply reply{};
  Clock::time_point received{};
  std::string response{};          // "ACCEPT_AND_EXECUTE" | "REJECT" ("" = not answered yet)
  Clock::time_point responded{};
  std::string terminal{};          // "SUCCEEDED" | "ABORTED" | "CANCELED" ("" = none)
  Clock::time_point finished{};
  std::string finished_by{};       // "script" | "a second client's cancel" | "the harness"
  control_msgs::action::GripperCommand::Result result{};
  bool result_empty{false};
  int cancel_requests{0};
};

inline std::string resultText(const control_msgs::action::GripperCommand::Result& r, bool empty)
{
  if (empty) return "(empty result)";
  return "reached_goal=" + std::to_string(r.reached_goal) + " stalled=" + std::to_string(r.stalled) +
         " position=" + num17(r.position) + " effort=" + num17(r.effort);
}

class ScriptedGripper
{
public:
  using Gripper = control_msgs::action::GripperCommand;
  using Handle = rclcpp_action::ServerGoalHandle<Gripper>;

  // `script`: the reply to goal 1, 2, ...; after it, every goal is SUCCEED(reached, echo).
  ScriptedGripper(Log& log, const std::string& ns, const std::string& action,
                  std::vector<GripperReply> script = {})
  : log_(log), script_(script.begin(), script.end())
  {
    for (const auto& r : script) script_text_ += (script_text_.empty() ? "" : ", ") + r.describe();
    if (script_text_.empty()) script_text_ = "default";
    script_text_ += " (then SUCCEED(reached_goal=1,stalled=0,position=goal,effort=0))";
    const std::string tag = tagOf(ns == "/" ? action : ns + "/" + action);
    node_ = std::make_shared<rclcpp::Node>("m26_gripper" + tag, ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    name_ = resolved(node_, action, false);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_ = rclcpp_action::create_server<Gripper>(
      node_, action,
      [this](const rclcpp_action::GoalUUID& id, std::shared_ptr<const Gripper::Goal> goal) {
        return onGoal(id, *goal);
      },
      [this](std::shared_ptr<Handle> h) { return onCancel(h); },
      [this](std::shared_ptr<Handle> h) { onAccepted(h); },
      rcl_action_server_get_default_options(), group_);
    spin_.executor().add_node(node_);
    spin_.start();
    bool preempts = false;
    for (const auto& r : script) preempts = preempts || r.kind == GripperReply::Kind::PREEMPT;
    if (preempts)
    {
      // the SECOND client, on its own node and executor (RViz Plan & Execute, another program)
      preempt_node_ = std::make_shared<rclcpp::Node>("m26_preemptor" + tag, ns,
                                                     rclcpp::NodeOptions().use_global_arguments(false));
      preempt_client_ = rclcpp_action::create_client<Gripper>(preempt_node_, action);
      preempt_name_ = preempt_node_->get_fully_qualified_name();
      preempt_spin_.executor().add_node(preempt_node_);
      preempt_spin_.start();
    }
  }

  ~ScriptedGripper()
  {
    joinWorkers();
    releaseWithheld();
    if (preempt_node_)
    {
      preempt_spin_.stop();
      preempt_spin_.executor().remove_node(preempt_node_);
    }
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  ScriptedGripper(const ScriptedGripper&) = delete;
  ScriptedGripper& operator=(const ScriptedGripper&) = delete;

  const std::string& name() const { return name_; }
  const std::string& scriptText() const { return script_text_; }

  std::vector<GoalRecord> goals() const
  {
    std::lock_guard<std::mutex> g(m_);
    return records_;
  }
  int cancelRequests() const
  {
    std::lock_guard<std::mutex> g(m_);
    int n = 0;
    for (const auto& r : records_) n += r.cancel_requests;
    return n;
  }
  // cancel requests the harness's own second client sent (PREEMPT)
  int preemptsSent() const { return preempts_sent_.load(); }

  // Finish every goal still held (WITHHOLD), recorded as the harness's doing.
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
      const bool canceling = h->is_canceling();
      const std::string status = canceling ? "CANCELED" : "ABORTED";
      log_.event("ACTION", name_, "goal #" + std::to_string(k) + " released by the harness: " + status +
                 (canceling ? " (it was CANCELING)" : ""), "(empty result)");
      {
        std::lock_guard<std::mutex> g(m_);
        auto& rec = records_[static_cast<std::size_t>(k - 1)];
        rec.terminal = status;
        rec.finished = Clock::now();
        rec.finished_by = "the harness";
        rec.result_empty = true;
      }
      auto res = std::make_shared<Gripper::Result>();
      if (canceling) h->canceled(res);
      else h->abort(res);
    }
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

private:
  rclcpp_action::GoalResponse onGoal(const rclcpp_action::GoalUUID& id, const Gripper::Goal& goal)
  {
    GripperReply r = GripperReply::succeed();
    int k = 0;
    {
      std::lock_guard<std::mutex> g(m_);
      if (!script_.empty())
      {
        r = script_.front();
        script_.pop_front();
      }
      k = static_cast<int>(records_.size()) + 1;
      GoalRecord rec;
      rec.ordinal = k;
      rec.position = goal.command.position;
      rec.max_effort = goal.command.max_effort;
      rec.reply = r;
      rec.received = Clock::now();
      records_.push_back(rec);
      ordinals_[id] = k;
    }
    const std::string ks = std::to_string(k);
    log_.event("ACTION", name_, "GripperCommand.goal #" + ks + " -> " + r.describe(),
               "position=" + num17(goal.command.position) + " max_effort=" + num17(goal.command.max_effort),
               yaml17(goal));
    if (r.accept_delay_ms > 0) std::this_thread::sleep_for(milliseconds(r.accept_delay_ms));
    const bool reject = r.kind == GripperReply::Kind::REJECT;
    const std::string response = reject ? "REJECT" : "ACCEPT_AND_EXECUTE";
    // recorded BEFORE it is sent, so a client cannot have it before the capture does
    log_.event("ACTION", name_, "goal #" + ks + " response " + response);
    {
      std::lock_guard<std::mutex> g(m_);
      records_[static_cast<std::size_t>(k - 1)].response = response;
      records_[static_cast<std::size_t>(k - 1)].responded = Clock::now();
    }
    return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<Handle>& h)
  {
    int k = 0;
    {
      std::lock_guard<std::mutex> g(m_);
      k = ordinals_.at(h->get_goal_id());
      ++records_[static_cast<std::size_t>(k - 1)].cancel_requests;
    }
    log_.event("ACTION", name_, "goal #" + std::to_string(k) + " cancel request -> ACCEPT");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  std::shared_ptr<Gripper::Result> resultFor(const GripperReply& r, double goal_position) const
  {
    auto res = std::make_shared<Gripper::Result>();
    if (r.empty_result) return res;
    res->position = r.position_rule == GripperReply::Position::GOAL ? goal_position
                    : r.position_rule == GripperReply::Position::SHIM ? (goal_position >= 0.5 ? 1.0 : 0.0)
                                                                     : r.position;
    res->effort = r.effort;
    res->stalled = r.stalled;
    res->reached_goal = r.reached_goal;
    return res;
  }

  void onAccepted(const std::shared_ptr<Handle>& h)
  {
    int k = 0;
    GripperReply r;
    double position = 0.0;
    {
      std::lock_guard<std::mutex> g(m_);
      k = ordinals_.at(h->get_goal_id());
      r = records_[static_cast<std::size_t>(k - 1)].reply;
      position = records_[static_cast<std::size_t>(k - 1)].position;
    }
    const std::string ks = std::to_string(k);
    switch (r.kind)
    {
      case GripperReply::Kind::SUCCEED:
      case GripperReply::Kind::ABORT:
      {
        const bool ok = r.kind == GripperReply::Kind::SUCCEED;
        auto res = resultFor(r, position);
        const std::string status = ok ? "SUCCEEDED" : "ABORTED";
        log_.event("ACTION", name_, "goal #" + ks + " result " + status, resultText(*res, r.empty_result));
        {
          std::lock_guard<std::mutex> g(m_);
          auto& rec = records_[static_cast<std::size_t>(k - 1)];
          rec.terminal = status;
          rec.finished = Clock::now();
          rec.finished_by = "script";
          rec.result = *res;
          rec.result_empty = r.empty_result;
        }
        if (ok) h->succeed(res);
        else h->abort(res);
        break;
      }
      case GripperReply::Kind::WITHHOLD:
      {
        log_.event("ACTION", name_, "goal #" + ks + " held EXECUTING: no result (script)");
        std::lock_guard<std::mutex> g(m_);
        held_[k] = h;
        break;
      }
      case GripperReply::Kind::PREEMPT:
      {
        std::lock_guard<std::mutex> g(m_);
        held_[k] = h;
        workers_.push_back(std::async(std::launch::async, [this, h, k] { preempt(h, k); }));
        break;
      }
      case GripperReply::Kind::REJECT:
        break;   // never accepted
    }
  }

  // The second client cancels every goal on the server; the server ACCEPTs (the goal becomes
  // CANCELING), then ends it CANCELED with an empty result -- what a preempted client receives.
  void preempt(const std::shared_ptr<Handle>& h, int k)
  {
    const std::string ks = std::to_string(k);
    if (!preempt_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      log_.event("CLIENT", preempt_name_, "the second client sees no server");
      return;
    }
    log_.event("CLIENT", preempt_name_, "the second client asks to cancel every goal (preempting goal #" + ks + ")");
    ++preempts_sent_;
    auto fut = preempt_client_->async_cancel_all_goals();
    if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      log_.event("CLIENT", preempt_name_, "no cancel response");
      return;
    }
    const auto resp = fut.get();
    if (!waitUntil([&] { return h->is_canceling(); }, milliseconds(2000)))
    {
      log_.event("ACTION", name_, "goal #" + ks + " is not CANCELING after the cancel request");
      return;
    }
    log_.event("ACTION", name_, "goal #" + ks + " result CANCELED",
               "(empty result); the cancel response listed " +
                 std::to_string(resp ? resp->goals_canceling.size() : 0u) + " goal(s)");
    {
      std::lock_guard<std::mutex> g(m_);
      auto& rec = records_[static_cast<std::size_t>(k - 1)];
      rec.terminal = "CANCELED";
      rec.finished = Clock::now();
      rec.finished_by = "a second client's cancel";
      rec.result_empty = true;
      held_.erase(k);
    }
    h->canceled(std::make_shared<Gripper::Result>());
  }

  Log& log_;
  std::deque<GripperReply> script_;
  std::string script_text_;
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<rclcpp_action::Server<Gripper>> server_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  rclcpp::Node::SharedPtr preempt_node_;
  rclcpp_action::Client<Gripper>::SharedPtr preempt_client_;
  std::string preempt_name_;
  Spinner<rclcpp::executors::SingleThreadedExecutor> preempt_spin_{};
  mutable std::mutex m_;
  std::vector<GoalRecord> records_;
  std::map<rclcpp_action::GoalUUID, int> ordinals_;
  std::map<int, std::shared_ptr<Handle>> held_;
  std::vector<std::future<void>> workers_;
  std::atomic<int> preempts_sent_{0};
};

// --- what a goal record says, for the semantic line --------------------------------------------------
inline std::string goalCommand(const GoalRecord& g)
{
  return "GripperCommand(position=" + num17(g.position) + ",max_effort=" + num17(g.max_effort) + ")";
}

// `returned`: when the client's call returned (the tick, or TaskRunner::run).
inline std::string goalAck(const GoalRecord& g, Clock::time_point returned)
{
  if (g.response.empty()) return "no goal response";
  const std::string a = g.response == "REJECT" ? "REJECTED" : "ACCEPTED";
  return g.responded > returned ? a + " after the client had returned" : a;
}

// What the goal had reached when the client returned, and what the server did with it after.
inline std::string goalAfterwards(const GoalRecord& g, Clock::time_point returned)
{
  std::vector<std::string> later;
  if (!g.response.empty() && g.responded > returned) later.push_back(g.response == "REJECT" ? "REJECTED" : "ACCEPTED");
  if (!g.terminal.empty() && g.finished > returned) later.push_back(g.terminal + " by " + g.finished_by);
  return later.empty() ? std::string() : " (afterwards: " + join(later) + ")";
}

inline std::string goalState(const GoalRecord& g, Clock::time_point returned)
{
  if (g.response == "REJECT" && g.responded <= returned) return "-";
  if (g.response.empty() || g.responded > returned) return "not answered when the client returned" + goalAfterwards(g, returned);
  if (g.terminal.empty() || g.finished > returned) return "EXECUTING when the client returned" + goalAfterwards(g, returned);
  return g.terminal + "{" + resultText(g.result, g.result_empty) + "}";
}

inline std::string goalCode(const GoalRecord& g, Clock::time_point returned)
{
  if (g.response == "REJECT" && g.responded <= returned) return "REJECTED";
  if (g.terminal.empty() || g.finished > returned) return "none before the client returned";
  return g.terminal;
}

// Which wait expired, from the order of what the server did and when the client returned.
inline std::string goalTimeout(const GoalRecord& g, Clock::time_point returned)
{
  if (g.response.empty() || g.responded > returned) return "goal-response wait (the response came after the client returned)";
  if (g.response == "REJECT") return "none";
  if (g.terminal.empty() || g.finished > returned) return "result wait (no result before the client returned)";
  return "none";
}

// ===================================================================================================
// BoolRecorder / BoolProvider
// ===================================================================================================
using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

inline Gid gidOf(const rmw_gid_t& g)
{
  Gid out{};
  std::copy(g.data, g.data + RMW_GID_STORAGE_SIZE, out.begin());
  return out;
}

class BoolRecorder
{
public:
  enum class Qos { VOLATILE, LATCHED };   // LATCHED == TRANSIENT_LOCAL: created late, a late joiner

  BoolRecorder(const std::string& node_name, const std::string& topic, Qos qos) : topic_(topic), qos_(qos)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name, "/", rclcpp::NodeOptions().use_global_arguments(false));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS q{rclcpp::KeepLast(10)};
    q.reliable();
    if (qos == Qos::LATCHED) q.transient_local();
    else q.durability_volatile();
    rclcpp::SubscriptionOptions so;
    so.callback_group = group_;
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic, q,
      [this](std::shared_ptr<const std_msgs::msg::Bool> m, const rclcpp::MessageInfo& info) {
        const Gid gid = gidOf(info.get_rmw_message_info().publisher_gid);
        refresh();
        std::lock_guard<std::mutex> g(m_);
        samples_.push_back({gid, m->data});
      },
      so);
    timer_ = node_->create_wall_timer(milliseconds(50), [this] { refresh(); }, group_);
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~BoolRecorder()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  BoolRecorder(const BoolRecorder&) = delete;
  BoolRecorder& operator=(const BoolRecorder&) = delete;

  std::string qosText() const
  {
    return qos_ == Qos::LATCHED ? "transient_local reliable depth 10" : "volatile reliable depth 10";
  }
  std::size_t count() const
  {
    std::lock_guard<std::mutex> g(m_);
    return samples_.size();
  }
  std::size_t publishers() const { return node_->count_publishers(topic_); }

  // (publisher, value) in arrival order; a publisher the graph never named is "?".
  std::vector<std::pair<std::string, bool>> samples() const
  {
    std::lock_guard<std::mutex> g(m_);
    std::vector<std::pair<std::string, bool>> out;
    for (const auto& s : samples_)
    {
      const auto it = names_.find(s.first);
      out.emplace_back(it == names_.end() ? std::string("?") : it->second, s.second);
    }
    return out;
  }
  // Per publisher (sorted by name), its samples in the order they arrived: one writer's samples
  // arrive in the order it wrote them, so this is deterministic; the interleaving is not.
  std::string byPublisher() const
  {
    std::map<std::string, std::vector<std::string>> per;
    for (const auto& [who, v] : samples()) per[who].push_back(boolText(v));
    if (per.empty()) return "(none)";
    std::vector<std::string> parts;
    for (const auto& [who, vs] : per) parts.push_back(who + ": " + join(vs));
    return join(parts, "; ");
  }
  // The sorted multiset "<publisher>:<value>".
  std::string multiset() const
  {
    std::vector<std::string> v;
    for (const auto& [who, b] : samples()) v.push_back(who + ":" + boolText(b));
    std::sort(v.begin(), v.end());
    return v.empty() ? "(none)" : "{" + join(v) + "}";
  }
  std::vector<std::string> values() const
  {
    std::vector<std::string> v;
    for (const auto& s : samples()) v.push_back(boolText(s.second));
    return v;
  }
  // Waits until `n` samples arrived, then until none has arrived for `quiet`.
  void settle(std::size_t n, milliseconds budget, milliseconds quiet = milliseconds(300)) const
  {
    waitUntil([&] { return count() >= n; }, budget);
    std::size_t seen = count();
    auto since = Clock::now();
    const auto deadline = Clock::now() + budget;
    while (Clock::now() < deadline && Clock::now() - since < quiet)
    {
      std::this_thread::sleep_for(milliseconds(20));
      if (count() != seen) { seen = count(); since = Clock::now(); }
    }
  }
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  void refresh()
  {
    std::vector<rclcpp::TopicEndpointInfo> infos;
    try { infos = node_->get_publishers_info_by_topic(topic_); }
    catch (const std::exception&) { return; }
    std::lock_guard<std::mutex> g(m_);
    for (const auto& i : infos)
      if (i.node_name() != "_NODE_NAME_UNKNOWN_")
        names_[i.endpoint_gid()] = fqn(i.node_namespace(), i.node_name());
  }

  std::string topic_;
  Qos qos_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
  mutable std::mutex m_;
  std::vector<std::pair<Gid, bool>> samples_;
  std::map<Gid, std::string> names_;
};

class BoolProvider
{
public:
  BoolProvider(const std::string& node_name, const std::string& ns, const std::string& topic, bool latched)
  : latched_(latched)
  {
    node_ = std::make_shared<rclcpp::Node>(node_name, ns, rclcpp::NodeOptions().use_global_arguments(false));
    rclcpp::QoS q{rclcpp::KeepLast(1)};
    q.reliable();
    if (latched) q.transient_local();
    pub_ = node_->create_publisher<std_msgs::msg::Bool>(topic, q);
    name_ = pub_->get_topic_name();
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~BoolProvider()
  {
    stopPeriodic();
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  BoolProvider(const BoolProvider&) = delete;
  BoolProvider& operator=(const BoolProvider&) = delete;

  void publish(bool v)
  {
    {
      std::lock_guard<std::mutex> g(m_);
      published_.emplace_back(Clock::now(), v);
    }
    std_msgs::msg::Bool m;
    m.data = v;
    pub_->publish(m);
  }
  void startPeriodic(bool v, milliseconds period)
  {
    stopPeriodic();
    timer_ = node_->create_wall_timer(period, [this, v] { publish(v); });
  }
  void stopPeriodic()
  {
    if (timer_) timer_->cancel();
    timer_.reset();
  }
  std::size_t matched() const { return pub_->get_subscription_count(); }
  // What was published in [from, to]: "true x N" collapsed to the distinct values, in order.
  std::vector<bool> publishedBetween(Clock::time_point from, Clock::time_point to) const
  {
    std::lock_guard<std::mutex> g(m_);
    std::vector<bool> v;
    for (const auto& [t, b] : published_)
      if (t >= from && t <= to && (v.empty() || v.back() != b)) v.push_back(b);
    return v;
  }
  std::vector<bool> publishedBefore(Clock::time_point t0) const
  {
    std::lock_guard<std::mutex> g(m_);
    std::vector<bool> v;
    for (const auto& [t, b] : published_)
      if (t < t0) v.push_back(b);
    return v;
  }
  std::string qosText() const
  {
    return latched_ ? "transient_local reliable depth 1" : "volatile reliable depth 1";
  }
  const std::string& name() const { return name_; }
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  bool latched_;
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
  mutable std::mutex m_;
  std::vector<std::pair<Clock::time_point, bool>> published_;
};

// ===================================================================================================
// FakePolicyRuntime -- trainit_policy_runtime as RunPolicy sees it
// ===================================================================================================
// policy_runtime_node.py (85053b3): ~/run is a std_srvs/Trigger; ~/status a std_msgs/String with
// json.dumps({"policy", "mode", "state", ...}) on RELIABLE + TRANSIENT_LOCAL depth 1; "running" is
// published inside the run handler, before the answer; a refusal publishes nothing.
struct PolicyScript
{
  bool success{true};
  std::string message{"pure run started"};
  int respond_after_ms{0};                  // the run service answers this late
  std::vector<std::string> inline_states{}; // published inside the handler after "running", before
                                            // the answer (hybrid: "hybrid_target_ready")
  std::vector<std::string> then_states{};   // published AFTER the answer, once RunPolicy's status
                                            // subscription has matched, 200 ms apart
  std::string describe() const
  {
    std::string s = "run -> success=" + boolText(success) + " message='" + message + "'";
    if (respond_after_ms > 0) s += " answered after " + std::to_string(respond_after_ms) + "ms";
    if (!inline_states.empty()) s += "; status before the answer: running -> " + join(inline_states, " -> ");
    if (!then_states.empty()) s += "; then status: " + join(then_states, " -> ");
    return s;
  }
};

class FakePolicyRuntime
{
public:
  FakePolicyRuntime(Log& log, const std::string& node_name, const std::string& mode, PolicyScript script)
  : log_(log), mode_(mode), script_(std::move(script))
  {
    node_ = std::make_shared<rclcpp::Node>(node_name, "/", rclcpp::NodeOptions().use_global_arguments(false));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS latch{rclcpp::KeepLast(1)};
    latch.reliable().transient_local();
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("~/status", latch);
    status_name_ = status_pub_->get_topic_name();
    run_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/run",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onRun(*res);
      },
      rmw_qos_profile_services_default, group_);
    run_name_ = run_->get_service_name();
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakePolicyRuntime()
  {
    std::vector<std::future<void>> w;
    {
      std::lock_guard<std::mutex> g(m_);
      w.swap(workers_);
    }
    for (auto& f : w) f.wait();
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  FakePolicyRuntime(const FakePolicyRuntime&) = delete;
  FakePolicyRuntime& operator=(const FakePolicyRuntime&) = delete;

  const std::string& runName() const { return run_name_; }
  const std::string& statusName() const { return status_name_; }
  int calls() const { return calls_.load(); }
  bool answered() const { return answered_.load(); }
  Clock::time_point answeredAt() const { std::lock_guard<std::mutex> g(m_); return answered_at_; }
  // (state, when it was published)
  std::vector<std::pair<std::string, Clock::time_point>> states() const
  {
    std::lock_guard<std::mutex> g(m_);
    return states_;
  }
  void waitWorkers()
  {
    std::vector<std::future<void>> w;
    {
      std::lock_guard<std::mutex> g(m_);
      w.swap(workers_);
    }
    for (auto& f : w) f.wait();
  }

private:
  void publishState(const std::string& state, const std::string& note = "")
  {
    std_msgs::msg::String m;
    m.data = "{\"policy\": \"m26_fake_policy\", \"mode\": \"" + mode_ + "\", \"state\": \"" + state + "\"}";
    log_.event("TOPIC", status_name_, "status published: " + state + note);
    {
      std::lock_guard<std::mutex> g(m_);
      states_.emplace_back(state, Clock::now());
    }
    status_pub_->publish(m);
  }

  void onRun(std_srvs::srv::Trigger::Response& res)
  {
    const int k = ++calls_;
    log_.event("SERVICE", run_name_, "Trigger.call #" + std::to_string(k) + " -> " + script_.describe());
    if (script_.success) publishState("running", " (inside the run handler, before the answer)");
    for (const auto& st : script_.inline_states) publishState(st, " (inside the run handler, before the answer)");
    if (script_.respond_after_ms > 0) std::this_thread::sleep_for(milliseconds(script_.respond_after_ms));
    res.success = script_.success;
    res.message = script_.message;
    log_.event("SERVICE", run_name_, "Trigger.call #" + std::to_string(k) + " answered success=" +
               boolText(res.success) + " message='" + res.message + "'");
    {
      std::lock_guard<std::mutex> g(m_);
      answered_at_ = Clock::now();
    }
    answered_ = true;
    if (!script_.then_states.empty())
    {
      std::lock_guard<std::mutex> g(m_);
      workers_.push_back(std::async(std::launch::async, [this] {
        const bool matched = waitUntil([this] { return status_pub_->get_subscription_count() >= 1; }, milliseconds(2000));
        for (const auto& s : script_.then_states)
        {
          std::this_thread::sleep_for(milliseconds(200));
          publishState(s, matched ? " (a status subscriber is matched)" : " (NO status subscriber matched)");
        }
      }));
    }
  }

  Log& log_;
  std::string mode_;
  PolicyScript script_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_;
  std::string run_name_, status_name_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  std::atomic<int> calls_{0};
  std::atomic<bool> answered_{false};
  mutable std::mutex m_;
  Clock::time_point answered_at_{};
  std::vector<std::pair<std::string, Clock::time_point>> states_;
  std::vector<std::future<void>> workers_;
};

// ===================================================================================================
// FakeSceneServices -- what scene_manager_node drives in move_group
// ===================================================================================================
inline std::string objectOp(int8_t op)
{
  switch (op)
  {
    case 0: return "ADD";
    case 1: return "REMOVE";
    case 2: return "APPEND";
    case 3: return "MOVE";
    default: return "OP" + std::to_string(op);
  }
}

// One line: what a PlanningScene diff does (the whole message is captured beside it).
inline std::string sceneSummary(const moveit_msgs::msg::PlanningScene& ps)
{
  std::vector<std::string> world, attached, defaults;
  for (const auto& o : ps.world.collision_objects) world.push_back(objectOp(o.operation) + " " + o.id);
  for (const auto& a : ps.robot_state.attached_collision_objects)
    attached.push_back(objectOp(a.object.operation) + " " + a.object.id + "@" + a.link_name);
  const auto& acm = ps.allowed_collision_matrix;
  for (std::size_t i = 0; i < acm.default_entry_names.size(); ++i)
    defaults.push_back(acm.default_entry_names[i] + "=" +
                       std::to_string(i < acm.default_entry_values.size() ? acm.default_entry_values[i] : -1));
  std::string s = "world=[" + join(world) + "] attached=[" + join(attached) + "]";
  if (!acm.entry_names.empty() || !defaults.empty())
    s += " acm(entries=[" + join(acm.entry_names) + "] defaults=[" + join(defaults) + "])";
  return s;
}

class FakeSceneServices
{
public:
  using GetScene = moveit_msgs::srv::GetPlanningScene;
  using ApplyScene = moveit_msgs::srv::ApplyPlanningScene;

  // refuse_attach: EMULATE move_group refusing a diff that ADDs an attached collision object
  // (answer success=false). Labelled as an emulation wherever it shows. The planning_scene TOPIC
  // is not recorded: PlanningSceneInterface publishes its remove diff right after creating its
  // publisher, so whether a subscriber sees it is a discovery race.
  FakeSceneServices(Log& log, const std::string& ns, bool refuse_attach = false)
  : log_(log), refuse_attach_(refuse_attach)
  {
    node_ = std::make_shared<rclcpp::Node>("m26_fake_scene", ns, rclcpp::NodeOptions().use_global_arguments(false));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const std::string get_name = resolved(node_, "get_planning_scene", true);
    get_ = node_->create_service<GetScene>(
      "get_planning_scene",
      [this, get_name](const std::shared_ptr<GetScene::Request> req, std::shared_ptr<GetScene::Response> res) {
        const std::string what = "GetPlanningScene components=" + std::to_string(req->components.components);
        log_.event("SERVICE", get_name, "GetPlanningScene.request components=" + std::to_string(req->components.components));
        res->scene.is_diff = true;              // an empty DIFF: never REPLACE a scene
        res->scene.robot_state.is_diff = true;
        std::lock_guard<std::mutex> g(m_);
        records_.push_back(what);
      },
      rmw_qos_profile_services_default, group_);
    const std::string apply_name = resolved(node_, "apply_planning_scene", true);
    apply_ = node_->create_service<ApplyScene>(
      "apply_planning_scene",
      [this, apply_name](const std::shared_ptr<ApplyScene::Request> req, std::shared_ptr<ApplyScene::Response> res) {
        bool adds_attached = false;
        std::set<std::string> attached_ids;
        for (const auto& a : req->scene.robot_state.attached_collision_objects) {
          const bool add = a.object.operation == moveit_msgs::msg::CollisionObject::ADD;
          adds_attached = adds_attached || add;
          if (add) attached_ids.insert(a.object.id);
        }
        // move_group's own rule (MoveIt Humble PlanningScene::setPlanningSceneDiffMsg, seen on a real
        // move_group in vNext I0): the robot state is applied first, and an attach under a world
        // object's id MOVES it off the world, so a world REMOVE of that id in the same diff fails and
        // the apply answers false (while the attach stays).
        bool remove_after_attach = false;
        for (const auto& o : req->scene.world.collision_objects)
          remove_after_attach = remove_after_attach ||
            (o.operation == moveit_msgs::msg::CollisionObject::REMOVE && attached_ids.count(o.id) > 0);
        const bool refuse = (refuse_attach_ && adds_attached) || remove_after_attach;
        res->success = !refuse;
        const int k = applies_.load() + 1;
        const std::string answer = "success=" + boolText(res->success) +
                                   (refuse_attach_ && adds_attached
                                      ? " (EMULATED REFUSAL: this fake refuses a diff that ADDs an attached object)"
                                      : remove_after_attach
                                          ? " (as move_group: the attach already moved the object off the world)"
                                          : "");
        log_.event("SERVICE", apply_name, "ApplyPlanningScene.request #" + std::to_string(k) + " -> " + answer,
                   sceneSummary(req->scene), yaml17(*req));
        {
          std::lock_guard<std::mutex> g(m_);
          records_.push_back("ApplyPlanningScene " + sceneSummary(req->scene) + " -> " + answer);
        }
        ++applies_;
      },
      rmw_qos_profile_services_default, group_);
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakeSceneServices()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  FakeSceneServices(const FakeSceneServices&) = delete;
  FakeSceneServices& operator=(const FakeSceneServices&) = delete;

  int applies() const { return applies_.load(); }
  // Every request so far, in order: "GetPlanningScene components=N" | "ApplyPlanningScene <summary> -> success=b".
  std::vector<std::string> records() const
  {
    std::lock_guard<std::mutex> g(m_);
    return records_;
  }
  std::size_t count() const
  {
    std::lock_guard<std::mutex> g(m_);
    return records_.size();
  }
  // Waits until at least `n` apply requests arrived, then until no request (of either kind) has
  // arrived for `quiet`. Returns whether `n` was reached.
  bool settle(int n, milliseconds budget, milliseconds quiet = milliseconds(600)) const
  {
    const bool reached = waitUntil([&] { return applies() >= n; }, budget);
    std::size_t seen = count();
    auto since = Clock::now();
    const auto deadline = Clock::now() + budget;
    while (Clock::now() < deadline && Clock::now() - since < quiet)
    {
      std::this_thread::sleep_for(milliseconds(20));
      const std::size_t now = count();
      if (now != seen) { seen = now; since = Clock::now(); }
    }
    return reached;
  }

private:
  Log& log_;
  bool refuse_attach_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<GetScene>::SharedPtr get_;
  rclcpp::Service<ApplyScene>::SharedPtr apply_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  std::atomic<int> applies_{0};
  mutable std::mutex m_;
  std::vector<std::string> records_;
};

// ===================================================================================================
// The runner's end-effector stack
// ===================================================================================================
// The end-effector keys trainit_run_bt reads (src/trainit_run_bt_node.cpp:143, 168-175) and their
// defaults: gripper_action "" (no GripperController), gripper_close_position 1.0,
// gripper_open_position 0.0, gripper_cmd_topic "" (no grasp Bool publisher).
struct RunnerKeys
{
  std::string gripper_action{};
  double close_pos{1.0};
  double open_pos{0.0};
  std::string gripper_cmd_topic{};

  std::string describe() const
  {
    return "gripper_action=" + (gripper_action.empty() ? std::string("(unset)") : gripper_action) +
           " gripper_close_position=" + num17(close_pos) + " gripper_open_position=" + num17(open_pos) +
           " gripper_cmd_topic=" + (gripper_cmd_topic.empty() ? std::string("(unset)") : gripper_cmd_topic);
  }
};

struct RunnerContext
{
  std::unique_ptr<trainit::GripperController> gripper;
  std::unique_ptr<trainit::ProcessController> process;
  trainit::BtContext ctx{};
};

// Built EXACTLY like trainit_run_bt_node.cpp:155-182: a GripperController only when gripper_action
// is set; the mock process controller (process_controller defaults to "mock"); the actuation
// targets; and, only when gripper_cmd_topic is set, a QoS(1).transient_local() Bool publisher on
// the runner's node. The runtime is null: no end-effector node (Close/Open/SetGripper,
// CheckRobotState without a position, SetReleasePolicy, RunPolicy) dereferences it.
inline std::unique_ptr<RunnerContext> makeRunnerContext(const rclcpp::Node::SharedPtr& node, const RunnerKeys& k)
{
  auto r = std::make_unique<RunnerContext>();
  if (!k.gripper_action.empty()) r->gripper = std::make_unique<trainit::GripperController>(node, k.gripper_action);
  r->process = std::make_unique<trainit::MockProcessController>(node->get_logger());
  // == BtContext ctx{runtime.get(), gripper.get(), process.get(), node} with a null runtime
  r->ctx.runtime = nullptr;
  r->ctx.gripper = r->gripper.get();
  r->ctx.process = r->process.get();
  r->ctx.node = node;
  r->ctx.gripper_close_pos = k.close_pos;
  r->ctx.gripper_open_pos = k.open_pos;
  if (!k.gripper_cmd_topic.empty())
  {
    auto qos = rclcpp::QoS(1).transient_local();
    r->ctx.gripper_cmd_pub = node->create_publisher<std_msgs::msg::Bool>(k.gripper_cmd_topic, qos);
  }
  return r;
}

}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_FAKES_HPP_
