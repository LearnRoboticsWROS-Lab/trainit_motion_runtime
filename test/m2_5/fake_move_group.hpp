// vNext M2.5 test harness -- in-process fakes of everything TMR talks to below the BT.
//
//   FakeMoveGroup  one per namespace: /move_action (MoveGroup), /execute_trajectory
//                  (ExecuteTrajectory), sequence_move_group (Pilz MoveGroupSequence),
//                  compute_cartesian_path, get_planning_scene + apply_planning_scene, tripwire
//                  services BEFORE never calls, and the topics MoveIt clients publish on.
//                  Every endpoint can be served or withheld, so a test can prove where a
//                  constructor blocks.
//   FakeJointStates  <ns>/joint_states at 100 Hz, RELIABLE, stamp = now; can be stopped.
//   FakeServo      a moveit_servo node's ROS contract (ADR-0015 Decision 5).
//   FakeGripper    a control_msgs/GripperCommand action server.
//
// MoveIt 2.5.10 facts the fakes respect: a REJECTED MoveGroup or ExecuteTrajectory goal makes
// MoveGroupInterface dereference a null result (segfault) and plan()/execute() have no client
// timeout -- so those two servers ALWAYS accept and always answer (a timeout is emulated as ABORT
// with TIMED_OUT); REJECT and a withheld result exist only on sequence_move_group. The
// GetPlanningScene answer is an empty DIFF (a non-diff empty scene would REPLACE the monitored
// scene). Fakes are never named move_group (the graph guard keeps its meaning) and forward nothing.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_5_FAKE_MOVE_GROUP_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_5_FAKE_MOVE_GROUP_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_planner_params.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/srv/query_planner_interfaces.hpp>
#include <moveit_msgs/srv/set_planner_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "capture.hpp"

namespace m25
{

inline const std::vector<std::string> kJoints{"j1", "j2", "j3"};

inline std::string resolvedName(const rclcpp::Node::SharedPtr& node, const std::string& name,
                                bool is_service)
{
  return rclcpp::expand_topic_or_service_name(name, node->get_name(), node->get_namespace(),
                                              is_service);
}

// --- scripted answers --------------------------------------------------------------------------------
using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

struct Reply
{
  // the action status the server ends the goal with; `code` is the MoveIt error code in the
  // result, independent of it (so "SUCCEEDED carrying -1" and "ABORTED carrying SUCCESS" exist).
  // No CANCELED: a server may only cancel a goal its client asked to cancel (rcl_action refuses
  // EXECUTING -> CANCELED), and no TMR path cancels -- PREEMPTED is an ABORT carrying -7.
  enum class Status { SUCCEED, ABORT, REJECT, HOLD };
  Status status{Status::SUCCEED};
  int code{ErrorCodes::SUCCESS};
  bool empty_trajectory{false};   // trajectory fixtures for the validator
  bool joint_jump{false};         // one 0.6 rad step (max_joint_jump is 0.5)
  bool out_of_limits{false};      // j1 = 3.5 (limit 3.0)
  bool decreasing_time{false};
  double fraction{1.0};           // GetCartesianPath

  static Reply succeed() { return {}; }
  static Reply abort(int code) { Reply r; r.status = Status::ABORT; r.code = code; return r; }
  static Reply preempted() { return abort(ErrorCodes::PREEMPTED); }
  static Reply reject() { Reply r; r.status = Status::REJECT; return r; }
  static Reply hold() { Reply r; r.status = Status::HOLD; return r; }
  static Reply carrying(Status s, int code) { Reply r; r.status = s; r.code = code; return r; }
  Reply& with(bool Reply::*flag) { this->*flag = true; return *this; }

  std::string describe() const
  {
    static const char* names[] = {"SUCCEED", "ABORT", "REJECT", "HOLD"};
    std::string s = std::string(names[static_cast<int>(status)]) + "(" + std::to_string(code);
    if (empty_trajectory) s += ",empty";
    if (joint_jump) s += ",jump";
    if (out_of_limits) s += ",out_of_limits";
    if (decreasing_time) s += ",decreasing_time";
    if (fraction != 1.0) s += ",fraction=" + num17(fraction);
    return s + ")";
  }
};

inline bool isPoseGoal(const moveit_msgs::msg::MotionPlanRequest& req)
{
  for (const auto& c : req.goal_constraints)
    if (!c.position_constraints.empty() || !c.orientation_constraints.empty()) return true;
  return false;
}

inline std::string goalKind(const moveit_msgs::msg::MotionPlanRequest& req)
{
  if (req.goal_constraints.empty()) return "empty";
  if (isPoseGoal(req)) return "pose";
  for (const auto& c : req.goal_constraints)
    if (!c.joint_constraints.empty()) return "joint";
  return "other";
}

inline std::string requestSummary(const moveit_msgs::msg::MotionPlanRequest& r)
{
  std::ostringstream os;
  os << "group=" << r.group_name << " pipeline=" << r.pipeline_id << " planner=" << r.planner_id
     << " target=" << goalKind(r);
  std::string frames;
  for (const auto& c : r.goal_constraints)
  {
    for (const auto& p : c.position_constraints)
      frames += (frames.empty() ? "" : ",") + p.link_name + "@" + p.header.frame_id;
    for (const auto& o : c.orientation_constraints)
      frames += (frames.empty() ? "" : ",") + o.link_name + "@" + o.header.frame_id;
  }
  if (!frames.empty()) os << " frames=" << frames;
  os << " vel=" << num17(r.max_velocity_scaling_factor)
     << " acc=" << num17(r.max_acceleration_scaling_factor)
     << " time=" << num17(r.allowed_planning_time) << " attempts=" << r.num_planning_attempts;
  if (!r.path_constraints.name.empty()) os << " path_constraints=" << r.path_constraints.name;
  return os.str();
}

// A trajectory TrajectoryValidator accepts unless a fixture flag breaks it: the group's joints
// from (0,0,0) to `goal`, steps <= 0.25 rad, time k*0.1 s, no velocities. +0.001*ordinal on j3
// names the attempt that produced it, so an executed trajectory says which plan it was.
inline moveit_msgs::msg::RobotTrajectory fakeTrajectory(std::vector<double> goal, int ordinal,
                                                        const Reply& r)
{
  moveit_msgs::msg::RobotTrajectory t;
  t.joint_trajectory.joint_names = kJoints;
  if (r.empty_trajectory) return t;
  goal[2] += 0.001 * ordinal;
  double span = 0.0;
  for (double g : goal) span = std::max(span, std::abs(g));
  const int steps = std::max(1, static_cast<int>(std::ceil(span / 0.25)));
  for (int k = 0; k <= steps; ++k)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    for (double g : goal) p.positions.push_back(g * k / steps);
    const int tenths = (r.decreasing_time && k == steps) ? 0 : k;
    p.time_from_start.sec = tenths / 10;
    p.time_from_start.nanosec = static_cast<uint32_t>((tenths % 10) * 100000000);
    t.joint_trajectory.points.push_back(p);
  }
  if (r.joint_jump)
  {
    auto p = t.joint_trajectory.points.back();
    p.positions[0] += 0.6;
    p.time_from_start.sec += 1;
    t.joint_trajectory.points.push_back(p);
  }
  if (r.out_of_limits) t.joint_trajectory.points.back().positions[0] = 3.5;
  return t;
}

inline std::vector<double> jointGoalOf(const moveit_msgs::msg::MotionPlanRequest& req)
{
  if (req.goal_constraints.empty() || req.goal_constraints[0].joint_constraints.empty())
    return {0.2, -0.3, 0.4};
  std::vector<double> q{0.0, 0.0, 0.0};
  for (const auto& jc : req.goal_constraints[0].joint_constraints)
    for (std::size_t j = 0; j < kJoints.size(); ++j)
      if (jc.joint_name == kJoints[j]) q[j] = jc.position;
  return q;
}

template <typename ActionT>
using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

// --- the fake move_group, one per namespace ------------------------------------------------------------
class FakeMoveGroup
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using Execute = moveit_msgs::action::ExecuteTrajectory;
  using Sequence = moveit_msgs::action::MoveGroupSequence;
  using GetScene = moveit_msgs::srv::GetPlanningScene;
  using ApplyScene = moveit_msgs::srv::ApplyPlanningScene;
  using Cartesian = moveit_msgs::srv::GetCartesianPath;

  enum Endpoint : unsigned
  {
    MOVE_ACTION = 1u << 0,
    EXECUTE = 1u << 1,
    SEQUENCE = 1u << 2,
    CARTESIAN = 1u << 3,
    SCENE_SERVICES = 1u << 4,   // get_planning_scene + apply_planning_scene
    TRIPWIRES = 1u << 5,
    TOPICS = 1u << 6,
    ALL = 0x7fu,
  };

  FakeMoveGroup(Log& log, const std::string& ns = "/", unsigned serve = ALL) : log_(log)
  {
    node_ = std::make_shared<rclcpp::Node>("m25_fake_move_group", ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    // MUTUALLY EXCLUSIVE, not reentrant: rclcpp_action's ServerBase (Humble) is not safe when one
    // server's goal and result requests run on two executor threads at once -- under load a
    // result request then answers STATUS_UNKNOWN and MoveGroupInterface re-plans
    // ("unknown result code"), observed with 16 saturated cores. Every callback here is short.
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    spin_.executor().add_node(node_);
    spin_.start();
    start(serve);
  }

  ~FakeMoveGroup()
  {
    releaseHeld();
    spin_.stop();
    spin_.executor().remove_node(node_);
  }

  // Serve the endpoints in `which` (a constructor that waits for them then proceeds).
  void start(unsigned which)
  {
    if ((which & MOVE_ACTION) && !mg_server_) makeMoveAction();
    if ((which & EXECUTE) && !ex_server_) makeExecute();
    if ((which & SEQUENCE) && !seq_server_) makeSequence();
    if ((which & CARTESIAN) && !cart_) makeCartesian();
    if ((which & SCENE_SERVICES) && !scene_) makeSceneServices();
    if ((which & TRIPWIRES) && trips_.empty()) makeTripwires();
    if ((which & TOPICS) && !event_sub_) makeTopics();
  }

  // Withdraw a server AFTER a runtime was built; returns once a fresh client sees it gone.
  void dropMoveAction() { mg_server_.reset(); waitGone<MoveGroup>("move_action"); }
  void dropExecute() { ex_server_.reset(); waitGone<Execute>("execute_trajectory"); }
  void dropSequence() { seq_server_.reset(); waitGone<Sequence>("sequence_move_group"); }

  // A fresh scenario: scripts cleared, rules back to their defaults, ordinals back to 0.
  void resetScenario()
  {
    std::lock_guard<std::mutex> g(script_m_);
    mg_script.clear(); ex_script.clear(); seq_script.clear(); cart_script.clear();
    chomp_refuses_pose = true;
    mg_ordinal_ = 0; seq_ordinal_ = 0; cart_ordinal_ = 0;
  }

  // Finish every held goal (HOLD) with ABORT.
  void releaseHeld()
  {
    std::vector<std::function<void()>> pending;
    { std::lock_guard<std::mutex> g(held_m_); pending.swap(held_); }
    for (auto& f : pending) f();
  }

  std::string scriptText() const
  {
    std::lock_guard<std::mutex> g(script_m_);
    std::string s;
    auto add = [&](const char* name, const std::deque<Reply>& q) {
      if (q.empty()) return;
      s += (s.empty() ? "" : " ") + std::string(name) + ":";
      for (const auto& r : q) s += " " + r.describe();
    };
    add("move_action", mg_script);
    add("execute_trajectory", ex_script);
    add("sequence_move_group", seq_script);
    add("compute_cartesian_path", cart_script);
    if (!chomp_refuses_pose) s += (s.empty() ? "" : " ") + std::string("rule chomp_refuses_pose OFF");
    return s.empty() ? "default (SUCCEED; chomp refuses a pose goal with -16)" : s;
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  // per-endpoint scripts, consumed in order; empty -> the default rule, else SUCCEED
  std::deque<Reply> mg_script, ex_script, seq_script, cart_script;
  // the mock-verified KNOWN_DEBT behaviour: the CHOMP pipeline refuses a pose goal (-16)
  bool chomp_refuses_pose{true};

private:
  template <typename ActionT>
  void waitGone(const std::string& name)
  {
    auto probe = std::make_shared<rclcpp::Node>("m25_gone_probe", node_->get_namespace(),
                                                rclcpp::NodeOptions().use_global_arguments(false));
    auto client = rclcpp_action::create_client<ActionT>(probe, name);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (client->action_server_is_ready() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  Reply next(std::deque<Reply>& script, const std::optional<Reply>& rule = std::nullopt)
  {
    std::lock_guard<std::mutex> g(script_m_);
    if (!script.empty())
    {
      Reply r = script.front();
      script.pop_front();
      return r;
    }
    return rule.value_or(Reply::succeed());
  }

  template <typename ActionT>
  void finish(const std::shared_ptr<GoalHandle<ActionT>>& h, const Reply& r,
              const std::shared_ptr<typename ActionT::Result>& res)
  {
    switch (r.status)
    {
      case Reply::Status::SUCCEED: h->succeed(res); break;
      case Reply::Status::HOLD:
      {
        std::lock_guard<std::mutex> g(held_m_);
        held_.push_back([h, res] { if (h->is_active()) h->abort(res); });
        break;
      }
      default: h->abort(res); break;
    }
  }

  void makeMoveAction()
  {
    const std::string name = resolvedName(node_, "move_action", false);
    mg_server_ = rclcpp_action::create_server<MoveGroup>(
      node_, "move_action",
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveGroup::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;   // never REJECT: MGI segfaults
      },
      [](std::shared_ptr<GoalHandle<MoveGroup>>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this, name](std::shared_ptr<GoalHandle<MoveGroup>> h) {
        const MoveGroup::Goal goal = *h->get_goal();
        const int ordinal = ++mg_ordinal_;
        std::optional<Reply> rule;
        if (chomp_refuses_pose && goal.request.pipeline_id == "chomp" && isPoseGoal(goal.request))
          rule = Reply::abort(ErrorCodes::INVALID_GOAL_CONSTRAINTS);
        const Reply r = next(mg_script, rule);
        log_.event("ACTION", name, "MoveGroup.goal #" + std::to_string(ordinal) + " -> " + r.describe(),
                   requestSummary(goal.request) + " plan_only=" +
                     std::to_string(goal.planning_options.plan_only),
                   yaml17(goal));
        auto res = std::make_shared<MoveGroup::Result>();
        res->error_code.val = r.code;
        res->planned_trajectory = fakeTrajectory(jointGoalOf(goal.request), ordinal, r);
        res->planning_time = 0.01;
        finish<MoveGroup>(h, r, res);
      },
      rcl_action_server_get_default_options(), group_);
  }

  void makeExecute()
  {
    const std::string name = resolvedName(node_, "execute_trajectory", false);
    ex_server_ = rclcpp_action::create_server<Execute>(
      node_, "execute_trajectory",
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Execute::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;   // never REJECT: MGI segfaults
      },
      [](std::shared_ptr<GoalHandle<Execute>>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this, name](std::shared_ptr<GoalHandle<Execute>> h) {
        const Execute::Goal goal = *h->get_goal();
        const Reply r = next(ex_script);
        log_.event("ACTION", name, "ExecuteTrajectory.goal -> " + r.describe(),
                   "trajectory\n" + trajectorySummary(goal.trajectory, 4));
        auto res = std::make_shared<Execute::Result>();
        res->error_code.val = r.code;
        finish<Execute>(h, r, res);
      },
      rcl_action_server_get_default_options(), group_);
  }

  void makeSequence()
  {
    const std::string name = resolvedName(node_, "sequence_move_group", false);
    seq_server_ = rclcpp_action::create_server<Sequence>(
      node_, "sequence_move_group",
      [this, name](const rclcpp_action::GoalUUID&, std::shared_ptr<const Sequence::Goal> goal) {
        std::lock_guard<std::mutex> g(script_m_);
        if (!seq_script.empty() && seq_script.front().status == Reply::Status::REJECT)
        {
          seq_script.pop_front();
          log_.event("ACTION", name, "MoveGroupSequence.goal -> REJECT", sequenceSummary(*goal),
                     yaml17(*goal));
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle<Sequence>>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this, name](std::shared_ptr<GoalHandle<Sequence>> h) {
        const Sequence::Goal goal = *h->get_goal();
        const int ordinal = ++seq_ordinal_;
        const Reply r = next(seq_script);
        log_.event("ACTION", name,
                   "MoveGroupSequence.goal #" + std::to_string(ordinal) + " -> " + r.describe(),
                   sequenceSummary(goal), yaml17(goal));
        auto res = std::make_shared<Sequence::Result>();
        res->response.error_code.val = r.code;
        res->response.planned_trajectories.push_back(fakeTrajectory({0.2, -0.3, 0.4}, ordinal, r));
        finish<Sequence>(h, r, res);
      },
      rcl_action_server_get_default_options(), group_);
  }

  static std::string sequenceSummary(const Sequence::Goal& goal)
  {
    std::string s = "items=" + std::to_string(goal.request.items.size()) +
                    " plan_only=" + std::to_string(goal.planning_options.plan_only);
    for (std::size_t i = 0; i < goal.request.items.size(); ++i)
      s += "\n  | item " + std::to_string(i) + ": " + requestSummary(goal.request.items[i].req) +
           " blend=" + num17(goal.request.items[i].blend_radius);
    return s;
  }

  void makeCartesian()
  {
    const std::string name = resolvedName(node_, "compute_cartesian_path", true);
    cart_ = node_->create_service<Cartesian>(
      "compute_cartesian_path",
      [this, name](const std::shared_ptr<Cartesian::Request> req,
                   std::shared_ptr<Cartesian::Response> res) {
        const int ordinal = ++cart_ordinal_;
        const Reply r = next(cart_script);
        std::ostringstream sum;
        sum << "group=" << req->group_name << " link=" << req->link_name
            << " frame=" << req->header.frame_id << " waypoints=" << req->waypoints.size()
            << " max_step=" << num17(req->max_step) << " jump=" << num17(req->jump_threshold)
            << " avoid_collisions=" << req->avoid_collisions
            << " vel=" << num17(req->max_velocity_scaling_factor)
            << " acc=" << num17(req->max_acceleration_scaling_factor);
        log_.event("SERVICE", name, "GetCartesianPath.request #" + std::to_string(ordinal) + " -> " +
                   r.describe(), sum.str(), yaml17(*req));
        res->error_code.val = r.code;
        res->fraction = r.fraction;
        res->solution = fakeTrajectory({0.2, -0.3, 0.4}, ordinal, r);
      },
      rmw_qos_profile_services_default, group_);
  }

  void makeSceneServices()
  {
    const std::string get_name = resolvedName(node_, "get_planning_scene", true);
    scene_ = node_->create_service<GetScene>(
      "get_planning_scene",
      [this, get_name](const std::shared_ptr<GetScene::Request> req,
                       std::shared_ptr<GetScene::Response> res) {
        log_.event("SERVICE", get_name,
                   "GetPlanningScene.request components=" +
                     std::to_string(req->components.components));
        res->scene.is_diff = true;              // an empty DIFF: never REPLACE the monitored scene
        res->scene.robot_state.is_diff = true;
      },
      rmw_qos_profile_services_default, group_);
    const std::string apply_name = resolvedName(node_, "apply_planning_scene", true);
    apply_ = node_->create_service<ApplyScene>(
      "apply_planning_scene",
      [this, apply_name](const std::shared_ptr<ApplyScene::Request> req,
                         std::shared_ptr<ApplyScene::Response> res) {
        log_.event("SERVICE", apply_name, "ApplyPlanningScene.request", "", yaml17(*req));
        res->success = true;
      },
      rmw_qos_profile_services_default, group_);
  }

  template <typename SrvT>
  void tripwire(const std::string& srv)
  {
    const std::string name = resolvedName(node_, srv, true);
    trips_.push_back(node_->create_service<SrvT>(
      srv,
      [this, name](const std::shared_ptr<typename SrvT::Request> req,
                   std::shared_ptr<typename SrvT::Response>) {
        log_.event("SERVICE", name, "TRIPWIRE.request", "", yaml17(*req));
      },
      rmw_qos_profile_services_default, group_));
  }

  void makeTripwires()
  {
    tripwire<moveit_msgs::srv::QueryPlannerInterfaces>("query_planner_interface");
    tripwire<moveit_msgs::srv::GetPlannerParams>("get_planner_params");
    tripwire<moveit_msgs::srv::SetPlannerParams>("set_planner_params");
    tripwire<moveit_msgs::srv::GetPositionIK>("compute_ik");
    tripwire<moveit_msgs::srv::GetPositionFK>("compute_fk");
    tripwire<moveit_msgs::srv::GetStateValidity>("check_state_validity");
  }

  void makeTopics()
  {
    rclcpp::SubscriptionOptions so;
    so.callback_group = group_;
    const std::string ev = resolvedName(node_, "trajectory_execution_event", false);
    event_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "trajectory_execution_event", rclcpp::SystemDefaultsQoS(),
      [this, ev](std_msgs::msg::String::ConstSharedPtr m) { log_.topic(ev, "String: " + m->data, ""); },
      so);
    const std::string aco = resolvedName(node_, "attached_collision_object", false);
    aco_sub_ = node_->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
      "attached_collision_object", rclcpp::SystemDefaultsQoS(),
      [this, aco](moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr m) {
        log_.topic(aco, "AttachedCollisionObject", yaml17(*m));
      },
      so);
    const std::string ps = resolvedName(node_, "planning_scene", false);
    scene_sub_ = node_->create_subscription<moveit_msgs::msg::PlanningScene>(
      "planning_scene", rclcpp::SystemDefaultsQoS(),
      [this, ps](moveit_msgs::msg::PlanningScene::ConstSharedPtr m) {
        log_.topic(ps, "PlanningScene", yaml17(*m));
      },
      so);
  }

  Log& log_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 4};
  std::shared_ptr<rclcpp_action::Server<MoveGroup>> mg_server_;
  std::shared_ptr<rclcpp_action::Server<Execute>> ex_server_;
  std::shared_ptr<rclcpp_action::Server<Sequence>> seq_server_;
  rclcpp::Service<Cartesian>::SharedPtr cart_;
  rclcpp::Service<GetScene>::SharedPtr scene_;
  rclcpp::Service<ApplyScene>::SharedPtr apply_;
  std::vector<rclcpp::ServiceBase::SharedPtr> trips_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr aco_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_sub_;
  mutable std::mutex script_m_;
  std::mutex held_m_;
  std::vector<std::function<void()>> held_;
  std::atomic<int> mg_ordinal_{0}, seq_ordinal_{0}, cart_ordinal_{0};
};

// --- joint states ----------------------------------------------------------------------------------------
class FakeJointStates
{
public:
  explicit FakeJointStates(const std::string& ns, std::vector<double> q = {0.0, 0.0, 0.0})
  : q_(std::move(q))
  {
    node_ = std::make_shared<rclcpp::Node>("m25_fake_joint_states", ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                 rclcpp::SystemDefaultsQoS());
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this] {
      std::lock_guard<std::mutex> g(m_);
      if (!on_) return;
      sensor_msgs::msg::JointState m;
      m.header.stamp = node_->now();
      m.name = kJoints;
      m.position = q_;
      pub_->publish(m);
    });
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakeJointStates()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }
  void set(std::vector<double> q) { std::lock_guard<std::mutex> g(m_); q_ = std::move(q); }
  void on(bool publishing) { std::lock_guard<std::mutex> g(m_); on_ = publishing; }
  std::string describe() const
  {
    std::lock_guard<std::mutex> g(m_);
    return on_ ? "joint_states " + resolvedName(node_, "joint_states", false) + "=" + vec17(q_)
               : "joint_states " + resolvedName(node_, "joint_states", false) + " OFF";
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Spinner<rclcpp::executors::SingleThreadedExecutor> spin_{};
  mutable std::mutex m_;
  std::vector<double> q_;
  bool on_{true};
};

// --- a moveit_servo node's contract (ServoTrack) ------------------------------------------------------------
class FakeServo
{
public:
  // `name` must be unique per scenario: ServoTrack's clients, publishers and "started once"
  // registry are process-static, keyed by name.
  FakeServo(Log& log, const std::string& ns, const std::string& name, bool serve_triggers = true,
            bool subscribe_twist = true)
  : log_(log)
  {
    node_ = std::make_shared<rclcpp::Node>(name, ns, rclcpp::NodeOptions().use_global_arguments(false));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (serve_triggers)
      for (const char* srv : {"~/start_servo", "~/unpause_servo", "~/pause_servo"})
      {
        const std::string full = resolvedName(node_, srv, true);
        triggers_.push_back(node_->create_service<std_srvs::srv::Trigger>(
          srv,
          [this, full](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            log_.event("SERVICE", full, "Trigger.call");
            res->success = true;          // moveit_servo 2.5.10 answers success unconditionally
          },
          rmw_qos_profile_services_default, group_));
      }
    status_pub_ = node_->create_publisher<std_msgs::msg::Int8>("~/status", rclcpp::SystemDefaultsQoS());
    if (subscribe_twist)
    {
      rclcpp::SubscriptionOptions so;
      so.callback_group = group_;
      twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/delta_twist_cmds", rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { onTwist(*m); }, so);
    }
    status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this] {
      std::lock_guard<std::mutex> g(m_);
      if (!halting_) return;
      std_msgs::msg::Int8 s;
      s.data = *inject_;
      status_pub_->publish(s);
    }, group_);
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakeServo()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }

  // After the first non-zero twist, publish this status every 10 ms (a halt, on demand).
  void injectAfterFirstTwist(int8_t code) { std::lock_guard<std::mutex> g(m_); inject_ = code; }
  // Called once, on the first non-zero twist: lets a scenario make the arm "arrive" (the fake
  // joint states jump to the target), so ServoTrack's feedback read sees the move converge.
  void onFirstNonzeroTwist(std::function<void()> fn) { std::lock_guard<std::mutex> g(m_); on_move_ = std::move(fn); }

  // The first command and the last (the release) -- never the count, which is rate-dependent.
  std::string twists() const
  {
    std::lock_guard<std::mutex> g(m_);
    if (!first_) return "twists: none\n";
    return "twist first: " + fmt(*first_) + "\ntwist last:  " + fmt(last_) + "\n";
  }

private:
  static std::string fmt(const geometry_msgs::msg::TwistStamped& t)
  {
    return "frame=" + t.header.frame_id + " lin=(" + num17(t.twist.linear.x) + "," +
           num17(t.twist.linear.y) + "," + num17(t.twist.linear.z) + ") ang=(" +
           num17(t.twist.angular.x) + "," + num17(t.twist.angular.y) + "," +
           num17(t.twist.angular.z) + ")";
  }
  void onTwist(const geometry_msgs::msg::TwistStamped& m)
  {
    std::lock_guard<std::mutex> g(m_);
    if (!first_) first_ = m;
    last_ = m;
    const bool nonzero = m.twist.linear.x != 0.0 || m.twist.linear.y != 0.0 ||
                         m.twist.linear.z != 0.0 || m.twist.angular.x != 0.0 ||
                         m.twist.angular.y != 0.0 || m.twist.angular.z != 0.0;
    if (nonzero && inject_) halting_ = true;
    if (nonzero && on_move_)
    {
      auto fn = std::move(on_move_);
      on_move_ = nullptr;
      fn();
    }
  }

  Log& log_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> triggers_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
  mutable std::mutex m_;
  std::optional<geometry_msgs::msg::TwistStamped> first_;
  geometry_msgs::msg::TwistStamped last_;
  std::optional<int8_t> inject_;
  bool halting_{false};
  std::function<void()> on_move_;
};

// --- a GripperCommand controller ------------------------------------------------------------------------
class FakeGripper
{
public:
  using Gripper = control_msgs::action::GripperCommand;
  FakeGripper(Log& log, const std::string& ns, const std::string& action) : log_(log)
  {
    node_ = std::make_shared<rclcpp::Node>("m25_fake_gripper", ns,
                                           rclcpp::NodeOptions().use_global_arguments(false));
    const std::string name = resolvedName(node_, action, false);
    server_ = rclcpp_action::create_server<Gripper>(
      node_, action,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Gripper::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle<Gripper>>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this, name](std::shared_ptr<GoalHandle<Gripper>> h) {
        const auto goal = *h->get_goal();
        log_.event("ACTION", name, "GripperCommand.goal", "", yaml17(goal));
        auto res = std::make_shared<Gripper::Result>();
        res->reached_goal = true;
        res->position = goal.command.position;
        h->succeed(res);
      });
    spin_.executor().add_node(node_);
    spin_.start();
  }
  ~FakeGripper()
  {
    spin_.stop();
    spin_.executor().remove_node(node_);
  }

private:
  Log& log_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp_action::Server<Gripper>> server_;
  Spinner<rclcpp::executors::MultiThreadedExecutor> spin_{rclcpp::ExecutorOptions(), 2};
};

}  // namespace m25

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_5_FAKE_MOVE_GROUP_HPP_
