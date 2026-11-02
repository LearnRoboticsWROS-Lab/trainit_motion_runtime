// =============================================================================
// pilz_sequence_backend.cpp
// =============================================================================
#include "trainit_motion_runtime/pilz_sequence_backend.hpp"
#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"   // PIPELINE_PILZ, PILZ_LIN/CIRC
#include "trainit_motion_runtime/motion_failure.hpp"

#include <chrono>
#include <future>
#include <sstream>

#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <rclcpp_action/exceptions.hpp>

using namespace std::chrono_literals;

namespace trainit
{

namespace
{
// vNext M2.7: a failure before MoveIt answered names its cause; the ONE sequence goal plans AND executes.
MotionResult unanswered(const std::string& stage, const char* cause)
{
  MotionResult r = MotionResult::failure(stage);
  r.phase = MotionPhase::PLAN_AND_EXECUTE;
  r.cause = cause;
  return r;
}
}  // namespace

PilzSequenceBackend::PilzSequenceBackend(rclcpp::Node::SharedPtr node, std::string group,
                                         std::string tip_link, std::string base_frame,
                                         double result_timeout_s)
: node_(std::move(node)), group_(std::move(group)), tip_(std::move(tip_link)),
  base_(std::move(base_frame)), result_timeout_s_(result_timeout_s)
{
  client_ = rclcpp_action::create_client<MoveGroupSequence>(node_, kActionName);
}

moveit_msgs::msg::MotionPlanRequest PilzSequenceBackend::buildRequest(
  const ProcessSegment& seg, const MotionOptions& options) const
{
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = group_;
  req.pipeline_id = PIPELINE_PILZ;
  req.planner_id = (seg.type == ProcessSegmentType::CIRCULAR) ? PILZ_CIRC : PILZ_LIN;
  req.num_planning_attempts = std::max(1, options.planning_attempts);
  req.allowed_planning_time = options.planning_time;
  req.max_velocity_scaling_factor = seg.velocity_scaling;
  req.max_acceleration_scaling_factor = seg.acceleration_scaling;

  // vNext M3.4 (owner D3): a segment may name its own link (CircularTarget::tip_link); none = the
  // configured tip, as before.
  const std::string& tip = seg.tip_link.empty() ? tip_ : seg.tip_link;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = base_;
  goal.pose = seg.target_pose;
  req.goal_constraints = {kinematic_constraints::constructGoalConstraints(
    tip, goal, options.goal_position_tolerance, options.goal_orientation_tolerance)};

  // CIRC needs an auxiliary point (interim/via or center), carried as a named
  // position constraint on the same link as the goal.
  if (seg.type == ProcessSegmentType::CIRCULAR &&
      (seg.interim_point.has_value() || seg.circle_center.has_value()))
  {
    moveit_msgs::msg::Constraints path;
    path.name = seg.circle_center.has_value() ? "center" : "interim";
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = base_;
    pc.link_name = tip;
    pc.weight = 1.0;
    geometry_msgs::msg::Pose aux;
    aux.orientation.w = 1.0;
    // vNext M3.4 (owner D3): the centre wins. `circle_center.value_or(interim_point.value())`
    // evaluated its argument first, so a centre with no interim point threw bad_optional_access.
    aux.position = seg.circle_center.has_value() ? *seg.circle_center : *seg.interim_point;
    pc.constraint_region.primitive_poses.push_back(aux);
    path.position_constraints.push_back(pc);
    req.path_constraints = path;
  }

  return req;
}

std::string PilzSequenceBackend::cancel(
  const rclcpp_action::ClientGoalHandle<MoveGroupSequence>::SharedPtr& handle)
{
  // The pattern of the end-effector registry's cancelGoal: a goal that ended meanwhile, or a cancel
  // that cannot be sent (a context shutting down), is a sentence, never an exception out of a tick.
  try
  {
    auto answer = client_->async_cancel_goal(handle);
    if (answer.wait_for(std::chrono::duration<double>(kCancelResponseTimeoutS)) != std::future_status::ready)
      return "the goal was asked to cancel; no answer within " + std::to_string(static_cast<int>(kCancelResponseTimeoutS)) + " s";
    const auto response = answer.get();
    if (!response) return "the goal was asked to cancel; the answer was empty";
    switch (response->return_code)
    {
      case action_msgs::srv::CancelGoal::Response::ERROR_NONE: return "the goal was cancelled";
      case action_msgs::srv::CancelGoal::Response::ERROR_REJECTED: return "the server REFUSED to cancel the goal";
      case action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED: return "the goal had already ended";
      default: return "the cancel was answered with code " + std::to_string(response->return_code);
    }
  }
  catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
  {
    return "the goal had already ended";
  }
  catch (const std::exception& e)
  {
    return std::string("the goal could not be asked to cancel: ") + e.what();
  }
}

MotionResult PilzSequenceBackend::run(const std::vector<ProcessSegment>& segments,
                                      const MotionOptions& options)
{
  if (segments.empty())
    return MotionResult::failure("pilz_sequence: empty segment list");
  // vNext M3.4 (review): a CIRC item without its via point or centre is no circle -- refused here,
  // whoever built it, instead of a goal with no aux constraint
  for (std::size_t i = 0; i < segments.size(); ++i)
    if (segments[i].type == ProcessSegmentType::CIRCULAR && !segments[i].interim_point && !segments[i].circle_center)
    {
      MotionResult r = MotionResult::failure("pilz_sequence: segment " + std::to_string(i) +
                                             " is a CIRC with neither a via point nor a centre -- nothing was sent");
      r.phase = MotionPhase::REQUEST;
      r.cause = kCauseCircleGeometry;
      RCLCPP_ERROR(node_->get_logger(), "%s", r.stage.c_str());
      return r;
    }

  if (!client_->wait_for_action_server(5s))
    return unanswered("pilz_sequence: 'sequence_move_group' action server not available", kCauseServerUnavailable);

  MoveGroupSequence::Goal goal;
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    moveit_msgs::msg::MotionSequenceItem item;
    item.req = buildRequest(segments[i], options);
    // Pilz requires the LAST item's blend_radius to be 0.
    item.blend_radius = (i + 1 == segments.size()) ? 0.0 : segments[i].blend_radius;
    goal.request.items.push_back(item);
  }
  goal.planning_options.plan_only = options.plan_only;

  RCLCPP_INFO(node_->get_logger(), "pilz_sequence: sending %zu segments (plan_only=%d)",
              segments.size(), options.plan_only);

  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(10s) != std::future_status::ready)
    return unanswered("pilz_sequence: goal send timed out", kCauseGoalSendTimeout);
  auto handle = goal_future.get();
  if (!handle)
    return unanswered("pilz_sequence: goal rejected", kCauseGoalRejected);

  auto result_future = client_->async_get_result(handle);
  if (result_future.wait_for(std::chrono::duration<double>(result_timeout_s_)) != std::future_status::ready)
  {
    // vNext M3.4 (owner D3): the caller now treats this motion as FAILED, so its goal must not go on
    // moving the robot: it is cancelled before the failure is returned.
    std::ostringstream os;
    os << "pilz_sequence: result timed out after " << result_timeout_s_ << " s; " << cancel(handle);
    // ... and the goal's end is awaited (bounded), so the caller's next motion does not overlap it
    os << (result_future.wait_for(std::chrono::duration<double>(kCancelResponseTimeoutS)) == std::future_status::ready
             ? "; the goal has ended" : "; the goal had not ended " + std::to_string(static_cast<int>(kCancelResponseTimeoutS)) + " s later");
    RCLCPP_ERROR(node_->get_logger(), "%s", os.str().c_str());
    return unanswered(os.str(), kCauseResultTimeout);
  }

  auto wrapped = result_future.get();
  const auto& resp = wrapped.result->response;

  MotionResult mr;
  mr.process.process_path_backend = "pilz_sequence";
  mr.process.number_of_input_waypoints = segments.size();
  mr.process.number_of_output_waypoints = resp.planned_trajectories.empty()
    ? 0 : resp.planned_trajectories.front().joint_trajectory.points.size();
  mr.error_code.val = resp.error_code.val;

  const bool ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                  (resp.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  mr.success = ok;
  if (!ok) mr.phase = MotionPhase::PLAN_AND_EXECUTE;   // vNext M2.7: the code is the answer's
  mr.process.execution_success = ok && !options.plan_only;
  mr.stage = ok ? "pilz_sequence: OK" : "pilz_sequence: FAILED (code=" +
                  std::to_string(resp.error_code.val) + ")";
  RCLCPP_INFO(node_->get_logger(), "%s", mr.stage.c_str());
  return mr;
}

}  // namespace trainit
