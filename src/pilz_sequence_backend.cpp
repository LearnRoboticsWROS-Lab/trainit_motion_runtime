// =============================================================================
// pilz_sequence_backend.cpp
// =============================================================================
#include "trainit_motion_runtime/pilz_sequence_backend.hpp"

#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit/kinematic_constraints/utils.h>

using namespace std::chrono_literals;

namespace trainit
{

namespace
{
constexpr const char* PILZ_PIPELINE = "pilz_industrial_motion_planner";
}

PilzSequenceBackend::PilzSequenceBackend(rclcpp::Node::SharedPtr node, std::string group,
                                         std::string tip_link, std::string base_frame)
: node_(std::move(node)), group_(std::move(group)), tip_(std::move(tip_link)),
  base_(std::move(base_frame))
{
  client_ = rclcpp_action::create_client<MoveGroupSequence>(node_, "sequence_move_group");
}

moveit_msgs::msg::MotionPlanRequest PilzSequenceBackend::buildRequest(
  const ProcessSegment& seg, const MotionOptions& options) const
{
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = group_;
  req.pipeline_id = PILZ_PIPELINE;
  req.planner_id = (seg.type == ProcessSegmentType::CIRCULAR) ? "CIRC" : "LIN";
  req.num_planning_attempts = std::max(1, options.planning_attempts);
  req.allowed_planning_time = options.planning_time;
  req.max_velocity_scaling_factor = seg.velocity_scaling;
  req.max_acceleration_scaling_factor = seg.acceleration_scaling;

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = base_;
  goal.pose = seg.target_pose;
  req.goal_constraints = {kinematic_constraints::constructGoalConstraints(
    tip_, goal, options.goal_position_tolerance, options.goal_orientation_tolerance)};

  // CIRC needs an auxiliary point (interim/via or center), carried as a named
  // position constraint on the tip link.
  if (seg.type == ProcessSegmentType::CIRCULAR &&
      (seg.interim_point.has_value() || seg.circle_center.has_value()))
  {
    moveit_msgs::msg::Constraints path;
    path.name = seg.circle_center.has_value() ? "center" : "interim";
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = base_;
    pc.link_name = tip_;
    pc.weight = 1.0;
    geometry_msgs::msg::Pose aux;
    aux.orientation.w = 1.0;
    aux.position = seg.circle_center.value_or(seg.interim_point.value());
    pc.constraint_region.primitive_poses.push_back(aux);
    path.position_constraints.push_back(pc);
    req.path_constraints = path;
  }

  return req;
}

MotionResult PilzSequenceBackend::run(const std::vector<ProcessSegment>& segments,
                                      const MotionOptions& options)
{
  if (segments.empty())
    return MotionResult::failure("pilz_sequence: empty segment list");

  if (!client_->wait_for_action_server(5s))
    return MotionResult::failure("pilz_sequence: 'sequence_move_group' action server not available");

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
    return MotionResult::failure("pilz_sequence: goal send timed out");
  auto handle = goal_future.get();
  if (!handle)
    return MotionResult::failure("pilz_sequence: goal rejected");

  auto result_future = client_->async_get_result(handle);
  if (result_future.wait_for(120s) != std::future_status::ready)
    return MotionResult::failure("pilz_sequence: result timed out");

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
  mr.process.execution_success = ok && !options.plan_only;
  mr.stage = ok ? "pilz_sequence: OK" : "pilz_sequence: FAILED (code=" +
                  std::to_string(resp.error_code.val) + ")";
  RCLCPP_INFO(node_->get_logger(), "%s", mr.stage.c_str());
  return mr;
}

}  // namespace trainit
