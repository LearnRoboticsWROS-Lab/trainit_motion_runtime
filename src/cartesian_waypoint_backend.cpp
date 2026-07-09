// =============================================================================
// cartesian_waypoint_backend.cpp
// =============================================================================
#include "trainit_motion_runtime/cartesian_waypoint_backend.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

namespace trainit
{

namespace
{
double pointToSegment(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1e-12)
    return (p - a).norm();
  double t = (p - a).dot(ab) / len2;
  t = std::clamp(t, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}
}  // namespace

CartesianWaypointBackend::CartesianWaypointBackend(
  rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& arm,
  moveit::core::RobotModelConstPtr robot_model, std::string group, std::string tip_link,
  std::string base_frame)
: node_(std::move(node)), arm_(arm), robot_model_(std::move(robot_model)),
  group_(std::move(group)), tip_(std::move(tip_link)), base_(std::move(base_frame))
{
}

MotionResult CartesianWaypointBackend::run(const CartesianProcessPath& path,
                                           const MotionOptions& options,
                                           double fraction_threshold, bool allow_partial)
{
  MotionResult mr;
  mr.process.process_path_backend = "cartesian_waypoints";
  mr.process.number_of_input_waypoints = path.waypoints.size();

  if (path.waypoints.size() < 2)
    return MotionResult::failure("cartesian_waypoints: need at least 2 waypoints");

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(path.waypoints.size());
  for (const auto& wp : path.waypoints)
    poses.push_back(wp.pose);

  const std::string tip = path.tcp_link.empty() ? tip_ : path.tcp_link;
  arm_.setEndEffectorLink(tip);
  arm_.setPoseReferenceFrame(path.reference_frame.empty() ? base_ : path.reference_frame);

  moveit_msgs::msg::RobotTrajectory traj_msg;
  moveit_msgs::msg::MoveItErrorCodes ec;
  const double eef_step = path.waypoint_resolution > 0.0 ? path.waypoint_resolution : 0.005;
  const double fraction = arm_.computeCartesianPath(
    poses, eef_step, 0.0, traj_msg, path.collision_checking_enabled, &ec);

  mr.process.cartesian_fraction = fraction;
  RCLCPP_INFO(node_->get_logger(), "cartesian_waypoints: fraction=%.3f (err=%d)", fraction, ec.val);

  if (fraction < fraction_threshold && !allow_partial)
    return MotionResult::failure(
      "cartesian_waypoints: incomplete path fraction=" + std::to_string(fraction), ec.val);

  // Wrap + time-parameterize (computeCartesianPath returns zero timing).
  robot_trajectory::RobotTrajectory rt(robot_model_, group_);
  moveit::core::RobotStatePtr start = arm_.getCurrentState(2.0);
  if (!start)
  {
    start = std::make_shared<moveit::core::RobotState>(robot_model_);
    start->setToDefaultValues();
  }
  rt.setRobotTrajectoryMsg(*start, traj_msg);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, options.velocity_scaling, options.acceleration_scaling))
    return MotionResult::failure("cartesian_waypoints: TOTG time-parameterization failed");
  if (options.smooth_with_ruckig)
    trajectory_processing::RuckigSmoothing::applySmoothing(
      rt, options.velocity_scaling, options.acceleration_scaling);

  // --- FK metrics (Section 21) ---------------------------------------------
  const std::size_t n = rt.getWayPointCount();
  mr.process.number_of_output_waypoints = n;
  mr.process.trajectory_duration = (n > 0) ? rt.getWayPointDurationFromStart(n - 1) : 0.0;

  std::vector<Eigen::Vector3d> ideal;  // input waypoint TCP positions (the ideal polyline)
  for (const auto& p : poses)
    ideal.emplace_back(p.position.x, p.position.y, p.position.z);

  Eigen::Vector3d prev_pos;
  double prev_t = 0.0, prev_speed = 0.0;
  double length = 0.0, max_speed = 0.0, max_acc = 0.0, max_dev = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const Eigen::Vector3d pos = rt.getWayPoint(i).getGlobalLinkTransform(tip).translation();
    const double t = rt.getWayPointDurationFromStart(i);
    double min_dev = std::numeric_limits<double>::max();
    for (std::size_t k = 1; k < ideal.size(); ++k)
      min_dev = std::min(min_dev, pointToSegment(pos, ideal[k - 1], ideal[k]));
    max_dev = std::max(max_dev, min_dev);
    if (i > 0)
    {
      const double d = (pos - prev_pos).norm();
      length += d;
      const double dt = t - prev_t;
      const double speed = (dt > 1e-6) ? d / dt : 0.0;
      max_speed = std::max(max_speed, speed);
      if (dt > 1e-6)
        max_acc = std::max(max_acc, std::abs(speed - prev_speed) / dt);
      prev_speed = speed;
    }
    prev_pos = pos;
    prev_t = t;
  }
  mr.process.cartesian_path_length = length;
  mr.process.maximum_tcp_speed = max_speed;
  mr.process.maximum_tcp_acceleration = max_acc;
  mr.process.maximum_tcp_line_deviation = max_dev;
  mr.process.average_tcp_speed =
    (mr.process.trajectory_duration > 1e-6) ? length / mr.process.trajectory_duration : 0.0;

  RCLCPP_INFO(node_->get_logger(),
              "cartesian_waypoints metrics: out_wp=%zu dur=%.2fs len=%.3fm avg_v=%.3f max_v=%.3f "
              "max_dev=%.4fm",
              n, mr.process.trajectory_duration, length, mr.process.average_tcp_speed,
              max_speed, max_dev);

  if (options.plan_only)
  {
    mr.success = true;
    mr.stage = "cartesian_waypoints: PLANNED (plan_only)";
    return mr;
  }

  moveit_msgs::msg::RobotTrajectory timed;
  rt.getRobotTrajectoryMsg(timed);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = timed;
  const bool ok = static_cast<bool>(arm_.execute(plan));
  mr.success = ok;
  mr.process.execution_success = ok;
  mr.stage = ok ? "cartesian_waypoints: OK" : "cartesian_waypoints: EXEC FAILED";
  return mr;
}

}  // namespace trainit
