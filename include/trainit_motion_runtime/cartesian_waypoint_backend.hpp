// =============================================================================
// cartesian_waypoint_backend.hpp — dense cartesian process path via
// computeCartesianPath + TOTG/Ruckig retiming.
//
// Secondary backend (KDL cartesian interpolation is fragile): prefer the Pilz
// sequence for production straight lines. Accepts the path only if the achieved
// fraction >= threshold (default 1.0) unless partial paths are explicitly
// allowed. Collects FK-based deviation metrics (Section 21).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__CARTESIAN_WAYPOINT_BACKEND_HPP_
#define TRAINIT_MOTION_RUNTIME__CARTESIAN_WAYPOINT_BACKEND_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>

#include "trainit_motion_runtime/process_path_backend.hpp"
#include "trainit_motion_runtime/motion_task.hpp"

namespace trainit
{

class CartesianWaypointBackend : public ProcessPathBackendBase
{
public:
  CartesianWaypointBackend(rclcpp::Node::SharedPtr node,
                           moveit::planning_interface::MoveGroupInterface& arm,
                           moveit::core::RobotModelConstPtr robot_model,
                           std::string group, std::string tip_link, std::string base_frame);

  std::string name() const override { return "cartesian_waypoints"; }
  bool supportsCircular() const override { return false; }  // approximated by dense sampling

  MotionResult run(const CartesianProcessPath& path, const MotionOptions& options,
                   double fraction_threshold = 1.0, bool allow_partial = false);

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface& arm_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_, tip_, base_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__CARTESIAN_WAYPOINT_BACKEND_HPP_
