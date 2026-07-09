// =============================================================================
// planning_scene_manager.hpp — world collision objects, attach/detach, and a
// planning-scene snapshot for trajectory validation.
//
// MoveIt plans "blind" against its planning scene: external fixtures (tables,
// bins, the simulated cell) are invisible unless added here. Generic box helpers
// plus, when a PlanningSceneMonitor is available, a scene snapshot so the
// TrajectoryValidator can run a real collision check.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PLANNING_SCENE_MANAGER_HPP_
#define TRAINIT_MOTION_RUNTIME__PLANNING_SCENE_MANAGER_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

namespace trainit
{

class PlanningSceneManager
{
public:
  explicit PlanningSceneManager(rclcpp::Node::SharedPtr node,
                                std::string robot_description = "robot_description");

  // --- world collision objects ----------------------------------------------
  void addCollisionBox(const std::string& id, const std::string& frame_id,
                       const std::array<double, 3>& dims, const geometry_msgs::msg::Pose& pose);
  void removeCollisionObject(const std::string& id);

  // --- attach/detach to a link (grasped object) -----------------------------
  void attachBox(const std::string& id, const std::string& link,
                 const std::array<double, 3>& dims, const geometry_msgs::msg::Pose& pose);
  void detachObject(const std::string& id, const std::string& link);

  // --- validation scene snapshot (nullptr if no monitor) --------------------
  planning_scene::PlanningScenePtr snapshotForValidation();
  bool hasMonitor() const { return static_cast<bool>(psm_); }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PLANNING_SCENE_MANAGER_HPP_
