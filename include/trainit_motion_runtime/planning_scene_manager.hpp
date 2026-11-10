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
#include <vector>

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
  // TMR 2.8.1 (M4.5 review): both return what move_group's apply_planning_scene answered (false = the
  // scene was NOT updated: e.g. an unknown link, a refused or unanswered request). `touch_links`: the robot
  // links the attached object may touch (AttachedCollisionObject::touch_links); empty = none, the message
  // MoveIt received before 2.8.1. A detach of an object that is not attached is answered true by
  // move_group (PlanningScene::processAttachedCollisionObjectMsg REMOVE: nothing to remove), so it is
  // idempotent; its world removal is a planning_scene TOPIC (no answer to read).
  bool attachBox(const std::string& id, const std::string& link,
                 const std::array<double, 3>& dims, const geometry_msgs::msg::Pose& pose,
                 const std::vector<std::string>& touch_links = {});
  bool detachObject(const std::string& id, const std::string& link);

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
