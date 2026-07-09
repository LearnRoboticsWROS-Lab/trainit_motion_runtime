// =============================================================================
// planning_scene_manager.cpp
// =============================================================================
#include "trainit_motion_runtime/planning_scene_manager.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace trainit
{

PlanningSceneManager::PlanningSceneManager(rclcpp::Node::SharedPtr node,
                                           std::string robot_description)
: node_(std::move(node))
{
  // The monitor is best-effort: if it fails to initialise (e.g. robot_description
  // not on this node), validation degrades to the model-only checks and we log it.
  try
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, robot_description);
    if (psm_->getPlanningScene())
    {
      psm_->startSceneMonitor();
      psm_->startWorldGeometryMonitor();
      psm_->startStateMonitor();
      psm_->requestPlanningSceneState();
      RCLCPP_INFO(node_->get_logger(), "PlanningSceneMonitor ready (collision validation enabled)");
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
                  "PlanningSceneMonitor has no scene; collision validation disabled");
      psm_.reset();
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(node_->get_logger(), "PlanningSceneMonitor init failed (%s); "
                "collision validation disabled", e.what());
    psm_.reset();
  }
}

static moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id, const std::string& frame_id,
  const std::array<double, 3>& dims, const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame_id;
  obj.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {dims[0], dims[1], dims[2]};
  obj.primitives = {box};
  obj.primitive_poses = {pose};
  obj.operation = obj.ADD;
  return obj;
}

void PlanningSceneManager::addCollisionBox(const std::string& id, const std::string& frame_id,
                                           const std::array<double, 3>& dims,
                                           const geometry_msgs::msg::Pose& pose)
{
  psi_.applyCollisionObject(makeBox(id, frame_id, dims, pose));
  RCLCPP_INFO(node_->get_logger(), "added collision box '%s'", id.c_str());
}

void PlanningSceneManager::removeCollisionObject(const std::string& id)
{
  psi_.removeCollisionObjects({id});
}

void PlanningSceneManager::attachBox(const std::string& id, const std::string& link,
                                     const std::array<double, 3>& dims,
                                     const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object = makeBox(id, link, dims, pose);
  aco.object.operation = aco.object.ADD;
  psi_.applyAttachedCollisionObject(aco);
  RCLCPP_INFO(node_->get_logger(), "attached '%s' to '%s'", id.c_str(), link.c_str());
}

void PlanningSceneManager::detachObject(const std::string& id, const std::string& link)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object.id = id;
  aco.object.operation = aco.object.REMOVE;
  psi_.applyAttachedCollisionObject(aco);
  psi_.removeCollisionObjects({id});
  RCLCPP_INFO(node_->get_logger(), "detached '%s'", id.c_str());
}

planning_scene::PlanningScenePtr PlanningSceneManager::snapshotForValidation()
{
  if (!psm_)
    return nullptr;
  psm_->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
  return planning_scene::PlanningScene::clone(ls);
}

}  // namespace trainit
