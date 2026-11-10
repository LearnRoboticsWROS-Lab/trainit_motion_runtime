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
: node_(std::move(node)),
  // vNext M2.7 (namespace fix): the PlanningSceneInterface's own node joins THIS node's namespace
  // and resolves get/apply_planning_scene and planning_scene there. Default-constructed (ns "") it
  // took the root unless a process-wide __ns remap moved it; "" and "/" are the same root and the
  // remap applies to both, so a root or a launch-remapped runtime is unchanged.
  psi_(node_->get_namespace())
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

bool PlanningSceneManager::attachBox(const std::string& id, const std::string& link,
                                     const std::array<double, 3>& dims,
                                     const geometry_msgs::msg::Pose& pose,
                                     const std::vector<std::string>& touch_links)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object = makeBox(id, link, dims, pose);
  aco.object.operation = aco.object.ADD;
  aco.touch_links = touch_links;
  if (!psi_.applyAttachedCollisionObject(aco))
  {
    RCLCPP_ERROR(node_->get_logger(), "attaching '%s' to '%s': move_group did not apply it (apply_planning_scene "
                 "answered failure): the planning scene does NOT hold the object", id.c_str(), link.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "attached '%s' to '%s'", id.c_str(), link.c_str());
  return true;
}

bool PlanningSceneManager::detachObject(const std::string& id, const std::string& link)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object.id = id;
  aco.object.operation = aco.object.REMOVE;
  const bool applied = psi_.applyAttachedCollisionObject(aco);
  psi_.removeCollisionObjects({id});
  if (!applied)
  {
    RCLCPP_ERROR(node_->get_logger(), "detaching '%s' from '%s': move_group did not apply it (apply_planning_scene "
                 "answered failure): the object may still be attached", id.c_str(), link.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "detached '%s'", id.c_str());
  return true;
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
