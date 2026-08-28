// =============================================================================
// bt_context.hpp — shared context injected on the BehaviorTree blackboard.
//
// Analogous to the reference framework's "RobotClient", but it wraps OUR
// MotionRuntime / GripperController / ProcessController (no MoveGroupInterface
// usage in the nodes). The runner owns the pointed-to objects for the lifetime
// of the tree; the nodes fetch a non-owning BtContext* by blackboard key.
//
// NOTE: trainit_motion_runtime does NOT depend on industrial_bt_framework. Only
// the patterns were copied; the only new dependency is behaviortree_cpp.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_
#define TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace trainit
{

// Blackboard keys.
inline constexpr const char* BB_BT_CONTEXT      = "trainit_context";
inline constexpr const char* BB_MOTION_PROFILES = "trainit_profiles";
inline constexpr const char* BB_ROS_NODE        = "ros_node";

// Non-owning handles to the runtime stack, shared with every BT node.
struct BtContext
{
  MoveItMotionRuntime* runtime{nullptr};   // movePtp/moveLinear/moveCircular/moveCollisionFree/...
  GripperController*    gripper{nullptr};   // may be null (no end-effector configured)
  ProcessController*    process{nullptr};   // may be null
  rclcpp::Node::SharedPtr node;             // logging / clock
  // TF, for nodes that consume data in a sensor frame (DetectObject). May be null
  // with an older launcher: the node then builds its own buffer lazily.
  std::shared_ptr<tf2_ros::Buffer> tf;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_
