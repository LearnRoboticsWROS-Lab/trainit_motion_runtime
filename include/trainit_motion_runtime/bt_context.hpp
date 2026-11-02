// =============================================================================
// bt_context.hpp — shared context injected on the BehaviorTree blackboard.
//
// Analogous to the reference framework's "RobotClient", but it wraps OUR
// MotionRuntime / GripperController / ProcessController (no MoveGroupInterface
// usage in the nodes). The runner owns the pointed-to objects for the lifetime
// of the tree; the nodes fetch a non-owning BtContext* by blackboard key.
// The nodes depend on the MotionRuntime INTERFACE (vNext M2.7, owner decision 11.2);
// the planning scene they write is the runner's PlanningSceneManager (`scene`).
//
// NOTE: trainit_motion_runtime does NOT depend on industrial_bt_framework. Only
// the patterns were copied; the only new dependency is behaviortree_cpp.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_
#define TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>

#include "trainit_motion_runtime/motion_runtime.hpp"
#include "trainit_motion_runtime/planning_scene_manager.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace trainit
{

class EndEffectorRegistry;   // end_effector_registry.hpp (ADR-0022)

// Blackboard keys.
inline constexpr const char* BB_BT_CONTEXT      = "trainit_context";
inline constexpr const char* BB_MOTION_PROFILES = "trainit_profiles";
inline constexpr const char* BB_ROS_NODE        = "ros_node";
// vNext M3.1 (ADR-0024): the runner's path library (std::shared_ptr<const PathLibrary>, from its
// `paths_file` parameter); absent when no paths file was given. FollowPath reads it.
inline constexpr const char* BB_PATH_LIBRARY    = "trainit_paths";

// Non-owning handles to the runtime stack, shared with every BT node.
struct BtContext
{
  MotionRuntime*        runtime{nullptr};   // the door: movePtp/moveLinear/.../tipLink/baseFrame/namedPose
  GripperController*    gripper{nullptr};   // may be null (no end-effector configured)
  ProcessController*    process{nullptr};   // may be null
  rclcpp::Node::SharedPtr node;             // logging / clock
  // TF, for nodes that consume data in a sensor frame (DetectObject). May be null
  // with an older launcher: the node then builds its own buffer lazily.
  std::shared_ptr<tf2_ros::Buffer> tf;

  // --- end-effector actuation (ADR-0010) -------------------------------------
  // The GripperCommand position Close/OpenGripper send. The defaults 1.0/0.0 are
  // exactly the historical suction on/off, so a project that leaves them unset
  // behaves byte-for-byte as before. A JOINT_POSITION gripper (e.g. a Robotiq)
  // overrides them with the closed/open joint angles.
  double gripper_close_pos{1.0};
  double gripper_open_pos{0.0};
  // --- sim grasp adapter signal (ADR-0010) -----------------------------------
  // LEGACY (ADR-0022, ADR-0023): the topic is used as the project writes it: an
  // absolute name is GLOBAL, shared by every robot. Never built in a canonical run.
  // When a gripper_cmd_topic is configured the runtime publishes a LATCHED Bool on
  // grasp (true) / release (false) that fires BOTH the planning-scene attach
  // (scene_manager_node) AND the active backend's sim grasp adapter (Isaac
  // SurfaceGripper / Gazebo LinkAttacher), on the same contract for every backend.
  // Null == not configured == the runtime publishes nothing (today's behaviour;
  // the adaptation-layer bridge drives the Bool instead).
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub;
  // --- the canonical end-effector contract (ADR-0022) --------------------------
  // Non-null iff the project carries an `eef` block: Close/OpenGripper then take their
  // canonical branch, and CommandActuator / VerifyGrasp work. Null (the default) == the
  // LEGACY contract: every legacy path runs exactly as before, and `gripper` /
  // `gripper_cmd_pub` above are never built in a canonical run. LAST, so the positional
  // initialisation `BtContext ctx{runtime, gripper, process, node}` still compiles.
  EndEffectorRegistry* eefs{nullptr};
  // --- the planning scene (vNext M2.7, owner decision 11.2) ------------------------------------
  // What AddCollisionObject / RemoveCollisionObject / AttachObject / DetachObject write: the
  // runner's PlanningSceneManager, i.e. &scene() of the MoveItMotionRuntime it built (until 2.0.0
  // the nodes reached it as runtime->scene()). Every composition root that sets `runtime` sets this
  // too; a scene node does not check it (like the motion nodes do not check `runtime`). LAST, so
  // the positional initialisation `BtContext ctx{runtime, gripper, process, node}` still compiles.
  PlanningSceneManager* scene{nullptr};
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__BT_CONTEXT_HPP_
