// =============================================================================
// bt_nodes.hpp — generic BehaviorTree.CPP v4 nodes wrapping the TrainIt runtime.
//
// All nodes are SyncActionNode. They fetch a BtContext* from the blackboard and
// call MotionRuntime / GripperController / ProcessController / PlanningSceneManager.
// Pose values travel on the blackboard as geometry_msgs::msg::Pose (the KEY is a
// string port; the VALUE is type-erased). Application-agnostic: no poses here.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__BT_NODES_HPP_
#define TRAINIT_MOTION_RUNTIME__BT_NODES_HPP_

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace trainit
{

// Declares a SyncActionNode with the standard ctor + providedPorts() + tick().
#define TRAINIT_BT_NODE(Name)                                              \
  class Name : public BT::SyncActionNode                                   \
  {                                                                        \
  public:                                                                  \
    Name(const std::string& name, const BT::NodeConfig& cfg)               \
    : BT::SyncActionNode(name, cfg) {}                                      \
    static BT::PortsList providedPorts();                                  \
    BT::NodeStatus tick() override;                                        \
  };

// --- motion (wrap MotionRuntime) ---
TRAINIT_BT_NODE(MoveWaypoint)          // data-driven: reads the full waypoint spec from bt_params
TRAINIT_BT_NODE(MovePtp)
TRAINIT_BT_NODE(MoveToNamed)
TRAINIT_BT_NODE(MoveToJoint)
TRAINIT_BT_NODE(MoveLinear)
TRAINIT_BT_NODE(MoveCircular)
TRAINIT_BT_NODE(MoveCollisionFree)
TRAINIT_BT_NODE(ExecuteProcessPath)

// --- end-effector ---
TRAINIT_BT_NODE(OpenGripper)
TRAINIT_BT_NODE(CloseGripper)
TRAINIT_BT_NODE(SetGripper)
TRAINIT_BT_NODE(EnableProcess)
TRAINIT_BT_NODE(DisableProcess)

// --- planning scene ---
TRAINIT_BT_NODE(AddCollisionObject)
TRAINIT_BT_NODE(RemoveCollisionObject)
TRAINIT_BT_NODE(AttachObject)
TRAINIT_BT_NODE(DetachObject)

// --- dynamic-object runtime flags (grasped-object planning + Isaac release) ---
TRAINIT_BT_NODE(SetAttachedCollisionCheck)  // toggle planning-collision check for held objects
TRAINIT_BT_NODE(SetReleasePolicy)           // tell Isaac freeze|gravity on the next release
TRAINIT_BT_NODE(ResetScene)                 // dynamic objects back to initial poses (sim loop)

// --- pose math (produce/consume a Pose blackboard entry) ---
TRAINIT_BT_NODE(MakePose)              // full TCP pose: position + orientation (all DOF)
TRAINIT_BT_NODE(StoreCurrentPose)
TRAINIT_BT_NODE(ComputeTcpTarget)
TRAINIT_BT_NODE(OffsetPoseInToolFrame)
TRAINIT_BT_NODE(OffsetPoseInBaseFrame)

// --- perception (consume the perception contract, write the blackboard) ---
// The runtime stays algorithm-agnostic: it reads vision_msgs/Detection3DArray from
// ANY detector and knows nothing about how the object was found. Because MoveWaypoint
// reads "<wp>.position" as a blackboard string, a detection written there at runtime
// drives the unchanged tree -- vision is a block, not a new application.
TRAINIT_BT_NODE(DetectObject)             // wait for a FRESH detection, TF it, write "<out_key>.*"
TRAINIT_BT_NODE(SetWaypointFromDetection) // "<from>.position" (+offset) -> "<waypoint>.position"
TRAINIT_BT_NODE(SetWaypointRelative)      // "<from>" pose (+offset, +rpy delta) -> "<waypoint>" (D-017)

// --- learned-policy execution (extension point; see docs POLICY_EXECUTION.md, ADR-0005) ---
// The Community core stays torch-free: these nodes only speak a ROS contract to the
// separate Pro package `trainit_policy_runtime`, which loads the policy and runs it.
// RunPolicy triggers a run in one of three modes (pure|hybrid|residual) and waits; in
// hybrid it returns once the policy has published its decided target on the Detection3D
// contract, so the EXISTING DetectObject + SetWaypointFromDetection + MoveWaypoint
// execute it smoothly (no policy-specific move node). CheckRobotState then asserts the
// policy card's end_state, so a mis-ending policy fails safe instead of corrupting the
// next step.
TRAINIT_BT_NODE(RunPolicy)         // call the policy runtime's run service, wait for its part
TRAINIT_BT_NODE(CheckRobotState)   // assert tcp near expected + suction/attached (the end_state)

// --- utility ---
TRAINIT_BT_NODE(Wait)
TRAINIT_BT_NODE(Log)

#undef TRAINIT_BT_NODE

// Register every node (plus the MoveFree alias for MoveCollisionFree).
void registerAllNodes(BT::BehaviorTreeFactory& factory);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__BT_NODES_HPP_
