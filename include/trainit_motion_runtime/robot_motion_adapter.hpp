// =============================================================================
// robot_motion_adapter.hpp — future execution backend abstraction (Section 27).
//
// Confines hardware specificity. Two future implementation families:
//   * GenericRosMotionAdapter   : RobotTrajectory -> FollowJointTrajectory (ros2_control)
//   * VendorNativeMotionAdapter : maps PTP/LINEAR/CIRCULAR to the controller's
//                                 native joint/linear/circular moves and streams
//                                 trajectories via its servo interface
//
// SPEED contract (the application "Set_Speed(80)"): MotionOptions.velocity_scaling
// is the speed FRACTION in [0,1]; speed_percent = velocity_scaling * 100. Each
// adapter translates it:
//   * GenericRos : velocity_scaling -> MoveGroup time-parameterization; the speed
//     is baked into the trajectory timestamps and replayed by the controller.
//   * VendorNative : velocity_scaling * 100 -> the SDK speed argument
//     (e.g. Fairino MoveJ(speed=%), MoveL(speed=%)); the robot regulates the
//     real velocity natively. acceleration_scaling maps the same way.
// So the SAME per-move speed value drives both sim and the real robot SDK.
//
// Not implemented in this iteration — declared so the runtime is designed for it.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__ROBOT_MOTION_ADAPTER_HPP_
#define TRAINIT_MOTION_RUNTIME__ROBOT_MOTION_ADAPTER_HPP_

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

class RobotMotionAdapter
{
public:
  virtual ~RobotMotionAdapter() = default;

  virtual MotionResult movePtp(const JointTarget& target, const MotionOptions& options) = 0;
  virtual MotionResult moveLinear(const CartesianTarget& target, const MotionOptions& options) = 0;
  virtual MotionResult moveCircular(const CircularTarget& target, const MotionOptions& options) = 0;
  virtual MotionResult executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) = 0;
  virtual MotionResult stop() = 0;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__ROBOT_MOTION_ADAPTER_HPP_
