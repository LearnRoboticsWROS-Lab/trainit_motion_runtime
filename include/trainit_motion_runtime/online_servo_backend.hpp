// =============================================================================
// online_servo_backend.hpp — ONLINE motion backend (Section 11). DESIGN ONLY.
//
// SUPERSEDED by ADR-0015 Decision 5 — do not implement this interface as written.
// moveit_servo 2.5.10 on Humble is a NODE, not a library: ServoNode is driven
// through ROS only (four std_srvs/Trigger services, a TwistStamped input, an Int8
// status output), and the stop() declared below has no counterpart in
// moveit_servo::Servo, which offers start() and setPaused(bool). Online servoing
// is therefore the `ServoTrack` BT node (src/bt_nodes.cpp): a service client plus a
// publisher, which keeps moveit_servo off the Community library's link line.
// This header is kept only as the record of the shape that was rejected; nothing
// includes it and it is not in the CMake sources.
//
// For visual servoing, online corrections, dynamic targets, cartesian jog,
// tracking and teleoperation. This is NOT an offline path planner — do not use
// it for offline geometric paths.
//
// Intended future implementation: MoveIt Servo (moveit_servo::Servo, installed).
//   * Input    : geometry_msgs/TwistStamped (cartesian) or control_msgs/JointJog,
//                streamed at a fixed command rate (e.g. 100-250 Hz).
//   * Output   : JointTrajectory / Float64MultiArray to the controller.
//   * Safety   : collision-proximity slowdown, singularity scaling, command
//                timeout (halt on stale input), pause/stop.
//
// Declared as an interface so the runtime is prepared for it; NOT implemented in
// this iteration (multi-planner + process path must be stable first).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__ONLINE_SERVO_BACKEND_HPP_
#define TRAINIT_MOTION_RUNTIME__ONLINE_SERVO_BACKEND_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace trainit
{

class OnlineServoBackend
{
public:
  virtual ~OnlineServoBackend() = default;

  virtual bool start() = 0;             // begin streaming loop
  virtual bool stop() = 0;              // halt and release the controller
  virtual void sendTwist(const geometry_msgs::msg::TwistStamped& twist) = 0;
  // sendJointJog(...) intentionally omitted until implementation.
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__ONLINE_SERVO_BACKEND_HPP_
