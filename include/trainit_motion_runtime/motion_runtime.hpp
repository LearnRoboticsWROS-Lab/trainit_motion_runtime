// =============================================================================
// motion_runtime.hpp — robot-agnostic semantic motion API (Section 13).
//
// The planned-motion door (vNext M2.5): the BT nodes and TaskRunner reach MoveIt only through
// this interface, and since vNext M2.7 they read the frames and the named-state FK through it too
// (BtContext::runtime and TaskRunner are typed MotionRuntime). The planning scene they WRITE is
// not motion: it is the runner's PlanningSceneManager, handed over beside the door
// (docs/trainit_motion_runtime_architecture.md). It must NOT contain MoveIt details such as
// setPlanningPipelineId(), setPlannerId() or computeCartesianPath(); those live inside the backends.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

// vNext M3.4 (R-M3-20, owner D3): a robot-state read that has no fresh state to read. Thrown by
// MotionRuntime::currentTipPose(); a node that reads the state turns it into a FAILURE that names
// this reason. Never answered with a pose: the identity pose it returned before is a VALID pose.
struct RobotStateUnavailable : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

class MotionRuntime
{
public:
  virtual ~MotionRuntime() = default;

  // Discrete motions ---------------------------------------------------------
  virtual MotionResult movePtp(const JointTarget& target, const MotionOptions& options) = 0;
  virtual MotionResult moveLinear(const CartesianTarget& target, const MotionOptions& options) = 0;
  virtual MotionResult moveCircular(const CircularTarget& target, const MotionOptions& options) = 0;

  // Free-space motion (planner mode decides OMPL / OMPL+CHOMP / STOMP / Pilz PTP)
  virtual MotionResult moveCollisionFree(const MotionTarget& target, const PlanningOptions& options) = 0;

  // Process paths ------------------------------------------------------------
  virtual MotionResult executeProcessSequence(const std::vector<ProcessSegment>& segments,
                                              const MotionOptions& options) = 0;
  virtual MotionResult executeCartesianProcess(const CartesianProcessPath& path,
                                               const MotionOptions& options) = 0;

  // Raw execution + lifecycle ------------------------------------------------
  virtual MotionResult executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) = 0;
  virtual MotionResult stop() = 0;

  // Strategy selection -------------------------------------------------------
  virtual void setPlannerMode(PlannerMode mode) = 0;
  virtual PlannerMode plannerMode() const = 0;

  // Robot state (vNext M2.5: the one read the BT nodes need, behind the door) --
  // The CURRENT pose of the runtime's configured tip link, stamped in the robot model frame
  // (the URDF root, which may differ from the base frame poses are given in). Throws
  // RobotStateUnavailable when there is no fresh robot state to read (vNext M3.4).
  virtual geometry_msgs::msg::PoseStamped currentTipPose() const = 0;

  // Frames and named states (vNext M2.7, owner decision 11.2: the reads the BT nodes and TaskRunner
  // made on the concrete runtime, now behind the door) --------------------------------------------
  // The configured tip link: the TCP a move targets when it names none.
  virtual const std::string& tipLink() const = 0;
  // The configured base frame: the frame target poses and generated paths are given in.
  virtual const std::string& baseFrame() const = 0;
  // FK: the tip-link pose of a NAMED group state (a waypoint captured as joints), std::nullopt if
  // the state is unknown. The pose is in the robot model frame (KNOWN_DEBT: a LIN/CIRC goal sends it
  // as a base-frame pose).
  virtual std::optional<geometry_msgs::msg::Pose> namedPose(const std::string& named) const = 0;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_
