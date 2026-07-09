// =============================================================================
// motion_runtime.hpp — robot-agnostic semantic motion API (Section 13).
//
// The application (and the future Behavior Tree) talks ONLY to this interface.
// It must NOT contain MoveIt details such as setPlanningPipelineId(),
// setPlannerId() or computeCartesianPath(); those live inside the backends.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_

#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

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
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_RUNTIME_HPP_
