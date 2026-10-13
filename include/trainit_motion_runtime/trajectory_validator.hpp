// =============================================================================
// trajectory_validator.hpp — real pre-execution trajectory checks (Section 22).
//
// NEVER returns a blanket true. Checks: non-empty, finite values, monotonic
// timestamps, joint limits, joint jumps, and (when a scene is supplied) full
// collision validation via PlanningScene::isPathValid. TCP/orientation deviation
// for process paths is computed by the process backends (FK), not here.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__TRAJECTORY_VALIDATOR_HPP_
#define TRAINIT_MOTION_RUNTIME__TRAJECTORY_VALIDATOR_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace trainit
{

struct ValidationReport
{
  bool valid{false};
  bool non_empty{false};
  bool finite{false};
  bool timestamps_monotonic{false};
  bool within_joint_limits{false};
  bool collision_checked{false};
  bool collision_free{true};
  double max_joint_step{0.0};        // [rad]
  bool   joint_jump{false};
  std::string reason;                // first failing check
  // vNext M2.7: the first waypoint the collision check refused (the first index
  // PlanningScene::isPathValid names). Set only when that check ran and failed.
  // Waypoint 0 is the trajectory's start: the robot's current state.
  std::optional<std::size_t> first_colliding_waypoint;
};

// vNext M2.7 (owner Q8.2): which check refused the trajectory, as a motion_failure.hpp
// cause token, in validate()'s own order: UNKNOWN_GROUP, EMPTY_TRAJECTORY, NON_FINITE,
// JOINT_LIMITS, JOINT_JUMP, NON_MONOTONIC_TIME, then the collision check:
// START_STATE_COLLISION when the first colliding waypoint is 0 (the start, the robot's
// current state: it would recur for every target), COLLISION when it is a later one
// (this path). VALIDATION_FAILED for any other combination.
const char* validationCause(const ValidationReport& report);

class TrajectoryValidator
{
public:
  TrajectoryValidator(moveit::core::RobotModelConstPtr robot_model, std::string group);

  // scene may be null (collision check skipped, logged in report).
  ValidationReport validate(const moveit_msgs::msg::RobotTrajectory& trajectory,
                            const planning_scene::PlanningScenePtr& scene,
                            double max_joint_jump = 0.5) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__TRAJECTORY_VALIDATOR_HPP_
