// =============================================================================
// trajectory_validator.cpp
// =============================================================================
#include "trainit_motion_runtime/trajectory_validator.hpp"
#include "trainit_motion_runtime/motion_failure.hpp"

#include <cmath>
#include <vector>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trainit
{

TrajectoryValidator::TrajectoryValidator(moveit::core::RobotModelConstPtr robot_model,
                                         std::string group)
: robot_model_(std::move(robot_model)), group_(std::move(group))
{
}

ValidationReport TrajectoryValidator::validate(
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  const planning_scene::PlanningScenePtr& scene,
  double max_joint_jump) const
{
  ValidationReport r;

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_);
  if (jmg == nullptr)
  {
    r.reason = "unknown planning group '" + group_ + "'";
    return r;
  }

  // Convert to a RobotTrajectory for FK / limit / collision queries.
  robot_trajectory::RobotTrajectory rt(robot_model_, group_);
  moveit::core::RobotState ref(robot_model_);
  ref.setToDefaultValues();
  rt.setRobotTrajectoryMsg(ref, trajectory);

  const std::size_t n = rt.getWayPointCount();
  r.non_empty = (n > 0);
  if (!r.non_empty)
  {
    r.reason = "empty trajectory";
    return r;
  }

  // finite values + joint limits + joint step
  r.finite = true;
  r.within_joint_limits = true;
  std::vector<double> prev, cur;
  for (std::size_t i = 0; i < n; ++i)
  {
    const moveit::core::RobotState& s = rt.getWayPoint(i);
    s.copyJointGroupPositions(jmg, cur);
    for (double v : cur)
    {
      if (!std::isfinite(v))
      {
        r.finite = false;
        r.reason = "non-finite joint value at waypoint " + std::to_string(i);
        return r;
      }
    }
    if (!s.satisfiesBounds(jmg))
    {
      r.within_joint_limits = false;
      r.reason = "joint limits violated at waypoint " + std::to_string(i);
      return r;
    }
    if (i > 0)
    {
      double step = 0.0;
      for (std::size_t j = 0; j < cur.size(); ++j)
        step = std::max(step, std::abs(cur[j] - prev[j]));
      r.max_joint_step = std::max(r.max_joint_step, step);
    }
    prev = cur;
  }
  if (r.max_joint_step > max_joint_jump)
  {
    r.joint_jump = true;
    r.reason = "joint jump " + std::to_string(r.max_joint_step) + " > " +
               std::to_string(max_joint_jump);
    return r;
  }

  // monotonic timestamps
  r.timestamps_monotonic = true;
  double last_t = -1.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double t = rt.getWayPointDurationFromStart(i);
    if (t < last_t - 1e-9)
    {
      r.timestamps_monotonic = false;
      r.reason = "non-monotonic timestamp at waypoint " + std::to_string(i);
      return r;
    }
    last_t = t;
  }

  // collision (only if a scene is available)
  if (scene)
  {
    std::vector<std::size_t> invalid;
    r.collision_checked = true;
    r.collision_free = scene->isPathValid(rt, group_, false, &invalid);
    if (!r.collision_free)
    {
      if (!invalid.empty()) r.first_colliding_waypoint = invalid.front();
      r.reason = "collision at waypoint(s) starting " +
                 (invalid.empty() ? std::string("?") : std::to_string(invalid.front()));
      return r;
    }
  }

  r.valid = true;
  r.reason = "ok";
  return r;
}

// vNext M2.7: the cause token of a refused trajectory, in validate()'s own order (trajectory_validator.hpp).
const char* validationCause(const ValidationReport& vr)
{
  if (vr.reason.rfind("unknown planning group", 0) == 0) return kCauseUnknownGroup;
  if (!vr.non_empty) return kCauseEmptyTrajectory;
  if (!vr.finite) return kCauseNonFinite;
  if (!vr.within_joint_limits) return kCauseJointLimits;
  if (vr.joint_jump) return kCauseJointJump;
  if (!vr.timestamps_monotonic) return kCauseNonMonotonicTime;
  // Waypoint 0 is the start. MoveIt plans from a colliding start without refusing it: move_group
  // accepts a path whose only invalid waypoint is 0, Pilz never checks the start for collision, and
  // FixStartStateCollision puts the colliding start back as waypoint 0. So every target fails alike.
  if (vr.collision_checked && !vr.collision_free && vr.first_colliding_waypoint)
    return *vr.first_colliding_waypoint == 0 ? kCauseStartStateCollision : kCauseCollision;
  return kCauseValidationFailed;
}

}  // namespace trainit
