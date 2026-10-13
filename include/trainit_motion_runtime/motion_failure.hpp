// =============================================================================
// motion_failure.hpp -- whose failure a failed motion is (vNext M2.7; owner Q8.2; ADR-0015 and
// ADR-0022 amendments M2.7). ONE table for every consumer:
//   TARGET  this target cannot be reached by this motion (no IK, no valid plan, the goal in
//           collision, a joint jump in the plan): another target may work.
//   FAULT   anything that would recur for every target -- the motion infrastructure (a missing
//           server, a rejected goal, a transport timeout, any execution failure), the robot or
//           scene state (the START state), the configuration, an exception, a value MoveIt does
//           not name: never "try the next candidate", never "bin empty".
// Default-deny: a failure is a FAULT unless the table names it TARGET. By phase (MotionPhase,
// recorded by the runtime on the failure paths MoveWaypoint reaches):
//   PLAN              TARGET for PLANNING_FAILED, INVALID_MOTION_PLAN, GOAL_IN_COLLISION,
//                     GOAL_VIOLATES_PATH_CONSTRAINTS, GOAL_CONSTRAINTS_VIOLATED, GOAL_STATE_INVALID,
//                     NO_IK_SOLUTION and TIMED_OUT (the planner's own budget for THIS goal)
//   PLAN_AND_EXECUTE  the same, but TIMED_OUT is a FAULT (Pilz planners never return it: it is the
//                     execution's)
//   VALIDATE          TARGET only for the causes JOINT_JUMP and COLLISION (this path: a collision
//                     at a waypoint after the start); START_STATE_COLLISION (a collision at
//                     waypoint 0, the robot's current state) is a FAULT
//   EXECUTE, REQUEST, NONE   always a FAULT
// A recorded cause wins over the error code: only the VALIDATE causes JOINT_JUMP and COLLISION are
// TARGET; every other cause (a Pilz transport outcome, a request refusal, an exception) is a FAULT.
// Pure: no ROS, no MoveIt C++ API, no function-static, no absolute name, and NO direct
// `#include <moveit...>` (the error codes arrive through motion_types.hpp).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_FAILURE_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_FAILURE_HPP_

#include <string>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

enum class MotionFailureClass { NONE, TARGET, FAULT };

// The causes the runtime records when the MoveIt error code is not the cause.
inline constexpr const char* kCauseServerUnavailable = "SERVER_UNAVAILABLE";   // Pilz: no server in 5 s
inline constexpr const char* kCauseGoalSendTimeout   = "GOAL_SEND_TIMEOUT";    // Pilz: no goal response in 10 s
inline constexpr const char* kCauseGoalRejected      = "GOAL_REJECTED";        // Pilz: the goal was rejected
inline constexpr const char* kCauseResultTimeout     = "RESULT_TIMEOUT";       // Pilz: no result in 120 s
// ... which check of the TrajectoryValidator refused the plan (VALIDATE)
inline constexpr const char* kCauseEmptyTrajectory   = "EMPTY_TRAJECTORY";
inline constexpr const char* kCauseNonFinite         = "NON_FINITE";
inline constexpr const char* kCauseJointLimits       = "JOINT_LIMITS";
inline constexpr const char* kCauseJointJump         = "JOINT_JUMP";           // TARGET
inline constexpr const char* kCauseNonMonotonicTime  = "NON_MONOTONIC_TIME";
inline constexpr const char* kCauseCollision         = "COLLISION";            // TARGET: at a waypoint k > 0
inline constexpr const char* kCauseStartStateCollision = "START_STATE_COLLISION";  // FAULT: at waypoint 0
inline constexpr const char* kCauseUnknownGroup      = "UNKNOWN_GROUP";
inline constexpr const char* kCauseValidationFailed  = "VALIDATION_FAILED";
// ... MoveWaypoint's own refusals before the runtime is asked (REQUEST), and an exception (no phase)
inline constexpr const char* kCauseEmptyWaypoint     = "EMPTY_WAYPOINT";
inline constexpr const char* kCauseUnknownMotion     = "UNKNOWN_MOTION";
inline constexpr const char* kCauseServoExecution    = "SERVO_EXECUTION";
inline constexpr const char* kCauseUnknownExecution  = "UNKNOWN_EXECUTION";
inline constexpr const char* kCauseException         = "EXCEPTION";

// NONE on success; else TARGET or FAULT by the table above.
MotionFailureClass classifyMotionFailure(const MotionResult& r);

// "MOTION_FAULT: <CAUSE>[ during <phase>]": the recorded cause, else INCONSISTENT_RESULT (a failed
// result carrying SUCCESS), NO_ERROR_CODE (0), the MoveIt code's name, or CODE_<n>; " during <phase>"
// unless the phase is NONE. It carries no project data (the waypoint's name is not in it).
std::string motionFaultText(const MotionResult& r);

// The name moveit_msgs gives an error code ("NO_IK_SOLUTION"); "" when it names no such value.
const char* moveItErrorCodeName(int code);

// request | plan | validate | execute | plan_and_execute; "" for NONE.
const char* toString(MotionPhase phase);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_FAILURE_HPP_
