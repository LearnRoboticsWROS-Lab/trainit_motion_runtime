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
// ... vNext M3.1 (ADR-0024): MoveWaypoint's additive keys and FollowPath, refused before the
// runtime is asked (REQUEST, so always a FAULT: the configuration's or the robot state's)
inline constexpr const char* kCauseUnknownProfile    = "UNKNOWN_PROFILE";     // a named profile not loaded
inline constexpr const char* kCauseFrameUnresolved   = "FRAME_UNRESOLVED";    // a pose's frame has no transform
inline constexpr const char* kCauseUnsupportedFrame  = "UNSUPPORTED_FRAME";   // a frame on a target with no pose
inline constexpr const char* kCauseInvalidPose       = "INVALID_POSE";        // a non-finite or non-unit pose
inline constexpr const char* kCauseUnsupportedTcp    = "UNSUPPORTED_TCP";     // a tcp other than the tip (M3.4)
inline constexpr const char* kCauseNoPathLibrary     = "NO_PATH_LIBRARY";     // no paths_file was loaded
inline constexpr const char* kCauseUnknownPath       = "UNKNOWN_PATH";
inline constexpr const char* kCausePathUnresolved    = "PATH_UNRESOLVED";     // a state's waypoint is not a pose
inline constexpr const char* kCausePathGeometry      = "PATH_GEOMETRY";       // resolved states fail the geometry
inline constexpr const char* kCauseNotAtPathStart    = "NOT_AT_PATH_START";   // the robot is not at states[0]
// ... vNext M3.4 (ADR-0015 amendment M3.4, owner D3): what the runtime cannot honour is refused before
// MoveIt is asked (REQUEST, so always a FAULT), never dropped or replaced silently
inline constexpr const char* kCauseCircleGeometry    = "CIRCLE_GEOMETRY";     // a CIRC aux point on the goal
inline constexpr const char* kCauseUnknownPlanner    = "UNKNOWN_PLANNER";     // not a planner mode (R-M3-19)
inline constexpr const char* kCauseStateUnavailable  = "STATE_UNAVAILABLE";   // no fresh robot state (R-M3-20)
inline constexpr const char* kCauseUnsupportedTolerance = "UNSUPPORTED_TOLERANCE";   // a goal tolerance not honourable
// ... TMR 2.6.0 (ADR-0028), both refused before anything is sent (phase request: a FAULT, the program's):
// a pose that cannot be reached near the configuration it was taught in, and a taught configuration that
// is not one (its size, a non-finite value, out of the joints' bounds)
inline constexpr const char* kCauseConfigurationUnreachable = "CONFIGURATION_UNREACHABLE";
inline constexpr const char* kCauseInvalidConfiguration = "INVALID_CONFIGURATION";
// ... vNext M3.5 (ADR-0024, M3_DESIGN §9): FollowPath's event-free runs of a path (from= to=)
inline constexpr const char* kCausePathRange         = "PATH_RANGE";          // from/to is not a run of the path
inline constexpr const char* kCauseNotAtPathState    = "NOT_AT_PATH_STATE";   // a run ended away from its event state

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
