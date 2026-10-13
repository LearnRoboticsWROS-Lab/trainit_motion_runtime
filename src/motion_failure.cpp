// =============================================================================
// motion_failure.cpp -- the classification of a failed motion (vNext M2.7; owner Q8.2). The table is
// in motion_failure.hpp. Switches only: no table object, no function-static. Built with
// -Werror=switch (CMakeLists.txt), so a MotionPhase added without a case here is a build error.
// =============================================================================
#include "trainit_motion_runtime/motion_failure.hpp"

namespace trainit
{

namespace
{
using Codes = moveit_msgs::msg::MoveItErrorCodes;

// The MoveIt codes that name THIS goal in a planning answer: no plan for it, an invalid plan, the goal
// in collision or violating its constraints, no IK. TIMED_OUT is added for the PLAN phase only.
bool isTargetCode(int code)
{
  switch (code)
  {
    case Codes::PLANNING_FAILED:
    case Codes::INVALID_MOTION_PLAN:
    case Codes::GOAL_IN_COLLISION:
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS:
    case Codes::GOAL_CONSTRAINTS_VIOLATED:
    case Codes::GOAL_STATE_INVALID:
    case Codes::NO_IK_SOLUTION:
      return true;
    default:
      return false;
  }
}
}  // namespace

MotionFailureClass classifyMotionFailure(const MotionResult& r)
{
  if (r.success) return MotionFailureClass::NONE;
  // a recorded cause wins over the code: only two of the validator's checks name this path
  // (START_STATE_COLLISION, a collision at waypoint 0, is the robot's start: a FAULT)
  if (!r.cause.empty())
    return (r.phase == MotionPhase::VALIDATE && (r.cause == kCauseJointJump || r.cause == kCauseCollision))
             ? MotionFailureClass::TARGET : MotionFailureClass::FAULT;
  const int c = r.error_code.val;
  switch (r.phase)
  {
    case MotionPhase::PLAN:
      return (isTargetCode(c) || c == Codes::TIMED_OUT) ? MotionFailureClass::TARGET : MotionFailureClass::FAULT;
    case MotionPhase::PLAN_AND_EXECUTE:
      return isTargetCode(c) ? MotionFailureClass::TARGET : MotionFailureClass::FAULT;
    case MotionPhase::NONE:
    case MotionPhase::REQUEST:
    case MotionPhase::VALIDATE:
    case MotionPhase::EXECUTE:
      return MotionFailureClass::FAULT;
  }
  return MotionFailureClass::FAULT;   // a value outside the enum: default-deny
}

std::string motionFaultText(const MotionResult& r)
{
  std::string cause = r.cause;
  if (cause.empty())
  {
    const int c = r.error_code.val;
    const std::string named = moveItErrorCodeName(c);
    if (c == Codes::SUCCESS) cause = "INCONSISTENT_RESULT";   // a failed result that says SUCCESS
    else if (c == 0) cause = "NO_ERROR_CODE";
    else if (!named.empty()) cause = named;
    else cause = "CODE_" + std::to_string(c);
  }
  const std::string phase = toString(r.phase);
  return "MOTION_FAULT: " + cause + (phase.empty() ? std::string() : " during " + phase);
}

const char* moveItErrorCodeName(int code)
{
  switch (code)
  {
    case Codes::SUCCESS: return "SUCCESS";
    case Codes::FAILURE: return "FAILURE";
    case Codes::PLANNING_FAILED: return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case Codes::CONTROL_FAILED: return "CONTROL_FAILED";
    case Codes::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case Codes::TIMED_OUT: return "TIMED_OUT";
    case Codes::PREEMPTED: return "PREEMPTED";
    case Codes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::START_STATE_INVALID: return "START_STATE_INVALID";
    case Codes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::GOAL_STATE_INVALID: return "GOAL_STATE_INVALID";
    case Codes::UNRECOGNIZED_GOAL_TYPE: return "UNRECOGNIZED_GOAL_TYPE";
    case Codes::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case Codes::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case Codes::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case Codes::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case Codes::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case Codes::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case Codes::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case Codes::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case Codes::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case Codes::COMMUNICATION_FAILURE: return "COMMUNICATION_FAILURE";
    case Codes::CRASH: return "CRASH";
    case Codes::ABORT: return "ABORT";
    case Codes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "";
  }
}

const char* toString(MotionPhase phase)
{
  switch (phase)
  {
    case MotionPhase::NONE: return "";
    case MotionPhase::REQUEST: return "request";
    case MotionPhase::PLAN: return "plan";
    case MotionPhase::VALIDATE: return "validate";
    case MotionPhase::EXECUTE: return "execute";
    case MotionPhase::PLAN_AND_EXECUTE: return "plan_and_execute";
  }
  return "";
}

}  // namespace trainit
