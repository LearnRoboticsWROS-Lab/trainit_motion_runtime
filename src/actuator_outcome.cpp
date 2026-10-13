// =============================================================================
// actuator_outcome.cpp — the ADR-0022 D2 outcome table, as one pure function.
// =============================================================================
#include "trainit_motion_runtime/actuator_outcome.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace trainit
{
namespace eef
{

namespace
{
using FjtResult = control_msgs::action::FollowJointTrajectory::Result;

ActuatorOutcome fail(Reason r, std::string detail, bool cancel = false)
{
  ActuatorOutcome o;
  o.reason = r;
  o.detail = std::move(detail);
  o.cancel = cancel;
  return o;
}

ActuatorOutcome done()
{
  ActuatorOutcome o;
  o.done = true;
  return o;
}

std::string flags(const ObservedCommand& c)
{
  return std::string("reached_goal=") + (c.reached_goal ? "true" : "false") +
         " stalled=" + (c.stalled ? "true" : "false");
}

std::string fjtText(const ObservedCommand& c)
{
  std::string s = std::string("FollowJointTrajectory ") + toToken(c.status) +
                  " error_code=" + std::to_string(c.error_code);
  if (!c.error_string.empty()) s += ": " + c.error_string;
  return s;
}

ActuatorOutcome gripperResult(const ObservedCommand& c)
{
  const std::string what = std::string("GripperCommand ") + toToken(c.status) + " " + flags(c);
  if (c.status == GoalStatus::CANCELED) return fail(Reason::COMMAND_CANCELED, what);
  if (c.engage_target)
  {
    if (c.status == GoalStatus::SUCCEEDED) return done();
    if (c.status == GoalStatus::ABORTED && c.stalled && c.result_policy == ResultPolicy::SUCCEEDED_OR_STALLED)
      return done();
    return fail(Reason::COMMAND_ABORTED, what + " on the engage target, outside result_policy " +
                                           toToken(c.result_policy));
  }
  if (c.status == GoalStatus::SUCCEEDED && c.reached_goal) return done();
  if ((c.status == GoalStatus::SUCCEEDED || c.status == GoalStatus::ABORTED) && c.stalled)
    return fail(Reason::ACTUATOR_BLOCKED, what + " on a non-engage target");
  return fail(Reason::COMMAND_ABORTED, what + " on a non-engage target");
}

ActuatorOutcome trajectoryResult(const ObservedCommand& c)
{
  const std::string what = fjtText(c);
  if (c.status == GoalStatus::CANCELED) return fail(Reason::COMMAND_CANCELED, what);
  if (c.status == GoalStatus::SUCCEEDED && c.error_code == FjtResult::SUCCESSFUL) return done();
  if (c.status == GoalStatus::ABORTED)
  {
    if (c.error_code == FjtResult::PATH_TOLERANCE_VIOLATED) return fail(Reason::PATH_TOLERANCE_VIOLATED, what);
    if (c.error_code == FjtResult::GOAL_TOLERANCE_VIOLATED) return fail(Reason::GOAL_TOLERANCE_VIOLATED, what);
    if (c.error_code == FjtResult::INVALID_GOAL && c.ros2_control)
      return fail(Reason::ACTUATOR_UNAVAILABLE, what + " (a joint_trajectory_controller ends its goal so "
                                                       "when it is deactivated)");
  }
  return fail(Reason::COMMAND_ABORTED, what);
}
}  // namespace

const char* toToken(GoalStatus s)
{
  switch (s)
  {
    case GoalStatus::SUCCEEDED: return "SUCCEEDED";
    case GoalStatus::ABORTED: return "ABORTED";
    case GoalStatus::CANCELED: return "CANCELED";
    case GoalStatus::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

bool mayCancel(CommandInterface command_interface, bool ros2_control)
{
  return !(ros2_control && command_interface == CommandInterface::GRIPPER_COMMAND);
}

ActuatorOutcome mapOutcome(const ObservedCommand& c)
{
  const bool set_bool = c.command_interface == CommandInterface::SET_BOOL;
  const bool cancel = mayCancel(c.command_interface, c.ros2_control);
  switch (c.phase)
  {
    case Phase::NO_SERVER:
      return fail(Reason::SERVER_MISSING, "no server within " + std::to_string(kServerWaitMs) + " ms");
    case Phase::NO_RESPONSE:
      if (set_bool) return fail(Reason::COMMAND_TIMEOUT, "no SetBool response by the deadline; the pending request was removed");
      return fail(Reason::SEND_TIMEOUT,
                  "no goal response within " + std::to_string(kGoalResponseMs) + " ms" +
                    (cancel ? "; a late-accepted goal is cancelled" : "; a late-accepted goal is NOT cancelled (ros2_control GripperCommand)"),
                  cancel);
    case Phase::REJECTED:
      return fail(Reason::COMMAND_REJECTED, "the goal was rejected");
    case Phase::NO_RESULT:
      return fail(Reason::COMMAND_TIMEOUT,
                  std::string("no result by the deadline; ") +
                    (cancel ? "the goal is cancelled" : "the goal is NOT cancelled (ros2_control GripperCommand)"),
                  cancel);
    case Phase::RESULT:
      break;
  }
  switch (c.command_interface)
  {
    case CommandInterface::SET_BOOL:
      if (c.success) return done();
      return fail(Reason::COMMAND_REJECTED, "SetBool success: false" + (c.message.empty() ? std::string() : ": " + c.message));
    case CommandInterface::GRIPPER_COMMAND:
      return gripperResult(c);
    case CommandInterface::FOLLOW_JOINT_TRAJECTORY:
      return trajectoryResult(c);
    case CommandInterface::UNKNOWN:
      break;
  }
  return fail(Reason::COMMAND_ABORTED, "unknown command interface");
}

int64_t resultDeadlineMs(const ActuatorSpec& actuator, int64_t command_timeout_ms)
{
  if (actuator.command_interface != CommandInterface::FOLLOW_JOINT_TRAJECTORY) return command_timeout_ms;
  // to the nearest millisecond: (1.0 + 0.5) x 1.1 + 0.5 s is 2150 ms, not 2151; saturated at
  // INT32_MAX ms, the longest wait (the parser bounds no duration_s, and llround must not
  // overflow: an infinite or NaN motion reads as the longest)
  const double motion_ms =
    ((actuator.duration_s + actuator.goal_time_tolerance_s) * 1.1 + 0.5) * 1000.0;
  const double longest_ms = static_cast<double>(std::numeric_limits<int32_t>::max());
  const double bounded_ms = motion_ms < longest_ms ? motion_ms : longest_ms;
  return std::max(command_timeout_ms, static_cast<int64_t>(std::llround(bounded_ms)));
}

}  // namespace eef
}  // namespace trainit
