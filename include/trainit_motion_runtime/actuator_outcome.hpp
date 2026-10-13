// =============================================================================
// actuator_outcome.hpp — what ONE actuator command ended as (ADR-0022 D2, "Outcome mapping").
//
// A pure function maps what the client observed of one command -- which phase it reached and,
// for a result, the pair (status, error_code) plus the GripperCommand flags, never the code
// alone -- to DONE or to exactly one reason, and says whether the goal must be cancelled.
// DONE is ACTUATION only (applied, within tolerance, reached or stalled), never a grasp.
//
//   any            no server within 5000 ms                    SERVER_MISSING
//   actions        no goal response within 5000 ms              SEND_TIMEOUT (+ cancel a late accept*)
//   actions        REJECTED                                     COMMAND_REJECTED
//   SetBool        success: false                               COMMAND_REJECTED (message -> detail)
//   SetBool        no response within command_timeout_ms        COMMAND_TIMEOUT (pending request removed)
//   GripperCommand the engage target outside result_policy      COMMAND_ABORTED
//   GripperCommand a stall on a non-engage target               ACTUATOR_BLOCKED
//   actions        CANCELED, whatever the code                  COMMAND_CANCELED
//   FJT            ABORTED -4 / -5                              PATH_ / GOAL_TOLERANCE_VIOLATED
//   FJT            ABORTED -1                                   JTC: ACTUATOR_UNAVAILABLE; else COMMAND_ABORTED
//   actions        no result by the deadline                    COMMAND_TIMEOUT (+ cancel*)
//   any            anything else                                COMMAND_ABORTED (flags, error_string)
//
// * never a ros2_control GripperCommand goal: after a deactivation the installed
//   GripperActionController's cancel callback reads a released state interface (undefined
//   behaviour in the controller manager's process); a later goal preempts it instead.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__ACTUATOR_OUTCOME_HPP_
#define TRAINIT_MOTION_RUNTIME__ACTUATOR_OUTCOME_HPP_

#include <cstdint>
#include <string>

#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_reason.hpp"

namespace trainit
{
namespace eef
{

// The fixed waits of the contract (ADR-0022 D2), on the steady clock.
inline constexpr int64_t kServerWaitMs = 5000;     // SERVER_MISSING
inline constexpr int64_t kGoalResponseMs = 5000;   // SEND_TIMEOUT
inline constexpr int64_t kConfirmDefaultMs = 2000; // the "must reach THIS device" window of a non-set_bool actuator

// How far ONE command got.
enum class Phase : uint8_t
{
  NO_SERVER,     // no server within kServerWaitMs
  NO_RESPONSE,   // actions: no goal response within kGoalResponseMs; SetBool: no response in time
  REJECTED,      // actions: the goal was rejected
  NO_RESULT,     // actions: accepted, no result by the result deadline
  RESULT,        // actions: a terminal status; SetBool: a response
};

// The terminal status of an action goal (rclcpp_action::ResultCode).
enum class GoalStatus : uint8_t
{
  SUCCEEDED,
  ABORTED,
  CANCELED,
  UNKNOWN,
};

struct ObservedCommand
{
  CommandInterface command_interface{CommandInterface::UNKNOWN};
  bool engage_target{false};   // GripperCommand: the goal is its site's ENGAGE target
  ResultPolicy result_policy{ResultPolicy::SUCCEEDED};
  bool ros2_control{false};    // a third-party (ros2_control) server, not the TrainIt adapter
  Phase phase{Phase::NO_SERVER};
  // RESULT of an action
  GoalStatus status{GoalStatus::UNKNOWN};
  int32_t error_code{0};       // FollowJointTrajectory
  std::string error_string{};  // FollowJointTrajectory
  bool reached_goal{false};    // GripperCommand
  bool stalled{false};         // GripperCommand
  // RESULT of a SetBool call
  bool success{false};
  std::string message{};
};

struct ActuatorOutcome
{
  bool done{false};
  Reason reason{Reason::NONE};
  std::string detail{};
  // NO_RESPONSE: cancel the goal if it is accepted late; NO_RESULT: cancel it now
  bool cancel{false};
};

ActuatorOutcome mapOutcome(const ObservedCommand& c);

// A client may cancel a goal, except a ros2_control GripperCommand goal.
bool mayCancel(CommandInterface command_interface, bool ros2_control);

// The result deadline: GripperCommand `command_timeout_ms`; FollowJointTrajectory
// max(command_timeout_ms, (duration_s + goal_time_tolerance_s) x 1.1 + 0.5 s); SetBool the
// response wait, `command_timeout_ms`.
int64_t resultDeadlineMs(const ActuatorSpec& actuator, int64_t command_timeout_ms);

const char* toToken(GoalStatus s);   // "SUCCEEDED" | "ABORTED" | "CANCELED" | "UNKNOWN"

}  // namespace eef
}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__ACTUATOR_OUTCOME_HPP_
