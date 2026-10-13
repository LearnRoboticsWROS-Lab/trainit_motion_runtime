// vNext M2.6 AFTER (ADR-0022), M2: the actuator outcome mapping, every row of the D2 table.
// Pure: (interface, engage target or not, result_policy, phase, status, error_code, flags,
// server flavour) -> DONE or ONE reason, and whether the goal is cancelled. DONE is actuation
// only; nothing here is a grasp.
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "trainit_motion_runtime/actuator_outcome.hpp"

namespace
{
namespace eef = trainit::eef;
using CI = eef::CommandInterface;
using P = eef::Phase;
using S = eef::GoalStatus;
using R = eef::Reason;
using RP = eef::ResultPolicy;

struct Row
{
  std::string what;
  eef::ObservedCommand in;
  bool done;
  R reason;
  bool cancel;
};

eef::ObservedCommand cmd(CI i, P phase)
{
  eef::ObservedCommand c;
  c.command_interface = i;
  c.phase = phase;
  return c;
}

eef::ObservedCommand gripper(bool engage, RP policy, S status, bool reached, bool stalled, bool ros2 = false)
{
  auto c = cmd(CI::GRIPPER_COMMAND, P::RESULT);
  c.engage_target = engage;
  c.result_policy = policy;
  c.status = status;
  c.reached_goal = reached;
  c.stalled = stalled;
  c.ros2_control = ros2;
  return c;
}

eef::ObservedCommand fjt(S status, int32_t code, bool ros2 = false, const std::string& text = "")
{
  auto c = cmd(CI::FOLLOW_JOINT_TRAJECTORY, P::RESULT);
  c.status = status;
  c.error_code = code;
  c.error_string = text;
  c.ros2_control = ros2;
  return c;
}

eef::ObservedCommand setBool(bool success, const std::string& message = "")
{
  auto c = cmd(CI::SET_BOOL, P::RESULT);
  c.success = success;
  c.message = message;
  return c;
}

eef::ObservedCommand flavour(eef::ObservedCommand c, bool ros2)
{
  c.ros2_control = ros2;
  return c;
}

std::vector<Row> table()
{
  const auto SB = CI::SET_BOOL;
  const auto GC = CI::GRIPPER_COMMAND;
  const auto FJ = CI::FOLLOW_JOINT_TRAJECTORY;
  return {
    // any: no server within 5000 ms
    {"SetBool, no server", cmd(SB, P::NO_SERVER), false, R::SERVER_MISSING, false},
    {"GripperCommand, no server", cmd(GC, P::NO_SERVER), false, R::SERVER_MISSING, false},
    {"FJT, no server", cmd(FJ, P::NO_SERVER), false, R::SERVER_MISSING, false},
    // actions: no goal response within 5000 ms -- a late-accepted goal is cancelled, except on a
    // ros2_control GripperCommand server
    {"GripperCommand (TrainIt mock), no goal response", cmd(GC, P::NO_RESPONSE), false, R::SEND_TIMEOUT, true},
    {"GripperCommand (ros2_control), no goal response", flavour(cmd(GC, P::NO_RESPONSE), true), false, R::SEND_TIMEOUT, false},
    {"FJT (mock), no goal response", cmd(FJ, P::NO_RESPONSE), false, R::SEND_TIMEOUT, true},
    {"FJT (JTC), no goal response", flavour(cmd(FJ, P::NO_RESPONSE), true), false, R::SEND_TIMEOUT, true},
    // actions: REJECTED
    {"GripperCommand rejected", cmd(GC, P::REJECTED), false, R::COMMAND_REJECTED, false},
    {"FJT rejected (e.g. a one-joint goal to a 2-joint JTC)", cmd(FJ, P::REJECTED), false, R::COMMAND_REJECTED, false},
    // SetBool
    {"SetBool success:false", setBool(false, "no stage subscriber"), false, R::COMMAND_REJECTED, false},
    {"SetBool no response by command_timeout_ms", cmd(SB, P::NO_RESPONSE), false, R::COMMAND_TIMEOUT, false},
    {"SetBool success:true (APPLIED)", setBool(true), true, R::NONE, false},
    // GripperCommand, the ENGAGE target, by result_policy
    {"engage, succeeded: SUCCEEDED", gripper(true, RP::SUCCEEDED, S::SUCCEEDED, true, false), true, R::NONE, false},
    {"engage, succeeded: SUCCEEDED stalled", gripper(true, RP::SUCCEEDED, S::SUCCEEDED, false, true), true, R::NONE, false},
    {"engage, succeeded: ABORTED stalled is outside the policy", gripper(true, RP::SUCCEEDED, S::ABORTED, false, true), false, R::COMMAND_ABORTED, false},
    {"engage, succeeded: ABORTED", gripper(true, RP::SUCCEEDED, S::ABORTED, false, false), false, R::COMMAND_ABORTED, false},
    {"engage, succeeded_or_stalled: SUCCEEDED", gripper(true, RP::SUCCEEDED_OR_STALLED, S::SUCCEEDED, true, false), true, R::NONE, false},
    {"engage, succeeded_or_stalled: ABORTED stalled (allow_stalling false)", gripper(true, RP::SUCCEEDED_OR_STALLED, S::ABORTED, false, true), true, R::NONE, false},
    {"engage, succeeded_or_stalled: ABORTED without a stall", gripper(true, RP::SUCCEEDED_OR_STALLED, S::ABORTED, false, false), false, R::COMMAND_ABORTED, false},
    {"engage, ABORTED with an empty result (the FAIRINO shim's answer)", gripper(true, RP::SUCCEEDED_OR_STALLED, S::ABORTED, false, false), false, R::COMMAND_ABORTED, false},
    // GripperCommand, any other target: SUCCEEDED with reached_goal; a stall is ACTUATOR_BLOCKED
    {"open: SUCCEEDED reached_goal", gripper(false, RP::SUCCEEDED, S::SUCCEEDED, true, false), true, R::NONE, false},
    {"open: SUCCEEDED stalled (blocked)", gripper(false, RP::SUCCEEDED_OR_STALLED, S::SUCCEEDED, false, true), false, R::ACTUATOR_BLOCKED, false},
    {"open: ABORTED stalled (blocked)", gripper(false, RP::SUCCEEDED_OR_STALLED, S::ABORTED, false, true), false, R::ACTUATOR_BLOCKED, false},
    {"open: SUCCEEDED with both flags false", gripper(false, RP::SUCCEEDED, S::SUCCEEDED, false, false), false, R::COMMAND_ABORTED, false},
    {"open: ABORTED without a stall", gripper(false, RP::SUCCEEDED, S::ABORTED, false, false), false, R::COMMAND_ABORTED, false},
    // actions: CANCELED, whatever the code (an empty GripperCommand result; FJT 0, or any code).
    // M9 measured the JTC 2.54.0: a preempted goal ends CANCELED with an EMPTY result (0, no
    // error_string), only when the preempting goal ends -- never -1 with a text
    {"GripperCommand preempted: CANCELED, empty result", gripper(true, RP::SUCCEEDED_OR_STALLED, S::CANCELED, false, false), false, R::COMMAND_CANCELED, false},
    {"GripperCommand CANCELED, open target", gripper(false, RP::SUCCEEDED, S::CANCELED, true, false), false, R::COMMAND_CANCELED, false},
    {"FJT (JTC) preempted: CANCELED 0, empty error_string (M9)", fjt(S::CANCELED, 0, true), false, R::COMMAND_CANCELED, false},
    {"FJT CANCELED -1 with a text (no installed server; CANCELED whatever the code)",
     fjt(S::CANCELED, -1, true, "Current goal cancelled due to new incoming action."), false, R::COMMAND_CANCELED, false},
    {"FJT CANCELED 0 (a cancel)", fjt(S::CANCELED, 0), false, R::COMMAND_CANCELED, false},
    // FJT
    {"FJT SUCCEEDED 0", fjt(S::SUCCEEDED, 0), true, R::NONE, false},
    {"FJT ABORTED -4", fjt(S::ABORTED, -4), false, R::PATH_TOLERANCE_VIOLATED, false},
    {"FJT ABORTED -5", fjt(S::ABORTED, -5), false, R::GOAL_TOLERANCE_VIOLATED, false},
    {"FJT ABORTED -1 from a JTC (deactivated)", fjt(S::ABORTED, -1, true), false, R::ACTUATOR_UNAVAILABLE, false},
    {"FJT ABORTED -1 from another server", fjt(S::ABORTED, -1, false, "mock: refused"), false, R::COMMAND_ABORTED, false},
    {"FJT ABORTED -2 (invalid joints)", fjt(S::ABORTED, -2, true), false, R::COMMAND_ABORTED, false},
    {"FJT SUCCEEDED with a nonzero code", fjt(S::SUCCEEDED, -5), false, R::COMMAND_ABORTED, false},
    {"FJT UNKNOWN status", fjt(S::UNKNOWN, 0), false, R::COMMAND_ABORTED, false},
    // actions: no result by the deadline -- cancelled, except a ros2_control GripperCommand goal
    {"GripperCommand (mock), no result", cmd(GC, P::NO_RESULT), false, R::COMMAND_TIMEOUT, true},
    {"GripperCommand (ros2_control), no result: NEVER cancelled", flavour(cmd(GC, P::NO_RESULT), true), false, R::COMMAND_TIMEOUT, false},
    {"FJT (mock), no result", cmd(FJ, P::NO_RESULT), false, R::COMMAND_TIMEOUT, true},
    {"FJT (JTC), no result: cancelled", flavour(cmd(FJ, P::NO_RESULT), true), false, R::COMMAND_TIMEOUT, true},
  };
}
}  // namespace

TEST(M26EefOutcome, every_row_of_the_d2_table)
{
  for (const auto& row : table())
  {
    const auto o = eef::mapOutcome(row.in);
    EXPECT_EQ(o.done, row.done) << row.what << ": " << o.detail;
    EXPECT_EQ(o.reason, row.reason) << row.what << ": got " << eef::toToken(o.reason) << " (" << o.detail << ")";
    EXPECT_EQ(o.cancel, row.cancel) << row.what;
    if (!o.done)
    {
      EXPECT_FALSE(o.detail.empty()) << row.what << ": a failure names its cause";
    }
    else
    {
      EXPECT_EQ(o.reason, R::NONE);
    }
  }
}

TEST(M26EefOutcome, every_actuator_reason_of_the_table_is_reachable)
{
  std::set<R> seen;
  for (const auto& row : table()) seen.insert(eef::mapOutcome(row.in).reason);
  for (const auto r : {R::SERVER_MISSING, R::SEND_TIMEOUT, R::COMMAND_REJECTED, R::COMMAND_TIMEOUT, R::COMMAND_ABORTED,
                       R::COMMAND_CANCELED, R::PATH_TOLERANCE_VIOLATED, R::GOAL_TOLERANCE_VIOLATED,
                       R::ACTUATOR_BLOCKED, R::ACTUATOR_UNAVAILABLE})
  {
    EXPECT_TRUE(seen.count(r)) << eef::toToken(r);
    EXPECT_EQ(eef::classOf(r), eef::ReasonClass::ACTUATOR_FAILURE) << eef::toToken(r);
  }
}

// The no-cancel rule: a ros2_control GripperCommand goal is never cancelled (after a deactivation
// its cancel callback reads a released state interface); everything else may be.
TEST(M26EefOutcome, a_ros2_control_gripper_goal_is_never_cancelled)
{
  EXPECT_FALSE(eef::mayCancel(CI::GRIPPER_COMMAND, true));
  EXPECT_TRUE(eef::mayCancel(CI::GRIPPER_COMMAND, false));
  EXPECT_TRUE(eef::mayCancel(CI::FOLLOW_JOINT_TRAJECTORY, true));
  EXPECT_TRUE(eef::mayCancel(CI::FOLLOW_JOINT_TRAJECTORY, false));
  for (const auto phase : {P::NO_RESPONSE, P::NO_RESULT})
  {
    EXPECT_FALSE(eef::mapOutcome(flavour(cmd(CI::GRIPPER_COMMAND, phase), true)).cancel);
    EXPECT_TRUE(eef::mapOutcome(flavour(cmd(CI::GRIPPER_COMMAND, phase), false)).cancel);
  }
}

// Result deadlines: GripperCommand command_timeout_ms; FJT max(command_timeout_ms,
// (duration_s + goal_time_tolerance_s) x 1.1 + 0.5 s); SetBool the response wait.
TEST(M26EefOutcome, the_result_deadlines)
{
  eef::ActuatorSpec axis;
  axis.command_interface = CI::FOLLOW_JOINT_TRAJECTORY;
  axis.duration_s = 1.0;
  axis.goal_time_tolerance_s = 0.5;
  EXPECT_EQ(eef::resultDeadlineMs(axis, 10000), 10000);
  EXPECT_EQ(eef::resultDeadlineMs(axis, 1000), 2150);   // (1.0 + 0.5) x 1.1 + 0.5 = 2.15 s
  axis.duration_s = 20.0;
  EXPECT_EQ(eef::resultDeadlineMs(axis, 10000), 23050);   // (20.5) x 1.1 + 0.5 = 23.05 s
  eef::ActuatorSpec jaw;
  jaw.command_interface = CI::GRIPPER_COMMAND;
  EXPECT_EQ(eef::resultDeadlineMs(jaw, 10000), 10000);
  eef::ActuatorSpec valve;
  valve.command_interface = CI::SET_BOOL;
  EXPECT_EQ(eef::resultDeadlineMs(valve, 4000), 4000);
  // saturated at INT32_MAX ms, the longest wait: the parser bounds no duration_s, so a huge, an
  // infinite or a NaN motion never overflows the conversion to milliseconds
  const int64_t longest = std::numeric_limits<int32_t>::max();
  for (const double d : {1e300, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    axis.duration_s = d;
    EXPECT_EQ(eef::resultDeadlineMs(axis, 10000), longest) << d;
  }
}
