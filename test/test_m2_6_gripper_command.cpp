// vNext M2.6 (tests BEFORE, ADR-0022 family E1): eef_gripper_command -- what CloseGripper,
// OpenGripper and SetGripper do through trainit::GripperController against every GripperCommand
// server answer TrainIt meets today, frozen on production 1dda503 before the canonical EEF
// contract touches a production line (test/data/m2_6/README.md).
//
// The servers are ScriptedGripper fakes (test/m2_6/eef_fakes.hpp), one per scenario, answering
// like the real ones do: the TSA bootstrap mock (always SUCCEEDED), the FAIRINO suction shim
// (every goal ABORTED, reached_goal=false), ros2_control's GripperActionController (SUCCEEDED or
// ABORTED with stalled=true, by allow_stalling; CANCELED with an empty result when preempted).
// The context is built the way trainit_run_bt builds it (makeRunnerContext). What is pinned:
// the goal each node sends (position, max_effort), whether the result code reaches the BT, the
// 5 s server / 5 s goal-response / 10 s result waits, and that nothing is ever cancelled.
#include <string>
#include <vector>

#include "m2_6/gripper_scenarios.hpp"

using m26::GripperReply;
using m26::GripperScenario;
using m26::RunnerKeys;

namespace
{
RunnerKeys action(const std::string& name, double close = 1.0, double open = 0.0)
{
  RunnerKeys k;
  k.gripper_action = name;
  k.close_pos = close;
  k.open_pos = open;
  return k;
}

const char* kClose = "<CloseGripper/>";
const char* kOpen = "<OpenGripper/>";
}  // namespace

TEST(M26GripperCommand, eef_gripper_command)
{
  const std::vector<GripperScenario> scenarios{
    {"GC-01", "the TSA bootstrap mock: every goal SUCCEEDED reached_goal=true; close 1.0 and open 0.0 (the suction defaults)",
     action("/m26_gc01/gripper_command"), true, {GripperReply::succeed(), GripperReply::succeed()}, {kClose, kOpen}},
    {"GC-02", "the FAIRINO suction shim: every goal ABORTED reached_goal=false; the BT still reports SUCCESS",
     action("/m26_gc02/gripper_command"), true, {GripperReply::shimAbort(), GripperReply::shimAbort()}, {kClose, kOpen}},
    {"GC-03", "ABORTED with an empty result: the BT still reports SUCCESS",
     action("/m26_gc03/gripper_command"), true, {GripperReply::abortEmpty()}, {kClose}},
    {"GC-04", "ros2_control allow_stalling=true: the jaws stall on the object, SUCCEEDED reached_goal=false stalled=true",
     action("/m26_gc04/gripper_command", 0.7691, 0.0), true, {GripperReply::succeed(false, true).at(0.35, 0.0)}, {kClose}},
    {"GC-05", "ros2_control allow_stalling=false (its default): the same stall ends ABORTED; the BT still reports SUCCESS",
     action("/m26_gc05/gripper_command", 0.7691, 0.0), true, {GripperReply::abort(false, true).at(0.35, 0.0)}, {kClose}},
    {"GC-06", "an EMPTY close (nothing between the jaws) ends SUCCEEDED reached_goal=true: the same answer as a grasp",
     action("/m26_gc06/gripper_command", 0.7691, 0.0), true, {GripperReply::succeed(true, false)}, {kClose}},
    {"GC-07", "a preemption: a second client's real cancel request ends the goal CANCELED with an empty result; the BT still reports SUCCESS",
     action("/m26_gc07/gripper_command"), true, {GripperReply::preempt()}, {kClose}},
    {"GC-08", "the server REJECTS the goal: FAILURE, and nothing else is tried",
     action("/m26_gc08/gripper_command"), true, {GripperReply::reject()}, {kClose}},
    {"GC-09", "no server at the configured name: FAILURE after the 5 s server wait",
     action("/m26_gc09/gripper_command"), false, {}, {kClose}},
    {"GC-10", "a LATE accept: the goal response comes after the 5 s goal-response wait; the client returns FAILURE, the server still "
              "executes the goal afterwards, and no cancel request ever arrives",
     action("/m26_gc10/gripper_command"), true, {GripperReply::succeed().acceptAfter(7000)}, {kClose}, 1000},
    {"GC-11", "the result is WITHHELD past the 10 s result wait: FAILURE; the goal stays EXECUTING and no cancel request ever arrives",
     action("/m26_gc11/gripper_command"), true, {GripperReply::withhold()}, {kClose}, 1000},
    {"GC-12", "configured joint targets (close 0.7691 rad, open 0.0 rad): the goals carry them, max_effort stays 0",
     action("/m26_gc12/gripper_command", 0.7691, 0.0), true, {}, {kClose, kOpen}},
    {"GC-13", "SetGripper with a numeric position, then with none (the port default \"1.0\")",
     action("/m26_gc13/gripper_command"), true, {}, {"<SetGripper position=\"0.35\"/>", "<SetGripper/>"}},
    {"GC-14", "SetGripper with a non-numeric position: std::stod throws inside the tick; no goal is sent",
     action("/m26_gc14/gripper_command"), true, {}, {"<SetGripper position=\"half\"/>"}},
    {"GC-15", "OpenGripper against a SUCCEEDED and then an ABORTED release: SUCCESS both times",
     action("/m26_gc15/gripper_command"), true, {GripperReply::succeed(), GripperReply::abort(false, false)}, {kOpen, kOpen}},
    {"GC-16", "neither a gripper action nor a grasp topic configured: every gripper node fails before any traffic",
     RunnerKeys{}, false, {}, {kClose, kOpen, "<SetGripper position=\"0.5\"/>"}},
  };
  m26::Family f("eef_gripper_command");
  for (const auto& s : scenarios) f.add(m26::runGripperScenario(s));
  f.check();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
