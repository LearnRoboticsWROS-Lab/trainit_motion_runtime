// vNext M2.7 (owner Q8.2): MoveWaypoint's optional `fault` output, ticked for real -- the REAL
// MoveWaypoint on a REAL MoveItMotionRuntime against the M2.5 fake move_group (test/m2_5/).
//
// Every case is ticked TWICE on a fresh runtime with the same script: as
//   <MoveWaypoint waypoint="a" fault="{f}"/>   (a tree that remaps the port, as the M2.7 canonical
//                                               bin-picking cycle does), and as
//   <MoveWaypoint waypoint="a"/>               (every other tree: the port is not remapped).
// What MoveIt received (the events, their bodies, the counts), the BT status and the planner mode left
// behind must be IDENTICAL in the two runs: the port changes no MoveIt traffic and no BT result. The
// remapped run's `f` must be the expected value (empty for a TARGET failure and on SUCCESS); the plain
// run never writes it. The new ERROR line ("... not this target's failure") appears only in a remapped
// run whose move failed for a reason that is not the target's; a plain run logs nothing new.
//
// Not covered end to end (too slow, classifier and text tests only, test_m2_7_motion_failure): the Pilz
// goal-send timeout (10 s) and result timeout (120 s); a REJECTED MoveGroup or ExecuteTrajectory goal
// (MoveGroupInterface dereferences a null result, test/m2_5/fake_move_group.hpp). Nor a validator
// COLLISION: the M2.5 test arm has no collision geometry (test/m2_5/test_arm.hpp), so no plan collides
// here; test_m2_7_motion_failure runs the TrajectoryValidator on a real model and scene instead
// (START_STATE_COLLISION at waypoint 0, a FAULT; COLLISION at a later waypoint, the target's).
//
// Domain 81 (INTEGRATION 3.2), set in code and in CMake, localhost only, graph-guarded (m25::runAll).
// The two blocks of every case are written to m2_7_mf_actual/motion_fault/motion_fault_report.txt
// (evidence only, compared with no committed file).
#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "m2_5/motion_harness.hpp"

using m25::Reply;
using m25::Scenario;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

// --- the new ERROR line, counted through rcutils' output handler ------------------------------------
std::mutex g_tap_m;
int g_not_target_lines = 0;
rcutils_logging_output_handler_t g_previous = nullptr;

void tap(const rcutils_log_location_t* location, int severity, const char* name, rcutils_time_point_value_t ts,
         const char* format, va_list* args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[2048];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  if (std::strstr(buf, "not this target's failure") != nullptr)
  {
    std::lock_guard<std::mutex> g(g_tap_m);
    ++g_not_target_lines;
  }
  if (g_previous) g_previous(location, severity, name, ts, format, args);
}

void installTap()
{
  rcutils_logging_output_handler_t current = rcutils_logging_get_output_handler();
  if (current == &tap) return;
  g_previous = current;
  rcutils_logging_set_output_handler(&tap);
  // restored by m25::runAll before the World is destroyed and before rclcpp::shutdown()
  m25::teardowns().push_back([] { rcutils_logging_set_output_handler(g_previous); });
}

int takeNotTargetLines()
{
  std::lock_guard<std::mutex> g(g_tap_m);
  const int n = g_not_target_lines;
  g_not_target_lines = 0;
  return n;
}

// --- the waypoint `a`, written the way trainit_run_bt writes bt_params (strings) -------------------
Strings waypoint(const Strings& fields)
{
  Strings out;
  for (const auto& [k, v] : fields) out.emplace_back("a." + k, v);
  out.emplace_back("f", "");   // the gate, preset empty (the canonical cycle's reset Script)
  return out;
}
Strings join(Strings a, const Strings& b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}
const Strings kTcp = {{"type", "tcp"}, {"position", "0.4;0.1;0.3"}, {"orientation", "1;0;0;0"}};
const Strings kTcpLin = join(kTcp, {{"motion", "lin"}});
const Strings kTcpFree = join(kTcp, {{"motion", "free"}});
const Strings kTcpFreeOmpl = join(kTcp, {{"motion", "free"}, {"planner", "ompl"}});
const Strings kJointReady = {{"type", "joint"}, {"named", "ready"}};
const Strings kJointReadyPtpPilz = {{"type", "joint"}, {"named", "ready"}, {"motion", "ptp"}, {"planner", "pilz"}};
const Strings kCirc = join(kTcp, {{"motion", "circ"}, {"aux", "0.43;0.12;0.32"}});

std::function<void(m25::FakeMoveGroup&)> plans(int n, const Reply& r)
{
  return [n, r](m25::FakeMoveGroup& mg) { for (int i = 0; i < n; ++i) mg.mg_script.push_back(r); };
}
std::function<void(m25::FakeMoveGroup&)> executes(const Reply& r)
{
  return [r](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(r); };
}
std::function<void(m25::FakeMoveGroup&)> sequence(const Reply& r)
{
  return [r](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(r); };
}

// One case of WP_MF section 8 (MF-01 .. MF-31).
struct Case
{
  std::string id;
  std::string pins;
  Strings data;                                          // the waypoint `a` (and the preset gate)
  std::function<void(m25::FakeMoveGroup&)> script{};     // MoveIt's answers
  std::function<void()> before_tick{};                   // e.g. drop a server
  std::function<void()> after{};                         // e.g. restore it
  std::string bt{"FAILURE"};                             // the BT status both runs must end with
  std::string fault{};                                   // the remapped run's `f`
  bool no_events{false};                                 // nothing reaches MoveIt
  std::string waypoint_name{"a"};                        // MF-27: the empty name
};

std::string reportFault(BT::Blackboard& bb)
{
  try { return "fault: \"" + bb.get<std::string>("f") + "\"\n"; }
  catch (const std::exception& e) { return std::string("fault: (unreadable: ") + e.what() + ")\n"; }
}

// The lines of a capture block that must be identical in the two runs: every line but the scenario id,
// the tree (the only input that differs) and the report's `fault:` line.
std::string comparable(const std::string& block)
{
  std::istringstream in(block);
  std::string line, out;
  while (std::getline(in, line))
  {
    if (line.rfind("=== scenario ", 0) == 0 || line.rfind("--- tick ", 0) == 0 || line.rfind("fault: ", 0) == 0)
      continue;
    out += line + "\n";
  }
  return out;
}

// What MoveIt received: the lines between "events:" and "bt: " (the events, then any topics).
std::string eventsOf(const std::string& block)
{
  std::istringstream in(block);
  std::string line, out;
  bool inside = false;
  while (std::getline(in, line))
  {
    if (line == "events:") { inside = true; continue; }
    if (inside && line.rfind("bt: ", 0) == 0) return out;
    if (inside) out += line + "\n";
  }
  return "(no events: ... bt: section)";
}

std::string lineValue(const std::string& block, const std::string& key)
{
  const auto a = block.find("\n" + key);
  if (a == std::string::npos) return "(no " + key + ")";
  const auto b = block.find('\n', a + 1 + key.size());
  return block.substr(a + 1 + key.size(), b - a - 1 - key.size());
}

std::string& reportText()
{
  static std::string r;   // test harness: the report of this process
  return r;
}

void runCase(const Case& c)
{
  m25::World& w = m25::world();
  installTap();
  const std::string remapped = "<MoveWaypoint waypoint=\"" + c.waypoint_name + "\" fault=\"{f}\"/>";
  const std::string plain = "<MoveWaypoint waypoint=\"" + c.waypoint_name + "\"/>";
  auto scenario = [&](const std::string& tag, const std::string& tree) {
    return Scenario{c.id + " " + tag, c.pins, nullptr, c.data, {}, {tree}, c.script, c.before_tick, c.after,
                    {0.0, 0.0, 0.0}, true, 90, reportFault};
  };
  takeNotTargetLines();
  const std::string a = m25::run(w, scenario("fault=", remapped));
  const int a_lines = takeNotTargetLines();
  const std::string b = m25::run(w, scenario("plain", plain));
  const int b_lines = takeNotTargetLines();
  reportText() += a + b;

  // the port changes no MoveIt traffic, no BT result, no planner mode left behind
  EXPECT_EQ(comparable(a), comparable(b)) << c.id << ": the two runs differ beyond the tree and `fault`";
  EXPECT_EQ(eventsOf(a), eventsOf(b)) << c.id;
  EXPECT_EQ(lineValue(a, "bt: "), c.bt) << c.id;
  EXPECT_EQ(lineValue(b, "bt: "), c.bt) << c.id;
  // the remapped run writes the expected value; the plain run never writes it
  EXPECT_EQ(lineValue(a, "fault: "), "\"" + c.fault + "\"") << c.id;
  EXPECT_EQ(lineValue(b, "fault: "), "\"\"") << c.id;
  if (c.no_events)
  {
    EXPECT_EQ(eventsOf(a), "") << c.id << ": nothing may reach MoveIt";
    EXPECT_EQ(lineValue(a, "counts: "), "none") << c.id;
  }
  // the new ERROR line: only a remapped run whose move the runtime refused for a reason that is not the
  // target's (MoveWaypoint's own refusals and an exception keep only their existing ERROR line)
  const bool runtime_fault = !c.fault.empty() && c.fault.find(" during request") == std::string::npos &&
                             c.fault != "MOTION_FAULT: EXCEPTION";
  EXPECT_EQ(a_lines, runtime_fault ? 1 : 0) << c.id << ": 'not this target's failure' lines, remapped run";
  EXPECT_EQ(b_lines, 0) << c.id << ": a tree that does not remap `fault` logs nothing new";
}

void runCases(const std::vector<Case>& cases)
{
  for (const auto& c : cases) runCase(c);
}
}  // namespace

// --- R1/R2: the PLAN phase (MoveGroupInterface::plan, plan_retries + 1 = 4 goals) ---------------------
TEST(M27MotionFault, plan_phase)
{
  runCases({
    {"MF-01", "tcp lin, NO_IK_SOLUTION on every plan: the target's (TARGET)", waypoint(kTcpLin),
     plans(4, Reply::abort(m25::ErrorCodes::NO_IK_SOLUTION)), nullptr, nullptr, "FAILURE", ""},
    {"MF-02", "tcp free ompl, PLANNING_FAILED on every plan (TARGET)", waypoint(kTcpFreeOmpl),
     plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED)), nullptr, nullptr, "FAILURE", ""},
    {"MF-03", "tcp free ompl, TIMED_OUT on every plan: the planner's own budget (TARGET)", waypoint(kTcpFreeOmpl),
     plans(4, Reply::abort(m25::ErrorCodes::TIMED_OUT)), nullptr, nullptr, "FAILURE", ""},
    {"MF-04", "tcp free ompl, GOAL_IN_COLLISION on every plan (TARGET)", waypoint(kTcpFreeOmpl),
     plans(4, Reply::abort(m25::ErrorCodes::GOAL_IN_COLLISION)), nullptr, nullptr, "FAILURE", ""},
    {"MF-05", "joint named ready, ptp pilz, START_STATE_IN_COLLISION on every plan: the robot's state (FAULT)",
     waypoint(kJointReadyPtpPilz), plans(4, Reply::abort(m25::ErrorCodes::START_STATE_IN_COLLISION)), nullptr,
     nullptr, "FAILURE", "MOTION_FAULT: START_STATE_IN_COLLISION during plan"},
    {"MF-06", "tcp free, FRAME_TRANSFORM_FAILURE on every plan (FAULT)", waypoint(kTcpFree),
     plans(4, Reply::abort(m25::ErrorCodes::FRAME_TRANSFORM_FAILURE)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: FRAME_TRANSFORM_FAILURE during plan"},
    {"MF-07", "tcp free, FAILURE (99999) on every plan (FAULT)", waypoint(kTcpFree),
     plans(4, Reply::abort(m25::ErrorCodes::FAILURE)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: FAILURE during plan"},
    {"MF-07b", "tcp free, PREEMPTED on every plan: TMR never cancels (FAULT)", waypoint(kTcpFree),
     plans(4, Reply::preempted()), nullptr, nullptr, "FAILURE", "MOTION_FAULT: PREEMPTED during plan"},
  });
}

TEST(M27MotionFault, the_move_action_gone)
{
  m25::World& w = m25::world();
  runCases({
    {"MF-08", "tcp free, move_group's MoveGroup action gone after construction: no goal reaches it (FAULT)",
     waypoint(kTcpFree), nullptr, [&w] { w.mg.dropMoveAction(); },
     [&w] { w.mg.start(m25::FakeMoveGroup::MOVE_ACTION); }, "FAILURE", "MOTION_FAULT: FAILURE during plan", true},
  });
}

// --- R1/R2: the VALIDATE phase (TMR's TrajectoryValidator on the plan MoveIt returned) ------------------
TEST(M27MotionFault, validate_phase)
{
  runCases({
    {"MF-09", "joint ready, a 0.6 rad joint jump in the plan: this path (TARGET)", waypoint(kJointReady),
     plans(1, Reply::succeed().with(&Reply::joint_jump)), nullptr, nullptr, "FAILURE", ""},
    {"MF-10", "joint ready, an empty trajectory: a broken planner (FAULT)", waypoint(kJointReady),
     plans(1, Reply::succeed().with(&Reply::empty_trajectory)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: EMPTY_TRAJECTORY during validate"},
    {"MF-11", "joint ready, a point outside the joint limits: planner/model inconsistency (FAULT)",
     waypoint(kJointReady), plans(1, Reply::succeed().with(&Reply::out_of_limits)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: JOINT_LIMITS during validate"},
    {"MF-12", "joint ready, a decreasing time_from_start: broken time parameterisation (FAULT)",
     waypoint(kJointReady), plans(1, Reply::succeed().with(&Reply::decreasing_time)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: NON_MONOTONIC_TIME during validate"},
  });
}

// --- R1/R2: the EXECUTE phase (MoveGroupInterface::execute) ---------------------------------------------
TEST(M27MotionFault, execute_phase)
{
  m25::World& w = m25::world();
  runCases({
    {"MF-13", "tcp free, the execution aborts CONTROL_FAILED (FAULT)", waypoint(kTcpFree),
     executes(Reply::abort(m25::ErrorCodes::CONTROL_FAILED)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: CONTROL_FAILED during execute"},
    {"MF-14", "tcp free, the execution times out (FAULT)", waypoint(kTcpFree),
     executes(Reply::abort(m25::ErrorCodes::TIMED_OUT)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: TIMED_OUT during execute"},
    {"MF-15", "tcp free, the execution is preempted (FAULT)", waypoint(kTcpFree), executes(Reply::preempted()),
     nullptr, nullptr, "FAILURE", "MOTION_FAULT: PREEMPTED during execute"},
    {"MF-16", "tcp free, move_group's ExecuteTrajectory action gone after construction (FAULT)", waypoint(kTcpFree),
     nullptr, [&w] { w.mg.dropExecute(); }, [&w] { w.mg.start(m25::FakeMoveGroup::EXECUTE); }, "FAILURE",
     "MOTION_FAULT: FAILURE during execute"},
  });
}

// --- R3: circ, ONE Pilz sequence goal that plans AND executes ------------------------------------------
TEST(M27MotionFault, plan_and_execute_phase)
{
  m25::World& w = m25::world();
  runCases({
    {"MF-17", "circ, the sequence aborts NO_IK_SOLUTION (TARGET)", waypoint(kCirc),
     sequence(Reply::abort(m25::ErrorCodes::NO_IK_SOLUTION)), nullptr, nullptr, "FAILURE", ""},
    {"MF-18", "circ, the sequence aborts CONTROL_FAILED (FAULT)", waypoint(kCirc),
     sequence(Reply::abort(m25::ErrorCodes::CONTROL_FAILED)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: CONTROL_FAILED during plan_and_execute"},
    {"MF-19", "circ, the sequence aborts TIMED_OUT: the execution's (FAULT)", waypoint(kCirc),
     sequence(Reply::abort(m25::ErrorCodes::TIMED_OUT)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: TIMED_OUT during plan_and_execute"},
    {"MF-19b", "circ, the sequence is preempted (FAULT)", waypoint(kCirc), sequence(Reply::preempted()), nullptr,
     nullptr, "FAILURE", "MOTION_FAULT: PREEMPTED during plan_and_execute"},
    {"MF-20", "circ, the sequence goal is rejected (FAULT)", waypoint(kCirc), sequence(Reply::reject()), nullptr,
     nullptr, "FAILURE", "MOTION_FAULT: GOAL_REJECTED during plan_and_execute"},
    {"MF-21", "circ, the sequence aborts carrying SUCCESS: an inconsistent server (FAULT)", waypoint(kCirc),
     sequence(Reply::carrying(Reply::Status::ABORT, m25::ErrorCodes::SUCCESS)), nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: INCONSISTENT_RESULT during plan_and_execute"},
    {"MF-22", "circ, no sequence_move_group server: FAILURE after the 5 s wait, no goal (FAULT)", waypoint(kCirc),
     nullptr, [&w] { w.mg.dropSequence(); }, [&w] { w.mg.start(m25::FakeMoveGroup::SEQUENCE); }, "FAILURE",
     "MOTION_FAULT: SERVER_UNAVAILABLE during plan_and_execute", true},
    {"MF-23", "circ, the sequence succeeds carrying PLANNING_FAILED (TARGET)", waypoint(kCirc),
     sequence(Reply::carrying(Reply::Status::SUCCEED, m25::ErrorCodes::PLANNING_FAILED)), nullptr, nullptr,
     "FAILURE", ""},
  });
}

// --- MoveWaypoint's own refusals and an exception: the configuration's, never the target's ---------------
TEST(M27MotionFault, request_refusals_and_exceptions)
{
  runCases({
    {"MF-24", "tcp, motion=srvo: refused before MoveIt is asked", waypoint(join(kTcp, {{"motion", "srvo"}})),
     nullptr, nullptr, nullptr, "FAILURE", "MOTION_FAULT: UNKNOWN_MOTION during request", true},
    {"MF-25", "tcp, execution=servo: ServoTrack's, refused", waypoint(join(kTcp, {{"execution", "servo"}})), nullptr,
     nullptr, nullptr, "FAILURE", "MOTION_FAULT: SERVO_EXECUTION during request", true},
    {"MF-26", "tcp, execution=stream: unknown, refused", waypoint(join(kTcp, {{"execution", "stream"}})), nullptr,
     nullptr, nullptr, "FAILURE", "MOTION_FAULT: UNKNOWN_EXECUTION during request", true},
    {"MF-27", "an empty waypoint name: refused", waypoint({}), nullptr, nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: EMPTY_WAYPOINT during request", true, ""},
    {"MF-28", "tcp with no orientation: MoveWaypoint throws building the pose, caught", waypoint(
       {{"type", "tcp"}, {"position", "0.4;0.1;0.3"}, {"motion", "free"}}), nullptr, nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: EXCEPTION", true},
    {"MF-29", "circ by its centre (aux_is_center): the KNOWN_DEBT throw, caught, no goal",
     waypoint(join(kCirc, {{"aux_is_center", "true"}})), nullptr, nullptr, nullptr, "FAILURE",
     "MOTION_FAULT: EXCEPTION", true},
  });
}

// --- SUCCESS: nothing is written -----------------------------------------------------------------------
TEST(M27MotionFault, success_writes_nothing)
{
  runCases({
    {"MF-30", "tcp free, every answer SUCCEED", waypoint(kTcpFree), nullptr, nullptr, nullptr, "SUCCESS", ""},
    {"MF-31", "tcp lin, every answer SUCCEED", waypoint(kTcpLin), nullptr, nullptr, nullptr, "SUCCESS", ""},
  });
}

// The report of this process: both blocks of every case (evidence only).
TEST(M27MotionFault, zz_write_the_report)
{
  const std::filesystem::path dir(TRAINIT_M27_MF_REPORT_DIR);
  std::filesystem::create_directories(dir);
  std::ofstream out(dir / "motion_fault_report.txt", std::ios::binary | std::ios::trunc);
  out << "m2_7 motion_fault report (not a frozen capture)\n" << m25::environmentLine() << "\n\n" << reportText();
  EXPECT_FALSE(reportText().empty());
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
