// vNext M4.5d (ADR-0024 amendment M4.5d, owner-approved 2026-11-11): a motion that fails while a part is HELD is a
// FAULT whatever its cause. The REAL MoveWaypoint on a REAL MoveItMotionRuntime against the M2.5 fake move_group
// (test/m2_5/), as test_m2_7_motion_fault.cpp; every case is the SAME MoveIt script ticked as
//   <MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/>     with h = "true" / "false" / bool true
//   <MoveWaypoint waypoint="a" fault="{f}"/>                     (2.8: the port not remapped)
// What MoveIt receives must be identical whatever `holding` says (the port changes no traffic and no BT status);
// only the fault differs:
//   HELD-01  a PLANNING failure (PLANNING_FAILED: the target's, a verdict) while held  -> a FAULT "... while holding"
//   HELD-02  NO_IK_SOLUTION (the target's) while held                                  -> a FAULT
//   HELD-03  an EXECUTION failure (CONTROL_FAILED, already a fault) while held         -> the fault, "while holding"
//   HELD-04  a planning failure NOT held (holding false, and the port absent)          -> no fault (a verdict)
//   HELD-05  a SUCCESS while held                                                       -> nothing written
//   HELD-06  holding written as a BOOL by a Script (true)                               -> read as held
//   HELD-07  holding remapped to a key that holds no readable value                     -> fail closed: HELD
// Domain 98 (INTEGRATION 3.2), set in code and in CMake, localhost only, graph-guarded (m25::runAll).
#include <string>
#include <utility>
#include <vector>

#include "m2_5/motion_harness.hpp"

using m25::Reply;
using m25::Scenario;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

Strings data(const std::string& holding)
{
  Strings out = {{"a.type", "tcp"}, {"a.position", "0.4;0.1;0.3"}, {"a.orientation", "1;0;0;0"},
                 {"a.motion", "free"}, {"a.planner", "ompl"}, {"f", ""}};
  if (!holding.empty()) out.emplace_back("h", holding);
  return out;
}

std::function<void(m25::FakeMoveGroup&)> plans(int n, const Reply& r)
{
  return [n, r](m25::FakeMoveGroup& mg) { for (int i = 0; i < n; ++i) mg.mg_script.push_back(r); };
}

std::string reportFault(BT::Blackboard& bb)
{
  try { return "fault: \"" + bb.get<std::string>("f") + "\"\n"; }
  catch (const std::exception& e) { return std::string("fault: (unreadable: ") + e.what() + ")\n"; }
}

std::string lineValue(const std::string& block, const std::string& key)
{
  const auto a = block.find("\n" + key);
  if (a == std::string::npos) return "(no " + key + ")";
  const auto b = block.find('\n', a + 1 + key.size());
  return block.substr(a + 1 + key.size(), b - a - 1 - key.size());
}

std::string eventsOf(const std::string& block)
{
  const auto a = block.find("\nevents:\n");
  const auto b = block.find("\nbt: ");
  return (a == std::string::npos || b == std::string::npos) ? "(none)" : block.substr(a, b - a);
}

struct Case
{
  std::string id, pins;
  Strings bb;
  std::function<void(m25::FakeMoveGroup&)> script;
  std::string tree;
  std::string bt;
  std::string fault;
};

std::string run(const Case& c)
{
  m25::World& w = m25::world();
  const std::string out = m25::run(w, Scenario{c.id, c.pins, nullptr, c.bb, {}, {c.tree}, c.script, nullptr, nullptr,
                                               {0.0, 0.0, 0.0}, true, 90, reportFault});
  EXPECT_EQ(lineValue(out, "bt: "), c.bt) << c.id;
  EXPECT_EQ(lineValue(out, "fault: "), "\"" + c.fault + "\"") << c.id;
  return out;
}

const char* kHeld = R"(<MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/>)";
const char* kPlain = R"(<MoveWaypoint waypoint="a" fault="{f}"/>)";
}  // namespace

TEST(M45dHeldMotionFault, a_target_failure_while_held_is_a_fault)
{
  const auto pf = plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED));
  const std::string held = run({"HELD-01", "PLANNING_FAILED, held", data("true"), pf, kHeld, "FAILURE",
                                "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  const std::string free_ = run({"HELD-04a", "PLANNING_FAILED, not held", data("false"), pf, kHeld, "FAILURE", ""});
  const std::string plain = run({"HELD-04b", "PLANNING_FAILED, the 2.8 tree", data(""), pf, kPlain, "FAILURE", ""});
  // the port changes nothing MoveIt receives
  EXPECT_EQ(eventsOf(held), eventsOf(free_));
  EXPECT_EQ(eventsOf(held), eventsOf(plain));
  run({"HELD-02", "NO_IK_SOLUTION, held", data("true"), plans(4, Reply::abort(m25::ErrorCodes::NO_IK_SOLUTION)),
       kHeld, "FAILURE", "MOTION_FAULT: NO_IK_SOLUTION during plan while holding"});
}

TEST(M45dHeldMotionFault, an_execution_failure_while_held_says_so)
{
  auto script = [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::abort(m25::ErrorCodes::CONTROL_FAILED)); };
  run({"HELD-03", "CONTROL_FAILED on execution, held", data("true"), script, kHeld, "FAILURE",
       "MOTION_FAULT: CONTROL_FAILED during execute while holding"});
  run({"HELD-03b", "CONTROL_FAILED on execution, not held (2.8: already a fault)", data("false"), script, kHeld,
       "FAILURE", "MOTION_FAULT: CONTROL_FAILED during execute"});
}

TEST(M45dHeldMotionFault, success_and_the_ways_holding_is_written)
{
  run({"HELD-05", "a success while held writes nothing", data("true"),
       nullptr,
       kHeld, "SUCCESS", ""});
  // a canonical tree writes the key with a Script: a BOOL value
  const std::string script_tree =
      R"(<Sequence><Script code="h:=true"/><MoveWaypoint waypoint="a" fault="{f}" holding="{h}"/></Sequence>)";
  run({"HELD-06", "holding written as a bool by a Script", data(""),
       plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED)), script_tree, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
  // remapped to a key holding a value that is not a boolean: fail closed (HELD)
  run({"HELD-07", "holding unreadable ('maybe')", data("maybe"),
       plans(4, Reply::abort(m25::ErrorCodes::PLANNING_FAILED)), kHeld, "FAILURE",
       "MOTION_FAULT: PLANNING_FAILED during plan while holding"});
}

int main(int argc, char** argv) { return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN); }
