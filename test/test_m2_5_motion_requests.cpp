// vNext M2.5 (tests first): what every planned-motion BT node and TaskRunner sends to MoveIt today.
//
// Frozen on TMR 1.8.0 (9cfb601) before the MotionRuntime seam touched a production line; every
// later commit must reproduce these captures byte for byte (test/data/m2_5/<family>.txt). Each
// family is one gtest; each scenario runs on a FRESH MoveItMotionRuntime against the fakes of
// test/m2_5/. Known defects are PINNED here, not fixed (owner brief M2.5 sections 10-12): the
// planner-mode drift, the CHOMP->OMPL fallback, the planner port that is a description, the
// ptp->PILZ line, the stale target, the cartesian frame/EE leak, the CIRC-centre throw.
#include <memory>
#include <string>
#include <vector>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/gripper_controller.hpp"
#include "trainit_motion_runtime/task_loader.hpp"
#include "trainit_motion_runtime/task_runner.hpp"

using m25::Config;
using m25::Reply;
using m25::Scenario;
using trainit::PlannerMode;

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

const geometry_msgs::msg::Pose kP1 = m25::pose(0.40, 0.10, 0.30, 1.0, 0.0, 0.0, 0.0);
const geometry_msgs::msg::Pose kP2 = m25::pose(0.45, 0.15, 0.30, 1.0, 0.0, 0.0, 0.0);

std::function<void(Config&)> mode(PlannerMode m)
{
  return [m](Config& c) { c.mode = m; };
}
std::function<void(Config&)> modeAndPipelines(PlannerMode m, std::vector<std::string> p)
{
  return [m, p](Config& c) { c.mode = m; c.available_pipelines = p; };
}
std::function<void(Config&)> cartesianBackend()
{
  return [](Config& c) { c.process_backend = trainit::ProcessPathBackend::CARTESIAN_WAYPOINTS; };
}

// A data-driven waypoint `<name>.<field>`, written the way trainit_run_bt writes bt_params:
// strings verbatim, integers by std::to_string, double arrays joined by ';' at ostream precision.
Strings waypoint(const std::string& name, const Strings& fields)
{
  Strings out;
  for (const auto& [k, v] : fields) out.emplace_back(name + "." + k, v);
  return out;
}
Strings join(Strings a, const Strings& b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}
const Strings kTcpA = {{"type", "tcp"}, {"position", "0.4;0.1;0.3"}, {"orientation", "1;0;0;0"}};

void family(const std::string& name, const std::vector<Scenario>& scenarios, bool bodies = true)
{
  m25::World& w = m25::world();
  w.log.setBodies(bodies);
  m25::Family f(name);
  for (const auto& s : scenarios) f.add(m25::run(w, s));
  w.log.setBodies(true);
  f.check();
}
}  // namespace

// --- PTP: MovePtp, MoveToNamed, MoveToJoint -> MotionRuntime::movePtp -----------------------------
TEST(M25MotionRequests, planned_ptp)
{
  family("planned_ptp", {
    {"PTP-01", "MoveToNamed: Pilz PTP joint goal, transit profile, then GetPlanningScene, then execute",
     mode(PlannerMode::PILZ), {}, {}, {"<MoveToNamed target=\"ready\"/>"}},
    {"PTP-02", "speed/accel percent overrides scale the transit profile",
     mode(PlannerMode::PILZ), {}, {}, {"<MoveToNamed target=\"ready\" speed=\"80\" accel=\"30\"/>"}},
    {"PTP-03", "MovePtp named with the approach profile",
     mode(PlannerMode::PILZ), {}, {}, {"<MovePtp named=\"home\" profile=\"approach\"/>"}},
    {"PTP-04", "MoveToJoint and MovePtp joints send identical requests",
     mode(PlannerMode::PILZ), {}, {},
     {"<MoveToJoint joints=\"0.1;-0.2;0.3\"/>", "<MovePtp joints=\"0.1;-0.2;0.3\"/>"}},
    {"PTP-05", "an unknown named state fails before any request",
     mode(PlannerMode::PILZ), {}, {}, {"<MoveToNamed target=\"nope\"/>"}},
    {"PTP-06", "a wrong-length joint vector on a fresh runtime; an empty one fails before any request",
     mode(PlannerMode::PILZ), {}, {},
     {"<MoveToJoint joints=\"0.1;0.2\"/>", "<MoveToJoint joints=\"\"/>"}},
    {"PTP-07a", "movePtp follows the mode: ompl",
     mode(PlannerMode::OMPL), {}, {}, {"<MoveToNamed target=\"ready\"/>"}},
    {"PTP-07b", "movePtp follows the mode: ompl_chomp (CHOMP first on a joint goal)",
     mode(PlannerMode::OMPL_CHOMP), {}, {}, {"<MoveToNamed target=\"ready\"/>"}},
  });
}

// --- LIN and CIRC: MoveLinear -> moveLinear (MGI), MoveCircular -> Pilz sequence ---------------------
TEST(M25MotionRequests, planned_lin_circ)
{
  m25::World& w = m25::world();
  family("planned_lin_circ", {
    {"LIN-01", "MoveLinear: Pilz LIN on /move_action, tcp in base_link, approach profile",
     nullptr, {}, {{"p1", kP1}}, {"<MoveLinear pose_key=\"p1\"/>"}},
    {"LIN-02", "MoveLinear tip_link=l3 sets the goal link",
     nullptr, {}, {{"p1", kP1}}, {"<MoveLinear pose_key=\"p1\" tip_link=\"l3\"/>"}},
    {"LIN-03", "a missing pose_key fails before any request",
     nullptr, {}, {}, {"<MoveLinear pose_key=\"absent\"/>"}},
    {"CIRC-01", "MoveCircular: one sequence goal, one CIRC item with its interim point",
     nullptr, {}, {{"p2", kP2}}, {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"}},
    {"CIRC-02", "MoveCircular tip_link=l3 is ignored: the item's link stays the runtime tip",
     nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\" tip_link=\"l3\"/>"}},
    {"CIRC-03", "aux_is_center=true throws out of the tick (KNOWN_DEBT: the CIRC-centre throw)",
     nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.30\" aux_is_center=\"true\"/>"}},
    {"CIRC-04", "no sequence_move_group server: FAILURE after the 5 s wait, no goal",
     nullptr, {}, {{"p2", kP2}}, {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     nullptr, [&w] { w.mg.dropSequence(); }, [&w] { w.mg.start(m25::FakeMoveGroup::SEQUENCE); }},
  });
}

// --- MoveWaypoint: the data-driven node a generated bundle uses ------------------------------------
TEST(M25MotionRequests, planned_waypoint)
{
  family("planned_waypoint", {
    {"WP-LIN-03", "motion=lin: approach profile, speed 50 -> velocity 0.5",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "lin"}, {"speed", "50"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-LIN-04", "motion=linear (the alias) takes the TRANSIT profile (KNOWN_DEBT)",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "linear"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-LIN-05", "lin to a named state: FK in the model frame, sent as a base_link pose",
     nullptr, waypoint("a", {{"type", "joint"}, {"named", "ready"}, {"motion", "lin"}}), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-LIN-06", "lin with neither a position nor a named state fails before any request",
     nullptr, waypoint("a", {{"type", "tcp"}, {"motion", "lin"}}), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-CIRC-04", "circ with aux_is_center: the throw is caught, FAILURE, no goal",
     nullptr,
     waypoint("a", join(kTcpA, {{"motion", "circ"}, {"aux", "0.43;0.12;0.30"}, {"aux_is_center", "true"}})),
     {}, {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-CIRC-05", "circ with an interim point: one CIRC item, approach profile",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "circ"}, {"aux", "0.43;0.12;0.32"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-01", "free tcp, planner ompl",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", "ompl"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-02", "free tcp, planner pilz: Pilz PTP to a pose",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", "pilz"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-03", "ptp + blank planner -> Pilz PTP, and the runtime mode drifts to pilz",
     nullptr, waypoint("a", {{"type", "joint"}, {"named", "ready"}, {"motion", "ptp"}}), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-04", "free joints, planner ompl",
     nullptr,
     waypoint("a", {{"type", "joint"}, {"joints", "0.1;-0.2;0.3"}, {"motion", "free"}, {"planner", "ompl"}}),
     {}, {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-07", "free tcp ompl_chomp: CHOMP refuses the pose (-16), OMPL plans, OMPL's plan runs",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", "ompl_chomp"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-08", "an unknown planner falls back to the runtime mode",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", "bogus"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
    {"WP-FREE-09", "execution=servo is refused before any request; execution=plan plans",
     nullptr,
     join(waypoint("s", join(kTcpA, {{"execution", "servo"}})),
          waypoint("p", join(kTcpA, {{"execution", "plan"}, {"planner", "ompl"}}))),
     {}, {"<MoveWaypoint waypoint=\"s\"/>", "<MoveWaypoint waypoint=\"p\"/>"}},
    {"WP-FREE-10", "an unknown motion is refused before any request",
     nullptr, waypoint("a", join(kTcpA, {{"motion", "linaer"}})), {},
     {"<MoveWaypoint waypoint=\"a\"/>"}},
  });
}

// --- MoveCollisionFree / MoveFree ------------------------------------------------------------------
TEST(M25MotionRequests, planned_free)
{
  family("planned_free", {
    {"FREE-05a", "no planner attribute -> the current mode (launch ompl): 'pilz' is a description",
     mode(PlannerMode::OMPL), {}, {{"p1", kP1}}, {"<MoveCollisionFree pose_key=\"p1\"/>"}},
    {"FREE-05b", "no planner attribute -> the current mode (launch pilz)",
     mode(PlannerMode::PILZ), {}, {{"p1", kP1}}, {"<MoveCollisionFree pose_key=\"p1\"/>"}},
    {"FREE-06", "MoveFree sends what MoveCollisionFree sends",
     mode(PlannerMode::OMPL), {}, {{"p1", kP1}},
     {"<MoveCollisionFree pose_key=\"p1\" planner=\"ompl\"/>", "<MoveFree pose_key=\"p1\" planner=\"ompl\"/>"}},
    {"FREE-10", "named and joint targets",
     mode(PlannerMode::OMPL), {}, {},
     {"<MoveCollisionFree named=\"ready\" planner=\"ompl\"/>",
      "<MoveCollisionFree joints=\"0.1;-0.2;0.3\" planner=\"ompl\"/>"}},
    {"FREE-11", "no target fails before any request",
     mode(PlannerMode::OMPL), {}, {}, {"<MoveCollisionFree planner=\"ompl\"/>"}},
  });
}

// --- process paths: ExecuteProcessPath -> executeCartesianProcess -----------------------------------
TEST(M25MotionRequests, process_path)
{
  family("process_path", {
    {"PATH-01", "pilz_sequence + CSV: one sequence goal, three LIN items in base_link/tcp",
     nullptr, {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"}},
    {"PATH-02", "pilz_sequence + YAML naming world/l3: the items stay base_link/tcp",
     nullptr, {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>"}},
    {"PATH-03", "cartesian_waypoints + YAML world/l3: computeCartesianPath then the TOTG+Ruckig execute",
     cartesianBackend(), {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>"}},
    {"PATH-04", "cartesian_waypoints + CSV: base_link/tcp",
     cartesianBackend(), {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"}},
    {"PATH-05", "a one-waypoint CSV fails to load: no request; the next LIN is still in base_link",
     cartesianBackend(), {}, {{"p1", kP1}},
     {"<ExecuteProcessPath path_file=\"${PATHS}/p1.csv\"/>", "<MoveLinear pose_key=\"p1\"/>"}},
    {"PATH-06", "a one-waypoint YAML is refused by the cartesian backend before it touches MoveIt",
     cartesianBackend(), {}, {{"p1", kP1}},
     {"<ExecuteProcessPath path_file=\"${PATHS}/p1_world.yaml\"/>", "<MoveLinear pose_key=\"p1\"/>"}},
  });
}

// --- TaskRunner: every step kind, executed and plan-only ----------------------------------------------
namespace
{
std::string runTask(const std::string& id, const std::string& pins, const std::string& file,
                    bool plan_only)
{
  m25::World& w = m25::world();
  m25::FakeGripper gripper(w.log, "/", "m25_gripper");
  Config cfg = m25::defaultConfig();
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.mg.resetScenario();
  w.dog.arm("build " + id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();
  trainit::GripperController gc(w.rig.node(), "m25_gripper");
  trainit::TaskRunner runner(*w.rig.runtime(), &gc, nullptr);
  trainit::MotionTask task = trainit::TaskLoader::fromYamlFile(std::string(TRAINIT_M25_TASKS_DIR) + "/" + file);
  if (plan_only)
    for (auto& step : task.steps) step.options.plan_only = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // the gripper client is discovered
  w.log.reset();
  w.dog.arm(id, std::chrono::seconds(120));
  std::string result;
  try { result = runner.run(task) ? "true" : "false"; }
  catch (const std::exception& e) { result = "THROW " + m25::demangle(typeid(e).name()) + ": " + e.what(); }
  m25::settleTopics(w.log);
  w.dog.disarm();
  std::ostringstream os;
  os << "=== scenario " << id << "\n"
     << "pins: " << pins << "\n"
     << "config: ns=/ " << m25::describe(cfg) << "\n"
     << "state: " << w.js.describe() << "\n"
     << "task: " << file << (plan_only ? " (plan_only on every step)" : "") << "\n"
     << "events:\n" << w.log.events();
  const std::string topics = w.log.topics();
  if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;
  os << "run: " << result << "\n"
     << "planner_mode_after: " << trainit::toString(w.rig.runtime()->plannerMode()) << "\n"
     << "counts: " << w.log.counts() << "\n\n";
  return os.str();
}
}  // namespace

TEST(M25MotionRequests, task_runner)
{
  m25::Family f("task_runner");
  f.add(runTask("TASK-01a", "every TaskRunner step kind, executed", "m2_5_task.yaml", false));
  f.add(runTask("TASK-01b", "the same task with plan_only on every step", "m2_5_task.yaml", true));
  f.add(runTask("TASK-02", "a CIRCULAR step given by its centre throws out of TaskRunner::run",
                "m2_5_task_center.yaml", false));
  f.check();
}

// --- T3 planner identity: what each mode x pipeline set resolves to, per move kind -------------------
TEST(M25MotionRequests, planner_identity)
{
  const std::vector<std::pair<std::string, std::vector<std::string>>> pipelines = {
    {"default", {"ompl", "pilz_industrial_motion_planner", "chomp"}},
    {"no_chomp", {"ompl", "pilz_industrial_motion_planner"}},
    {"default+stomp", {"ompl", "pilz_industrial_motion_planner", "chomp", "stomp"}},
    {"empty", {}},
  };
  const std::vector<PlannerMode> modes = {PlannerMode::PILZ, PlannerMode::OMPL,
                                          PlannerMode::OMPL_CHOMP, PlannerMode::STOMP};
  const Strings data = join(join(waypoint("free", join(kTcpA, {{"motion", "free"}})),
                                 waypoint("ptp", {{"type", "joint"}, {"joints", "0.1;-0.2;0.3"}, {"motion", "ptp"}})),
                            {});
  std::vector<Scenario> s;
  for (const auto& m : modes)
    for (const auto& [pname, pset] : pipelines)
      s.push_back({"T3-01 " + trainit::toString(m) + " x " + pname,
                   "launch mode x available pipelines; ptp-blank last (it drifts the mode)",
                   modeAndPipelines(m, pset), data, {{"p1", kP1}, {"p2", kP2}},
                   {"<MoveToNamed target=\"ready\"/>", "<MoveLinear pose_key=\"p1\"/>",
                    "<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>",
                    "<MoveWaypoint waypoint=\"free\"/>", "<MoveWaypoint waypoint=\"ptp\"/>"}});
  for (const std::string planner : {"pilz", "ompl", "ompl_chomp", "stomp", "bogus", ""})
  {
    s.push_back({"T3-02 free tcp planner='" + planner + "'", "a per-waypoint planner on a free pose goal",
                 nullptr, waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", planner}})), {},
                 {"<MoveWaypoint waypoint=\"a\"/>"}});
    s.push_back({"T3-02 ptp joint planner='" + planner + "'", "a per-waypoint planner on a ptp joint goal",
                 nullptr,
                 waypoint("a", {{"type", "joint"}, {"joints", "0.1;-0.2;0.3"}, {"motion", "ptp"}, {"planner", planner}}),
                 {}, {"<MoveWaypoint waypoint=\"a\"/>"}});
  }
  s.push_back({"T3-03", "ompl_chomp: CHOMP plans a joint goal; refuses a pose goal, OMPL plans it",
               mode(PlannerMode::OMPL_CHOMP), {}, {{"p1", kP1}},
               {"<MoveCollisionFree named=\"ready\"/>", "<MoveCollisionFree pose_key=\"p1\"/>"}});
  s.push_back({"T3-04", "ompl_chomp with every plan failing: 4 attempts x (CHOMP, OMPL) = 8 goals",
               mode(PlannerMode::OMPL_CHOMP), {}, {}, {"<MoveCollisionFree named=\"ready\"/>"},
               [](m25::FakeMoveGroup& mg) { for (int i = 0; i < 8; ++i) mg.mg_script.push_back(Reply::abort(-1)); }});
  s.push_back({"T3-05", "ompl_chomp asked for with no chomp pipeline: plain OMPL, mode ompl",
               modeAndPipelines(PlannerMode::OMPL, {"ompl", "pilz_industrial_motion_planner"}),
               waypoint("a", join(kTcpA, {{"motion", "free"}, {"planner", "ompl_chomp"}})), {},
               {"<MoveWaypoint waypoint=\"a\"/>"}});
  family("planner_identity", s, false);
}

// --- T4 failure mapping: a MoveIt answer -> a BT status ------------------------------------------------
TEST(M25MotionRequests, failure_mapping)
{
  m25::World& w = m25::world();
  auto ready = std::vector<std::string>{"<MoveToNamed target=\"ready\"/>"};
  auto fail = [](int n, int code) {
    return [n, code](m25::FakeMoveGroup& mg) { for (int i = 0; i < n; ++i) mg.mg_script.push_back(Reply::abort(code)); };
  };
  family("failure_mapping", {
    {"T4-01", "planning fails every time: 4 identical goals (plan_retries 3), no execute, FAILURE",
     nullptr, {}, {}, ready, fail(4, -1)},
    {"T4-02", "planning fails once then succeeds: the second plan is the one executed",
     nullptr, {}, {}, ready, fail(1, -1)},
    {"T4-03", "an empty trajectory fails validation: no retry, no execute",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { mg.mg_script.push_back(Reply::succeed().with(&Reply::empty_trajectory)); }},
    {"T4-04a", "a 0.6 rad joint jump fails validation",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { mg.mg_script.push_back(Reply::succeed().with(&Reply::joint_jump)); }},
    {"T4-04b", "the same jump with enforce_validation=false is executed",
     [](Config& c) { c.enforce_validation = false; }, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { mg.mg_script.push_back(Reply::succeed().with(&Reply::joint_jump)); }},
    {"T4-04c", "a point outside the joint limits fails validation",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { mg.mg_script.push_back(Reply::succeed().with(&Reply::out_of_limits)); }},
    {"T4-04d", "a decreasing time_from_start fails validation",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { mg.mg_script.push_back(Reply::succeed().with(&Reply::decreasing_time)); }},
    {"T4-06a", "TIMED_OUT (-6), the emulated planning timeout: 4 goals, FAILURE",
     nullptr, {}, {}, ready, fail(4, -6)},
    {"T4-06b", "PREEMPTED (-7): 4 goals, FAILURE",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) { for (int i = 0; i < 4; ++i) mg.mg_script.push_back(Reply::preempted()); }},
    {"T4-06c", "INVALID_GOAL_CONSTRAINTS (-16) and NO_IK_SOLUTION (-31)",
     nullptr, {}, {}, ready,
     [](m25::FakeMoveGroup& mg) {
       for (int c : {-16, -31, -16, -31}) mg.mg_script.push_back(Reply::abort(c));
     }},
    {"T4-07a", "execution aborts CONTROL_FAILED (-4): FAILURE, execution never retried",
     nullptr, {}, {}, ready, [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::abort(-4)); }},
    {"T4-07b", "execution TIMED_OUT (-6)",
     nullptr, {}, {}, ready, [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::abort(-6)); }},
    {"T4-07c", "execution PREEMPTED (-7)",
     nullptr, {}, {}, ready, [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::preempted()); }},
    {"T4-09a", "/move_action gone after construction: no goal reaches it, FAILURE",
     nullptr, {}, {}, ready, nullptr, [&w] { w.mg.dropMoveAction(); },
     [&w] { w.mg.start(m25::FakeMoveGroup::MOVE_ACTION); }},
    {"T4-09b", "/execute_trajectory gone after construction: planned, validated, not executed, FAILURE",
     nullptr, {}, {}, ready, nullptr, [&w] { w.mg.dropExecute(); },
     [&w] { w.mg.start(m25::FakeMoveGroup::EXECUTE); }},
    {"T4-11a", "sequence ABORT (-1)", nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::abort(-1)); }},
    {"T4-11b", "sequence PREEMPTED (-7)", nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::preempted()); }},
    {"T4-11c", "sequence REJECTED", nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::reject()); }},
    {"T4-11d", "sequence SUCCEEDED carrying -1: FAILURE", nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::carrying(Reply::Status::SUCCEED, -1)); }},
    {"T4-11e", "sequence ABORTED carrying SUCCESS: FAILURE", nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(Reply::carrying(Reply::Status::ABORT, 1)); }},
    {"T4-12a", "cartesian fraction 0.6 under threshold 1.0: FAILURE, no execute",
     cartesianBackend(), {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"},
     [](m25::FakeMoveGroup& mg) { Reply r; r.fraction = 0.6; mg.cart_script.push_back(r); }},
    {"T4-12b", "cartesian error -1: FAILURE, no execute",
     cartesianBackend(), {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.cart_script.push_back(Reply::abort(-1)); }},
    {"T4-12c", "cartesian OK, execution aborts -4: FAILURE",
     cartesianBackend(), {}, {}, {"<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>"},
     [](m25::FakeMoveGroup& mg) { mg.ex_script.push_back(Reply::abort(-4)); }},
  }, false);
}

// --- T5 sequences: state a move leaves behind for the next one (pinned, not fixed) ---------------------
TEST(M25MotionRequests, sequences)
{
  const Strings free_chomp = waypoint("fc", join(kTcpA, {{"motion", "free"}, {"planner", "ompl_chomp"}}));
  const Strings free_blank = waypoint("fb", join(kTcpA, {{"motion", "free"}}));
  const Strings ptp_named = waypoint("pn", {{"type", "joint"}, {"named", "ready"}, {"motion", "ptp"}});
  family("sequences", {
    {"T5-01", "the drift: free ompl_chomp, then a blank free move starts with CHOMP",
     mode(PlannerMode::OMPL), join(free_chomp, free_blank), {},
     {"<MoveWaypoint waypoint=\"fc\"/>", "<MoveWaypoint waypoint=\"fb\"/>"}},
    {"T5-02", "the drift: ptp+blank writes pilz, then a blank free pose move is Pilz PTP",
     mode(PlannerMode::OMPL), join(ptp_named, free_blank), {},
     {"<MoveWaypoint waypoint=\"pn\"/>", "<MoveWaypoint waypoint=\"fb\"/>"}},
    {"T5-03", "the drift: MoveCollisionFree planner=ompl under launch pilz, then MoveToNamed plans OMPL",
     mode(PlannerMode::PILZ), {}, {},
     {"<MoveCollisionFree named=\"ready\" planner=\"ompl\"/>", "<MoveToNamed target=\"home\"/>"}},
    {"T5-04", "the drift survives a failure: a failed ompl_chomp move, then a blank one starts with CHOMP",
     mode(PlannerMode::OMPL), join(free_chomp, free_blank), {},
     {"<MoveWaypoint waypoint=\"fc\"/>", "<MoveWaypoint waypoint=\"fb\"/>"},
     [](m25::FakeMoveGroup& mg) { for (int i = 0; i < 8; ++i) mg.mg_script.push_back(Reply::abort(-1)); }},
    {"T5-05", "launch ompl_chomp without a chomp pipeline is ompl from the start",
     modeAndPipelines(PlannerMode::OMPL_CHOMP, {"ompl", "pilz_industrial_motion_planner"}), free_blank, {},
     {"<MoveWaypoint waypoint=\"fb\"/>"}},
    {"T5-06", "the cartesian leak: a YAML path in world/l3 leaves the MGI pose frame and EE link behind",
     cartesianBackend(), {}, {{"p1", kP1}},
     {"<MoveLinear pose_key=\"p1\"/>", "<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>",
      "<MoveLinear pose_key=\"p1\"/>", "<ExecuteProcessPath path_file=\"${PATHS}/p3.csv\"/>",
      "<MoveLinear pose_key=\"p1\"/>"}},
    {"T5-07", "the stale target: a refused joint vector or named state re-sends the previous goal",
     mode(PlannerMode::OMPL),
     join(waypoint("bad", {{"type", "joint"}, {"joints", "1;2"}, {"motion", "free"}, {"planner", "ompl"}}),
          waypoint("nope", {{"type", "joint"}, {"named", "nope"}, {"motion", "ptp"}, {"planner", "ompl"}})),
     {},
     {"<MoveToJoint joints=\"0.1;-0.2;0.3\"/>", "<MoveWaypoint waypoint=\"bad\"/>",
      "<MoveWaypoint waypoint=\"nope\"/>", "<MoveToNamed target=\"nope\"/>"}},
    {"T5-08", "a joint move after a cartesian path",
     cartesianBackend(), {}, {},
     {"<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>", "<MoveToJoint joints=\"0.1;-0.2;0.3\"/>"}},
    {"T5-09", "two identical MoveCircular moves send identical goals",
     nullptr, {}, {{"p2", kP2}},
     {"<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>", "<MoveCircular goal_key=\"p2\" aux=\"0.43;0.12;0.32\"/>"}},
  });
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
