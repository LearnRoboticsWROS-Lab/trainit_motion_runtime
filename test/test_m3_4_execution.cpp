// test_m3_4_execution (vNext M3.4, ADR-0015 amendment M3.4): what TMR sends to MoveIt for every
// behaviour M3.4 hardens (owner decision D3), ticked for REAL -- BT nodes, MoveItMotionRuntime, the
// Pilz sequence and Cartesian backends -- against the M2.5 in-process fake move_group, on a private
// domain (84).
//
// The SAME scenarios are recorded twice: BEFORE, from the unmodified TMR 2.2.0 production sources
// (9418694, test/data/m3_4/before/), and AFTER, from the M3.4 production (test/data/m3_4/after/).
// test_m3_4_capture_diff proves the two differ only in the scenarios each M3.4 change names, and how.
// This file compiles against both productions: what only TMR >= 2.3.0 has is reached through a
// detection (HasSequenceTimeout), never named directly.
//
// What it pins, by family:
//   TOL   a canonical move's goal tolerance (a named `profile` that carries one) reaching MoveGroup on
//         the MoveGroupInterface routes; no leak into the next move; the legacy route unchanged; a
//         Cartesian tolerance on a joint target, and an invalid one, refused
//   PLN   an unknown planner token: a loud failure, never the runtime mode; a known one unchanged
//   CIRC  the via form unchanged; the centre form (MoveWaypoint, MoveCircular); a non-finite or
//         degenerate circle point refused before MoveIt
//   TIP   CircularTarget.tip_link in the sequence item
//   RST   the Cartesian backend's pose reference frame and end-effector link after a run
//   ST    a state read with no robot state: a failure, never the identity pose
//   TO    a Pilz sequence goal whose result times out is cancelled
//   STOP  MotionRuntime::stop() sends move_group's stop event (unchanged; the runner wiring is
//         test_m3_4_runner_stop's)
//   ALS   the motion aliases linear / circular take the approach profile
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/cartesian_waypoint_backend.hpp"
#include "trainit_motion_runtime/path_library.hpp"

namespace
{
using trainit::PathLibrary;
using trainit::PathRuntimeFacts;
using trainit::ProcessPathBackend;

const std::string kData = TRAINIT_M34_DATA_DIR;
const std::string kM31 = TRAINIT_M31_DATA_DIR;

// --- what only TMR >= 2.3.0 has ----------------------------------------------------------------------
// MoveItMotionRuntime::Config::sequence_result_timeout_s (vNext M3.4): a 2 s result timeout keeps the
// TO scenario short; the 2.2.0 production has the fixed 120 s (the BEFORE run waits for it).
template <typename C, typename = void>
struct HasSequenceTimeout : std::false_type {};
template <typename C>
struct HasSequenceTimeout<C, std::void_t<decltype(std::declval<C&>().sequence_result_timeout_s)>>
: std::true_type {};
template <typename C>
void shortSequenceTimeout(C& c)
{
  if constexpr (HasSequenceTimeout<C>::value) c.sequence_result_timeout_s = 2.0;
  else (void)c;
}

// TF for the rig (as test_m3_1_follow_path): world -> base_link, the test arm's fixed joint.
struct Tf
{
  explicit Tf(const rclcpp::Node::SharedPtr& node)
  {
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, true);
    pub_node = std::make_shared<rclcpp::Node>("m34_static_tf", "/");
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(pub_node);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.z = 0.1;
    t.transform.rotation.w = 1.0;
    broadcaster->sendTransform(t);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline &&
           !buffer->canTransform("base_link", "world", tf2::TimePointZero))
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Node::SharedPtr pub_node;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};

Tf& tf()
{
  static std::unique_ptr<Tf> t;   // test harness: one per process, torn down with the world
  if (!t)
  {
    t = std::make_unique<Tf>(m25::world().rig.node());
    m25::teardowns().push_back([] { t.reset(); });
  }
  return *t;
}

using Strings = std::vector<std::pair<std::string, std::string>>;
using Poses = std::vector<std::pair<std::string, geometry_msgs::msg::Pose>>;

struct Case
{
  std::string id;
  std::string pins;
  std::vector<std::string> trees{};                     // ticked in order on ONE runtime
  Strings strings{};
  Poses poses{};
  std::function<void(m25::Config&)> config{};
  std::function<void(m25::FakeMoveGroup&)> script{};
  bool joint_states{true};
  bool paths{false};                                    // the M3.1 path library (test/data/m3_1)
  int budget_s{90};
  // instead of trees: one direct call on the runtime (or a backend), whose result is the text
  std::function<std::string(m25::World&)> action{};
  // the AFTER outcome of each tick, "<bt>|<fault>" (asserted only when built for the M3.4 production)
  std::vector<std::string> after{};
  // runs on its own thread while the tick runs (e.g. the arm arriving late); joined after it
  std::function<void(m25::World&)> during{};
  std::vector<double> q{0.0, 0.0, 0.0};                 // the joint states at the start
};

m25::Family& family()
{
#ifdef TRAINIT_M34_AFTER
  static m25::Family f("m3_4_execution", kData + "/after");    // test harness: one per process
#else
  static m25::Family f("m3_4_execution", kData + "/before");
#endif
  return f;
}

std::shared_ptr<const PathLibrary> library()
{
  return std::make_shared<const PathLibrary>(
    trainit::loadPathLibrary(kM31 + "/paths.yaml", PathRuntimeFacts{ProcessPathBackend::PILZ_SEQUENCE, "tcp"}));
}

std::string tickOnce(m25::World& w, const std::string& xml, const BT::Blackboard::Ptr& bb, bool paths)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  trainit::BtContext ctx = m25::makeContext(w.rig.runtime(), w.rig.node());
  ctx.tf = tf().buffer;
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES,
                                          trainit::loadMotionProfiles(kData + "/motion_profiles.yaml"));
  if (paths) bb->set<std::shared_ptr<const PathLibrary>>(trainit::BB_PATH_LIBRARY, library());
  try
  {
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         m25::substitute(xml) + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return BT::toStr(status);
  }
  catch (const std::exception& e)
  {
    return "THROW " + m25::demangle(typeid(e).name()) + ": " + e.what();
  }
}

std::string storedPose(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  try { return "Pose" + m25::describe(bb->get<geometry_msgs::msg::Pose>(key)); }
  catch (const std::exception&) { return "(none)"; }
}

void runCase(const Case& c)
{
  m25::World& w = m25::world();
  tf();
  m25::Config cfg = m25::defaultConfig();
  if (c.config) c.config(cfg);
  w.js.set(c.q);
  w.js.on(c.joint_states);
  w.mg.resetScenario();
  if (c.script) c.script(w.mg);
  w.dog.arm("build " + c.id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();

  auto bb = BT::Blackboard::create();
  for (const auto& [k, v] : c.strings) bb->set<std::string>(k, v);
  for (const auto& [k, p] : c.poses) bb->set<geometry_msgs::msg::Pose>(k, p);

  std::ostringstream os;
  os << "=== scenario " << c.id << "\n"
     << "pins: " << c.pins << "\n"
     << "config: " << m25::describe(cfg) << " profiles=test/data/m3_4/motion_profiles.yaml\n"
     << "state: " << w.js.describe() << (c.during ? " (changed during the tick: see pins)" : "") << "\n"
     << "blackboard: " << m25::blackboardLine({c.id, "", {}, c.strings, c.poses}) << "\n"
     << "paths: " << (c.paths ? "test/data/m3_1/paths.yaml" : "(no library)") << "\n"
     << "script: " << w.mg.scriptText() << "\n";

  std::vector<std::string> outcomes;
  const std::size_t n = c.action ? 1 : c.trees.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    w.log.reset();
    bb->unset("f");
    w.dog.arm(c.id + " tick " + std::to_string(i + 1), std::chrono::seconds(c.budget_s));
    std::thread during;
    if (c.during) during = std::thread([&c, &w] { c.during(w); });
    const std::string bt = c.action ? c.action(w) : tickOnce(w, c.trees[i], bb, c.paths);
    if (during.joinable()) during.join();
    m25::settleTopics(w.log);
    w.dog.disarm();
    w.mg.releaseHeld();   // a goal the scenario held ends now (CANCELED if its client asked)
    std::string fault = "<unset>";
    try { fault = "\"" + bb->get<std::string>("f") + "\""; } catch (const std::exception&) {}
    os << "--- tick " << (i + 1) << ": " << (c.action ? std::string("(a direct call)") : c.trees[i]) << "\n"
       << "events:\n" << w.log.events();
    const std::string topics = w.log.topics();
    if (!topics.empty()) os << "topics (asynchronous, sorted):\n" << topics;
    os << (c.action ? "result: " : "bt: ") << bt << "\n"
       << "fault: " << fault << "\n";
    for (const auto& key : {"cur", "t_current"})
      if (c.trees.size() > i && c.trees[i].find(std::string("\"") + key + "\"") != std::string::npos)
        os << "stored: " << key << "=" << storedPose(bb, key) << "\n";
    os << "planner_mode_after: " << trainit::toString(w.rig.runtime()->plannerMode()) << "\n"
       << "counts: " << w.log.counts() << "\n";
    outcomes.push_back(bt + "|" + (fault == "<unset>" ? "" : fault.substr(1, fault.size() - 2)));
  }
  os << "\n";
  const std::string text = os.str();
  family().add(text);
#ifdef TRAINIT_M34_AFTER
  if (!c.after.empty())
  {
    EXPECT_EQ(outcomes, c.after) << c.id << "\n" << text;
  }
#else
  (void)outcomes;
#endif
}

geometry_msgs::msg::Pose pose(double x, double y, double z) { return m25::pose(x, y, z, 1, 0, 0, 0); }

const std::string kMove = "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>";
const std::string kMoveB = "<MoveWaypoint waypoint=\"b\" fault=\"{f}\"/>";

// a TCP waypoint "a" (TSA's keys for a task-program move), with extra keys
Strings tcp(const std::string& motion, Strings extra = {})
{
  Strings s{{"a.type", "tcp"}, {"a.position", "0.4;0.1;0.3"}, {"a.orientation", "1;0;0;0"}, {"a.motion", motion}};
  s.insert(s.end(), extra.begin(), extra.end());
  return s;
}
}  // namespace

// --- TOL: goal tolerances on the MoveGroupInterface routes ------------------------------------------------

TEST(M34Tolerance, a_canonical_profile_tolerance_reaches_movegroup)
{
  runCase({"TOL-01", "lin, profile tol (0.002 m / 0.02 rad): the MoveGroup pose goal carries them (2.2.0: MoveGroup"
           "Interface's 1e-4 / 1e-3, the profile's ignored)", {kMove},
           tcp("lin", {{"a.profile", "tol"}, {"a.speed", "10"}, {"a.accel", "30"}}), {}, {}, {}, true, false, 90,
           {}, {"SUCCESS|"}});
  runCase({"TOL-02", "ptp to a pose, planner pilz, profile tol: Pilz PTP's pose goal carries them", {kMove},
           tcp("ptp", {{"a.planner", "pilz"}, {"a.profile", "tol"}, {"a.speed", "50"}, {"a.accel", "50"}}), {}, {},
           {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"TOL-03", "free to a pose, planner ompl, profile pos_only (0.002 m): the position tolerance is sent, "
           "the orientation stays MoveGroupInterface's default", {kMove},
           tcp("free", {{"a.planner", "ompl"}, {"a.profile", "pos_only"}, {"a.speed", "50"}, {"a.accel", "50"}}), {},
           {}, {}, true, false, 90, {}, {"SUCCESS|"}});
}

TEST(M34Tolerance, a_tolerance_never_leaks_into_the_next_move)
{
  runCase({"TOL-04", "tick 1 lin with profile tol, tick 2 lin with profile plain (no tolerance): tick 2 sends "
           "MoveGroupInterface's defaults again", {kMove, kMoveB},
           tcp("lin", {{"a.profile", "tol"}, {"b.type", "tcp"}, {"b.position", "0.4;0.15;0.3"},
                       {"b.orientation", "1;0;0;0"}, {"b.motion", "lin"}, {"b.profile", "plain"}}),
           {}, {}, {}, true, false, 90, {}, {"SUCCESS|", "SUCCESS|"}});
}

TEST(M34Tolerance, the_legacy_route_is_unchanged)
{
  runCase({"TOL-05", "lin WITHOUT a profile key: the legacy approach profile (its explicit 0.001 is not sent: the "
           "legacy route keeps MoveGroupInterface's 1e-4 / 1e-3, as since TMR 1.x)", {kMove}, tcp("lin"), {}, {},
           {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"TOL-06", "circ (the sequence route) with profile tol: the sequence item carries 0.002 / 0.02, before "
           "and after (that route always sent the options' tolerances)", {kMove},
           tcp("circ", {{"a.aux", "0.45;0.15;0.3"}, {"a.profile", "tol"}}), {}, {}, {}, true, false, 90, {},
           {"SUCCESS|"}});
}

TEST(M34Tolerance, an_unsupported_or_invalid_tolerance_is_refused)
{
  runCase({"TOL-07", "ptp to a NAMED joint state with profile tol: a Cartesian tolerance on a joint goal cannot be "
           "honoured -- refused before MoveIt (2.2.0: moved, the tolerance dropped)", {kMove},
           {{"a.type", "joint"}, {"a.named", "ready"}, {"a.motion", "ptp"}, {"a.profile", "tol"}}, {}, {}, {}, true,
           false, 90, {}, {"FAILURE|MOTION_FAULT: UNSUPPORTED_TOLERANCE during request"}});
  runCase({"TOL-08", "lin with profile bad_tol (goal_position_tolerance -0.001): refused before MoveIt", {kMove},
           tcp("lin", {{"a.profile", "bad_tol"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: UNSUPPORTED_TOLERANCE during request"}});
}

// --- PLN: an unknown planner never falls back -----------------------------------------------------------

TEST(M34Planner, an_unknown_planner_fails_loudly)
{
  runCase({"PLN-01", "ptp to a pose with planner rrt_star (not a mode): refused, nothing sent (2.2.0: the runtime "
           "mode, ompl, planned it)", {kMove}, tcp("ptp", {{"a.planner", "rrt_star"}}), {}, {}, {}, true, false, 90,
           {}, {"FAILURE|MOTION_FAULT: UNKNOWN_PLANNER during request"}});
  runCase({"PLN-02", "lin with planner rrt_star: refused, nothing sent (2.2.0: ignored, Pilz LIN ran)", {kMove},
           tcp("lin", {{"a.planner", "rrt_star"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: UNKNOWN_PLANNER during request"}});
  runCase({"PLN-03", "MoveCollisionFree planner=\"rrt_star\": FAILURE, nothing sent (2.2.0: the runtime mode)",
           {"<MoveCollisionFree pose_key=\"g\" planner=\"rrt_star\"/>"}, {}, {{"g", pose(0.4, 0.1, 0.3)}}, {}, {},
           true, false, 90, {}, {"FAILURE|"}});
}

TEST(M34Planner, a_known_planner_is_unchanged)
{
  runCase({"PLN-04", "free to a pose with planner ompl: OMPL, as before", {kMove},
           tcp("free", {{"a.planner", "ompl"}}), {}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"PLN-05", "free with planner stomp: a registry mode whose pipeline is not loaded degrades to OMPL (the "
           "registry's documented semantics), as before", {kMove}, tcp("free", {{"a.planner", "stomp"}}), {}, {},
           {}, true, false, 90, {}, {"SUCCESS|"}});
}

// --- CIRC and TIP ---------------------------------------------------------------------------------------

TEST(M34Circ, the_via_form_is_unchanged)
{
  runCase({"CIRC-01", "MoveWaypoint circ through a via point: one CIRC item, path_constraints interim", {kMove},
           tcp("circ", {{"a.aux", "0.45;0.15;0.3"}}), {}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
}

TEST(M34Circ, the_centre_form_runs)
{
  runCase({"CIRC-02", "MoveWaypoint circ about a centre: one CIRC item, path_constraints center (2.2.0: the "
           "backend threw, the node failed with EXCEPTION, nothing sent)", {kMove},
           tcp("circ", {{"a.aux", "0.4;0.15;0.3"}, {"a.aux_is_center", "true"}}), {}, {}, {}, true, false, 90, {},
           {"SUCCESS|"}});
  runCase({"CIRC-03", "MoveCircular about a centre (2.2.0: the throw left the tick)",
           {"<MoveCircular goal_key=\"g\" aux=\"0.4;0.15;0.3\" aux_is_center=\"true\"/>"}, {},
           {{"g", pose(0.4, 0.1, 0.3)}}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
}

TEST(M34Circ, an_invalid_circle_point_is_refused)
{
  runCase({"CIRC-04", "a non-finite centre: refused before MoveIt", {kMove},
           tcp("circ", {{"a.aux", "0.4;nan;0.3"}, {"a.aux_is_center", "true"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: INVALID_POSE during request"}});
  runCase({"CIRC-05", "a non-finite via point: refused before MoveIt (2.2.0: sent)", {kMove},
           tcp("circ", {{"a.aux", "0.45;nan;0.3"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: INVALID_POSE during request"}});
  runCase({"CIRC-06", "a centre ON the goal (radius 0): refused before MoveIt", {kMove},
           tcp("circ", {{"a.aux", "0.4;0.1;0.3"}, {"a.aux_is_center", "true"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: CIRCLE_GEOMETRY during request"}});
  runCase({"CIRC-07", "a via point ON the goal: refused before MoveIt (2.2.0: sent)", {kMove},
           tcp("circ", {{"a.aux", "0.4;0.1;0.3"}}), {}, {}, {}, true, false, 90, {},
           {"FAILURE|MOTION_FAULT: CIRCLE_GEOMETRY during request"}});
}

TEST(M34Tip, circular_tip_link_reaches_the_sequence_item)
{
  runCase({"TIP-01", "MoveCircular tip_link=suction_link: the goal AND the via constraint are for suction_link "
           "(2.2.0: tcp, the target's link ignored)",
           {"<MoveCircular goal_key=\"g\" aux=\"0.45;0.15;0.3\" tip_link=\"suction_link\"/>"}, {},
           {{"g", pose(0.4, 0.1, 0.3)}}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"TIP-02", "MoveCircular with no tip_link: the configured tip (tcp), as before",
           {"<MoveCircular goal_key=\"g\" aux=\"0.45;0.15;0.3\"/>"}, {}, {{"g", pose(0.4, 0.1, 0.3)}}, {}, {}, true,
           false, 90, {}, {"SUCCESS|"}});
  runCase({"TIP-03", "MoveLinear tip_link=suction_link: MoveGroupInterface already honoured it (unchanged)",
           {"<MoveLinear pose_key=\"g\" tip_link=\"suction_link\"/>"}, {}, {{"g", pose(0.4, 0.1, 0.3)}}, {}, {}, true,
           false, 90, {}, {"SUCCESS|"}});
}

// --- RST: the Cartesian backend leaves MoveGroupInterface as it found it ---------------------------------------

TEST(M34Restore, the_next_move_sees_the_base_frame)
{
  runCase({"RST-01", "a Cartesian path in world for l3, then a LIN goal: the LIN goal is in base_link (2.2.0: in "
           "world -- the path's frame leaked into the next move)",
           {"<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>", "<MoveLinear pose_key=\"g\"/>"}, {},
           {{"g", pose(0.4, 0.1, 0.3)}},
           [](m25::Config& c) { c.process_backend = ProcessPathBackend::CARTESIAN_WAYPOINTS; }, {}, true, false, 90, {},
           {"SUCCESS|", "SUCCESS|"}});
}

TEST(M34Restore, the_backend_restores_frame_and_end_effector_link)
{
  // The backend driven directly, on a MoveGroupInterface this test owns (the runtime's is private):
  // after a successful run and after a failed one (fraction 0.5), what does it hold?
  auto action = [](m25::World& w) {
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface mgi(w.rig.node(), MoveGroupInterface::Options("arm", MoveGroupInterface::ROBOT_DESCRIPTION, "/"));
    mgi.setPoseReferenceFrame("base_link");
    mgi.setEndEffectorLink("tcp");
    trainit::CartesianWaypointBackend backend(w.rig.node(), mgi, mgi.getRobotModel(), "arm", "tcp", "base_link");
    trainit::CartesianProcessPath path;
    path.path_name = "m34_world_l3";
    path.reference_frame = "world";
    path.tcp_link = "l3";
    path.collision_checking_enabled = false;
    for (const auto& p : {pose(0.40, 0.10, 0.40), pose(0.40, 0.15, 0.40), pose(0.45, 0.15, 0.40)})
    {
      trainit::CartesianWaypoint wp;
      wp.pose = p;
      path.waypoints.push_back(wp);
    }
    std::ostringstream os;
    const auto ok = backend.run(path, trainit::MotionOptions{}, 1.0, false);
    os << "run 1 success=" << ok.success << " | after: frame=" << mgi.getPoseReferenceFrame()
       << " eef=" << mgi.getEndEffectorLink();
    mgi.setPoseReferenceFrame("base_link");
    mgi.setEndEffectorLink("tcp");
    const auto partial = backend.run(path, trainit::MotionOptions{}, 1.0, false);
    os << " | run 2 (fraction 0.5) success=" << partial.success << " | after: frame=" << mgi.getPoseReferenceFrame()
       << " eef=" << mgi.getEndEffectorLink();
    return os.str();
  };
  Case c{"RST-02", "CartesianWaypointBackend on a MoveGroupInterface set to base_link / tcp, a path in world for l3: "
         "after a success AND after a failure the interface holds base_link / tcp again (2.2.0: world / l3)"};
  c.script = [](m25::FakeMoveGroup& mg) {
    m25::Reply half;
    half.fraction = 0.5;
    mg.cart_script = {m25::Reply::succeed(), half};
  };
  c.action = action;
  c.after = {"run 1 success=1 | after: frame=base_link eef=tcp | run 2 (fraction 0.5) success=0 | after: "
             "frame=base_link eef=tcp|"};
  runCase(c);
}

// --- ST: no robot state is never the identity pose ------------------------------------------------------

TEST(M34State, a_fresh_state_reads_as_before)
{
  runCase({"ST-01", "joint states ON: StoreCurrentPose stores FK(tcp) in the model frame, as before",
           {"<StoreCurrentPose pose_key=\"cur\"/>"}, {}, {}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"ST-02", "joint states ON, FollowPath lcl_blend: the start check reads the state, one sequence goal",
           {"<FollowPath path=\"lcl_blend\" fault=\"{f}\"/>"}, {}, {}, {}, {}, true, true, 90, {}, {"SUCCESS|"}});
}

TEST(M34State, no_state_is_a_failure)
{
  runCase({"ST-03", "joint states OFF: StoreCurrentPose FAILS (2.2.0: SUCCESS, the identity pose stored)",
           {"<StoreCurrentPose pose_key=\"cur\"/>"}, {}, {}, {}, {}, false, false, 90, {}, {"FAILURE|"}});
  runCase({"ST-04", "joint states OFF: ComputeTcpTarget with the current orientation FAILS (2.2.0: the identity "
           "orientation)", {"<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" out_key=\"t_current\"/>"}, {}, {}, {}, {},
           false, false, 90, {}, {"FAILURE|"}});
  runCase({"ST-05", "joint states OFF: CheckRobotState at 0;0;0 FAILS (2.2.0: SUCCESS, the identity is at the origin)",
           {"<CheckRobotState position=\"0;0;0\"/>"}, {}, {}, {}, {}, false, false, 90, {}, {"FAILURE|"}});
  runCase({"ST-06", "joint states OFF, FollowPath lcl_blend: refused STATE_UNAVAILABLE, nothing sent (2.2.0: "
           "NOT_AT_PATH_START, a distance from the identity pose)", {"<FollowPath path=\"lcl_blend\" fault=\"{f}\"/>"},
           {}, {}, {}, {}, false, true, 90, {}, {"FAILURE|MOTION_FAULT: STATE_UNAVAILABLE during request"}});
}

TEST(M34State, the_path_start_check_waits_for_the_arm_to_settle)
{
  Case c{"ST-08", "FollowPath lcl_blend with the arm still 0.1 rad away on j1; it arrives at the start 0.5 s into the "
         "tick: the start check waits (bounded) and the path runs (2.2.0: NOT_AT_PATH_START at the first read)",
         {"<FollowPath path=\"lcl_blend\" fault=\"{f}\"/>"}};
  c.paths = true;
  c.q = {0.1, 0.0, 0.0};
  c.during = [](m25::World& w) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    w.js.set({0.0, 0.0, 0.0});
  };
  c.after = {"SUCCESS|"};
  runCase(c);
}

TEST(M34State, no_state_never_builds_a_cartesian_path)
{
  runCase({"ST-07", "joint states OFF, a Cartesian process path: refused before anything is sent (2.2.0: "
           "computeCartesianPath, then the trajectory timed from the model's DEFAULT state, and executed)",
           {"<ExecuteProcessPath path_file=\"${PATHS}/p_world_l3.yaml\"/>"}, {}, {},
           [](m25::Config& c) { c.process_backend = ProcessPathBackend::CARTESIAN_WAYPOINTS; }, {}, false, false, 90,
           {}, {"FAILURE|"}});
}

TEST(M34Circ, a_non_finite_goal_or_a_circ_without_its_point_is_refused)
{
  runCase({"CIRC-08", "MoveCircular to a goal with a non-finite coordinate: refused before MoveIt (2.2.0: sent)",
           {"<MoveCircular goal_key=\"g\" aux=\"0.45;0.15;0.3\"/>"}, {},
           {{"g", m25::pose(0.4, std::nan(""), 0.3, 1, 0, 0, 0)}}, {}, {}, true, false, 90, {}, {"FAILURE|"}});
  Case c{"CIRC-09", "executeProcessSequence with a CIRC item that has neither a via point nor a centre: refused "
         "before MoveIt (2.2.0: one goal with no aux constraint)"};
  c.action = [](m25::World& w) {
    trainit::ProcessSegment seg;
    seg.name = "circ";
    seg.type = trainit::ProcessSegmentType::CIRCULAR;
    seg.target_pose = pose(0.4, 0.1, 0.3);
    const trainit::MotionResult r = w.rig.runtime()->executeProcessSequence({seg}, trainit::MotionOptions{});
    return "success=" + std::to_string(r.success) + " cause=" + r.cause;
  };
  c.after = {"success=0 cause=CIRCLE_GEOMETRY|"};
  runCase(c);
}

// --- TO: a timed-out Pilz goal is cancelled ---------------------------------------------------------------

TEST(M34Timeout, a_timed_out_sequence_goal_is_cancelled)
{
  runCase({"TO-01", "the sequence result never comes: after the result timeout the goal is CANCELLED and the move "
           "fails RESULT_TIMEOUT (2.2.0: 120 s, no cancel -- the goal kept running)", {kMove},
           tcp("circ", {{"a.aux", "0.45;0.15;0.3"}}), {}, [](m25::Config& c) { shortSequenceTimeout(c); },
           [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(m25::Reply::hold()); }, true, false, 200, {},
           {"FAILURE|MOTION_FAULT: RESULT_TIMEOUT during plan_and_execute"}});
}

// --- STOP: the door's stop --------------------------------------------------------------------------------

TEST(M34Stop, stop_sends_the_stop_event)
{
  Case c{"STOP-01", "MotionRuntime::stop(): move_group's trajectory_execution_event 'stop' (unchanged; what the "
         "runner calls on SIGINT since M3.4)"};
  c.action = [](m25::World& w) {
    const trainit::MotionResult r = w.rig.runtime()->stop();
    return "success=" + std::to_string(r.success) + " stage=" + r.stage;
  };
  c.after = {"success=1 stage=stop|"};
  runCase(c);
}

// --- ALS: the motion aliases ------------------------------------------------------------------------------

TEST(M34Alias, the_aliases_take_the_approach_profile)
{
  runCase({"ALS-01", "motion=linear with no profile key: the approach profile (0.2), as lin (2.2.0: transit, 0.5)",
           {kMove}, tcp("linear"), {}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
  runCase({"ALS-02", "motion=circular with no profile key: the approach profile (0.2), as circ (2.2.0: transit)",
           {kMove}, tcp("circular", {{"a.aux", "0.45;0.15;0.3"}}), {}, {}, {}, true, false, 90, {}, {"SUCCESS|"}});
}

TEST(M34Family, zz_the_family_is_the_committed_record)
{
  family().check();
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
