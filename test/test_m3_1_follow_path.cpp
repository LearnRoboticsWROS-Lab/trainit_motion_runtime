// test_m3_1_follow_path (vNext M3.1, ADR-0024): FollowPath and MoveWaypoint's additive keys ticked
// for REAL (BT nodes, MoveItMotionRuntime, the Pilz sequence and Cartesian backends) against the
// M2.5 in-process fake move_group, on a private domain.
//
// What it pins:
//   - a path is ONE motion: one Pilz sequence goal (one item per segment, the blend radii of a
//     'continuity: blend' path, 0 on a stopped one, the last always 0), or one Cartesian path
//     request for a continuous dense path -- never a goal per sample, never a BT node per sample;
//   - the speeds, tolerances, interim/centre points and frames that goal carries;
//   - every refusal sends NOTHING to MoveIt and says why (and `fault` names the cause);
//   - MoveWaypoint's new keys (profile, frame, tcp) change only a waypoint that carries them.
// The whole family is compared with the committed AFTER record test/data/m3_1/captures/ (M3.1 is
// new behaviour: there is no BEFORE; the legacy families stay byte-identical in their own tests).
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "m2_5/motion_harness.hpp"
#include "trainit_motion_runtime/path_library.hpp"

namespace
{
using trainit::PathLibrary;
using trainit::PathRuntimeFacts;
using trainit::ProcessPathBackend;

const std::string kData = TRAINIT_M31_DATA_DIR;

// TF for the rig: world -> base_link (the test arm's fixed joint) and base_link -> tcp_frame, a
// frame at the arm's start tcp pose (0.5, 0, 0.3), so a pose "in tcp_frame" has a known base pose.
struct Tf
{
  explicit Tf(const rclcpp::Node::SharedPtr& node)
  {
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, true);
    pub_node = std::make_shared<rclcpp::Node>("m31_static_tf", "/");
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(pub_node);
    broadcaster->sendTransform({frame("world", "base_link", 0.0, 0.0, 0.1),
                                frame("base_link", "tcp_frame", 0.5, 0.0, 0.3)});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline &&
           !(buffer->canTransform("base_link", "tcp_frame", tf2::TimePointZero) &&
             buffer->canTransform("base_link", "world", tf2::TimePointZero)))
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  static geometry_msgs::msg::TransformStamped frame(const char* parent, const char* child, double x, double y,
                                                    double z)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x; t.transform.translation.y = y; t.transform.translation.z = z;
    t.transform.rotation.w = 1.0;
    return t;
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

struct Case
{
  std::string id;
  std::string pins;
  std::string tree;
  std::vector<std::pair<std::string, std::string>> strings{};
  std::shared_ptr<const PathLibrary> paths{};
  std::function<void(m25::FakeMoveGroup&)> script{};
  ProcessPathBackend backend{ProcessPathBackend::PILZ_SEQUENCE};
  std::string bt;               // the expected BT status
  std::string fault;            // the expected `fault` ("" = not written)
  bool no_events{false};        // nothing may reach MoveIt
};

std::shared_ptr<const PathLibrary> library(ProcessPathBackend b = ProcessPathBackend::PILZ_SEQUENCE)
{
  return std::make_shared<const PathLibrary>(trainit::loadPathLibrary(kData + "/paths.yaml",
                                                                      PathRuntimeFacts{b, "tcp"}));
}

std::string tickOnce(m25::World& w, const Case& c, const BT::Blackboard::Ptr& bb)
{
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  trainit::BtContext ctx = m25::makeContext(w.rig.runtime(), w.rig.node());
  ctx.tf = tf().buffer;
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES,
                                          trainit::loadMotionProfiles(TRAINIT_PROFILES_FILE));
  if (c.paths) bb->set<std::shared_ptr<const PathLibrary>>(trainit::BB_PATH_LIBRARY, c.paths);
  try
  {
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         c.tree + "</BehaviorTree></root>");
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
    return std::string("THROW ") + e.what();
  }
}

m25::Family& family()
{
  static m25::Family f("m3_1_follow_path", kData + "/captures");   // test harness: one per process
  return f;
}

std::string lineValue(const std::string& text, const std::string& prefix)
{
  const auto a = text.find("\n" + prefix);
  if (a == std::string::npos) return "<none>";
  const auto b = text.find('\n', a + 1);
  return text.substr(a + 1 + prefix.size(), b - a - 1 - prefix.size());
}

void runCase(const Case& c)
{
  m25::World& w = m25::world();
  tf();
  m25::Config cfg = m25::defaultConfig();
  cfg.process_backend = c.backend;
  w.js.set({0.0, 0.0, 0.0});
  w.js.on(true);
  w.mg.resetScenario();
  if (c.script) c.script(w.mg);
  w.dog.arm("build " + c.id, std::chrono::seconds(60));
  w.rig.build(cfg);
  w.dog.disarm();

  auto bb = BT::Blackboard::create();
  for (const auto& [k, v] : c.strings) bb->set<std::string>(k, v);
  std::string strings;
  for (const auto& [k, v] : c.strings) strings += (strings.empty() ? "" : "; ") + k + "=" + v;

  w.log.reset();
  w.dog.arm(c.id, std::chrono::seconds(120));
  const std::string bt = tickOnce(w, c, bb);
  m25::settleTopics(w.log);
  w.dog.disarm();
  std::string fault = "<unset>";
  try { fault = "\"" + bb->get<std::string>("f") + "\""; } catch (const std::exception&) {}

  std::ostringstream os;
  os << "=== scenario " << c.id << "\n"
     << "pins: " << c.pins << "\n"
     << "config: " << m25::describe(cfg) << "\n"
     << "state: " << w.js.describe() << "\n"
     << "blackboard: " << (strings.empty() ? "(none)" : strings) << "\n"
     << "paths: " << (c.paths ? std::to_string(c.paths->size()) + " path(s)" : "(no library)") << "\n"
     << "script: " << w.mg.scriptText() << "\n"
     << "--- tick: " << c.tree << "\n"
     << "events:\n" << w.log.events()
     << "bt: " << bt << "\n"
     << "fault: " << fault << "\n"
     << "counts: " << w.log.counts() << "\n\n";
  const std::string text = os.str();
  family().add(text);

  EXPECT_EQ(bt, c.bt) << c.id << "\n" << text;
  EXPECT_EQ(fault, c.fault.empty() ? "<unset>" : "\"" + c.fault + "\"") << c.id << "\n" << text;
  if (c.no_events) EXPECT_EQ(lineValue(text, "counts: "), "none") << c.id << ": nothing may reach MoveIt\n" << text;
}

const std::string kFollow = "<FollowPath path=\"%\" fault=\"{f}\"/>";
std::string follow(const std::string& name)
{
  std::string t = kFollow;
  return t.replace(t.find('%'), 1, name);
}

std::vector<std::pair<std::string, std::string>> waypoints()
{
  return {{"w1.type", "tcp"},          {"w1.position", "0.5;0.0;0.3"},  {"w1.orientation", "0;0;0;1"},
          {"w2.type", "tcp"},          {"w2.position", "0.0;0.1;0.0"},  {"w2.orientation", "0;0;0;1"},
          {"w2.frame", "tcp_frame"},   // = (0.5, 0.1, 0.3) in base_link
          {"w3.position", "0.5;0.2;0.2"}, {"w3.orientation", "0;0;0;1"},
          {"w4.position", "0.4;0.2;0.2"}, {"w4.orientation", "0;0;0;1"},
          {"w2b.position", "0.5;0.17;0.27"}};
}
}  // namespace

// --- a path is ONE motion ----------------------------------------------------------------------------

TEST(M31FollowPath, a_blended_lin_circ_lin_path_is_one_sequence_goal)
{
  runCase({"FP-01", "W1 -LIN-> W2 -CIRC(via)-> W3 -LIN-> W4, continuity blend: ONE sequence goal, 3 items, "
           "blend 0.02/0.02/0, the CIRC's interim point, the per-segment speeds, the path's goal tolerance",
           follow("lcl_blend"), {}, library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
}

TEST(M31FollowPath, a_stopped_path_has_no_blend)
{
  runCase({"FP-02", "the same path, continuity stop and the process profile: blend 0 everywhere, 0.05 scaling",
           follow("lcl_stop"), {}, library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
}

TEST(M31FollowPath, waypoint_states_are_resolved_from_the_blackboard_and_tf)
{
  runCase({"FP-03", "states by WAYPOINT name; w2 is given in tcp_frame and reaches the goal in base_link",
           follow("by_waypoints"), waypoints(), library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
}

TEST(M31FollowPath, the_centre_form_of_circ)
{
  runCase({"FP-04", "a CIRC by its centre: path_constraints 'center', no throw (the backend's CIRC-centre "
           "debt is avoided, not fixed)", follow("circ_center"), {}, library(), nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
}

TEST(M31FollowPath, a_path_in_another_frame)
{
  runCase({"FP-05", "a path whose poses are in tcp_frame: its goals are in base_link",
           follow("in_tool_frame"), {}, library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
}

TEST(M31FollowPath, a_dense_stopped_path_is_one_sequence_of_lin_items)
{
  runCase({"FP-06", "a dense CSV source, continuity stop: ONE sequence goal, a LIN item per sample after the "
           "start, blend 0", follow("dense_stop"), {}, library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE,
           "SUCCESS", ""});
}

TEST(M31FollowPath, a_continuous_dense_path_is_one_cartesian_path)
{
  auto lib = std::make_shared<const PathLibrary>(trainit::parsePathLibrary(
    "schema_version: 1\npaths:\n  bead:\n    continuity: blend\n"
    "    source: {file: bead.csv, orientation: [0, 0, 0, 1]}\n",
    kData, PathRuntimeFacts{ProcessPathBackend::CARTESIAN_WAYPOINTS, "tcp"}));
  runCase({"FP-07", "a dense source, continuity blend, on a cartesian_waypoints runtime: ONE "
           "compute_cartesian_path request through the samples after the start, then one execution",
           follow("bead"), {}, lib, nullptr, ProcessPathBackend::CARTESIAN_WAYPOINTS, "SUCCESS", ""});
}

// --- what fails, and how ---------------------------------------------------------------------------

TEST(M31FollowPath, a_planning_failure_is_the_targets)
{
  runCase({"FP-10", "the sequence answers PLANNING_FAILED: FAILURE, and no fault (the target's failure)",
           follow("lcl_blend"), {}, library(),
           [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(m25::Reply::abort(-1)); },
           ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", ""});
}

TEST(M31FollowPath, a_rejected_goal_is_a_fault)
{
  runCase({"FP-11", "the sequence goal is rejected: FAILURE and a fault (not the target's)",
           follow("lcl_blend"), {}, library(),
           [](m25::FakeMoveGroup& mg) { mg.seq_script.push_back(m25::Reply::reject()); },
           ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", "MOTION_FAULT: GOAL_REJECTED during plan_and_execute"});
}

TEST(M31FollowPath, refusals_send_nothing)
{
  const auto lib = library();
  runCase({"FP-20", "no path library", follow("lcl_blend"), {}, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE,
           "FAILURE", "MOTION_FAULT: NO_PATH_LIBRARY during request", true});
  runCase({"FP-21", "an unknown path", follow("nowhere"), {}, lib, nullptr, ProcessPathBackend::PILZ_SEQUENCE,
           "FAILURE", "MOTION_FAULT: UNKNOWN_PATH during request", true});
  runCase({"FP-22", "the robot is not at the path's start", follow("elsewhere"), {}, lib, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", "MOTION_FAULT: NOT_AT_PATH_START during request", true});
  auto joint = waypoints();
  joint[0] = {"w1.type", "joint"};
  runCase({"FP-23", "a waypoint state that is a joint waypoint", follow("by_waypoints"), joint, lib, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", "MOTION_FAULT: PATH_UNRESOLVED during request", true});
  auto missing = waypoints();
  missing.erase(missing.begin() + 7);   // w3.position
  runCase({"FP-24", "a waypoint state with no position", follow("by_waypoints"), missing, lib, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", "MOTION_FAULT: PATH_UNRESOLVED during request", true});
  auto close = waypoints();
  close[7] = {"w3.position", "0.5;0.11;0.3"};   // the 0.01 m blend of segment 0 is more than half of 0.01 m
  runCase({"FP-25", "resolved waypoints whose geometry fails (a blend larger than half a segment)",
           follow("by_waypoints"), close, lib, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: PATH_GEOMETRY during request", true});
  auto frameless = waypoints();
  frameless[6] = {"w2.frame", "no_such_frame"};
  runCase({"FP-26", "a waypoint state in a frame with no transform", follow("by_waypoints"), frameless, lib,
           nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE", "MOTION_FAULT: FRAME_UNRESOLVED during request", true});
  runCase({"FP-27", "the port names a profile that is not loaded", "<FollowPath path=\"by_waypoints\" "
           "profile=\"nope\" fault=\"{f}\"/>", waypoints(), lib, nullptr, ProcessPathBackend::PILZ_SEQUENCE,
           "FAILURE", "MOTION_FAULT: UNKNOWN_PROFILE during request", true});
}

// --- MoveWaypoint's additive keys -------------------------------------------------------------------

TEST(M31MoveWaypoint, the_new_keys_change_only_a_waypoint_that_carries_them)
{
  const std::vector<std::pair<std::string, std::string>> lin{
    {"a.type", "tcp"}, {"a.position", "0.5;0.1;0.3"}, {"a.orientation", "0;0;0;1"}, {"a.motion", "lin"}};
  runCase({"MW-01", "a LIN waypoint with none of the keys: the approach profile (0.2), the pose as given",
           "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>", lin, nullptr, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
  auto profiled = lin; profiled.emplace_back("a.profile", "process");
  runCase({"MW-02", "profile: process -- the named profile's 0.05 instead of approach",
           "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>", profiled, nullptr, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
  const std::vector<std::pair<std::string, std::string>> framed{
    {"a.type", "tcp"}, {"a.position", "0.0;0.1;0.0"}, {"a.orientation", "0;0;0;1"}, {"a.motion", "lin"},
    {"a.frame", "tcp_frame"}};
  runCase({"MW-03", "frame: tcp_frame -- the same goal as MW-01, given in another frame",
           "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>", framed, nullptr, nullptr,
           ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
  auto tcp = lin; tcp.emplace_back("a.tcp", "suction_link");
  runCase({"MW-04", "tcp other than the tip: refused, nothing sent", "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>",
           tcp, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: UNSUPPORTED_TCP during request", true});
  auto tip = lin; tip.emplace_back("a.tcp", "tcp");
  runCase({"MW-05", "tcp equal to the tip: moves as MW-01", "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>",
           tip, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "SUCCESS", ""});
  auto unknown = lin; unknown.emplace_back("a.profile", "nope");
  runCase({"MW-06", "an unknown profile: refused, nothing sent", "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>",
           unknown, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: UNKNOWN_PROFILE during request", true});
  auto noframe = framed; noframe.back() = {"a.frame", "no_such_frame"};
  runCase({"MW-07", "a frame with no transform: refused, nothing sent", "<MoveWaypoint waypoint=\"a\" fault=\"{f}\"/>",
           noframe, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: FRAME_UNRESOLVED during request", true});
  const std::vector<std::pair<std::string, std::string>> joint{
    {"j.type", "joint"}, {"j.named", "ready"}, {"j.motion", "ptp"}, {"j.frame", "tcp_frame"}};
  runCase({"MW-08", "a frame on a joint waypoint: refused, nothing sent", "<MoveWaypoint waypoint=\"j\" fault=\"{f}\"/>",
           joint, nullptr, nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: UNSUPPORTED_FRAME during request", true});
}

TEST(M31FollowPath, a_non_finite_state_is_refused)
{
  auto nan = waypoints();
  nan[7] = {"w3.position", "0.5;nan;0.2"};
  runCase({"FP-28", "a waypoint state with a non-finite coordinate: refused, nothing sent", follow("by_waypoints"),
           nan, library(), nullptr, ProcessPathBackend::PILZ_SEQUENCE, "FAILURE",
           "MOTION_FAULT: INVALID_POSE during request", true});
}

TEST(M31Family, zz_the_family_is_the_committed_record)
{
  family().check();
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
