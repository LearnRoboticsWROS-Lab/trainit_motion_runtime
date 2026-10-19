// vNext M2.5 (tests first): what the state-reading nodes and ServoTrack read and send today.
//
// These are the three class-A accesses of the M2.5 audit, the reads that reach around
// MotionRuntime through moveGroup().getCurrentPose(): the currentPose() helper (StoreCurrentPose,
// ComputeTcpTarget, CheckRobotState), ServoTrack's frame read and its 50 Hz feedback. Frozen on
// TMR 1.8.0 before the seam moved them (state_reads.txt, servo_track.txt); every later commit must
// reproduce the captures byte for byte.
//
// The arm's joint states are fixed, so a read is FK of a known configuration; with the publisher
// stopped a read waits ~1 s and returns the identity pose with SUCCESS (KNOWN_DEBT: the silent
// identity read -- pinned, not changed). ServoTrack talks to a fake moveit_servo node (ADR-0015
// Decision 5); each scenario uses its own node name because ServoTrack's clients, publishers and
// "started once" registry are process-static.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp_action/rclcpp_action.hpp>

#include "m2_5/motion_harness.hpp"

using m25::Scenario;

// The canary of test_m2_5_interface_purity: that test proves motion_runtime.hpp names no MoveIt
// C++ API by requiring MOVEIT_CLASS_FORWARD to be UNDEFINED there. It must be defined wherever
// MoveIt's C++ headers ARE included (here, through moveit_motion_runtime.hpp), or that check
// would pass for the wrong reason.
#ifndef MOVEIT_CLASS_FORWARD
#error "MOVEIT_CLASS_FORWARD is not defined by MoveIt's headers: the interface-purity check is blind"
#endif

namespace
{
using Strings = std::vector<std::pair<std::string, std::string>>;

const std::vector<double> kHome{0.0, 0.0, 0.0};
const std::vector<double> kReady{0.3, -0.4, 0.6};

std::string poseLine(BT::Blackboard& bb, const std::string& key)
{
  try { return key + "=Pose" + m25::describe(bb.get<geometry_msgs::msg::Pose>(key)) + "\n"; }
  catch (const std::exception&) { return key + "=(none)\n"; }
}

std::function<std::string(BT::Blackboard&)> report(std::vector<std::string> keys)
{
  return [keys](BT::Blackboard& bb) {
    std::string s;
    for (const auto& k : keys) s += "stored: " + poseLine(bb, k);
    return s;
  };
}

void family(const std::string& name, const std::vector<Scenario>& scenarios)
{
  m25::World& w = m25::world();
  m25::Family f(name);
  for (const auto& s : scenarios) f.add(m25::run(w, s));
  f.check();
}
}  // namespace

// --- READ: StoreCurrentPose, ComputeTcpTarget, CheckRobotState --------------------------------------
TEST(M25StateReads, state_reads)
{
  family("state_reads", {
    {"READ-01", "StoreCurrentPose stores FK(tcp) in the model frame (world); the frame is dropped",
     nullptr, {}, {}, {"<StoreCurrentPose pose_key=\"cur\"/>"}, nullptr, nullptr, nullptr, kReady,
     true, 90, report({"cur"})},
    {"READ-02", "ComputeTcpTarget: the current orientation is read; the quat and fixed_rpy branches read nothing",
     nullptr, {}, {},
     {"<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" out_key=\"t_current\"/>",
      "<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" quat=\"1;0;0;0\" out_key=\"t_quat\"/>",
      "<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" use_current_orientation=\"false\" fixed_rpy_deg=\"180;0;0\" out_key=\"t_rpy\"/>",
      "<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" tcp_offset_xyz=\"0;0;0.05\" out_key=\"t_offset\"/>"},
     nullptr, nullptr, nullptr, kReady, true, 90, report({"t_current", "t_quat", "t_rpy", "t_offset"})},
    {"READ-03", "CheckRobotState: inside and outside the tolerance (arm at home: tcp (0.5,0,0.4) in world)",
     nullptr, {}, {},
     {"<CheckRobotState position=\"0.5;0;0.4\"/>",
      "<CheckRobotState position=\"0.52;0;0.4\" position_tolerance=\"0.03\"/>",
      "<CheckRobotState position=\"0.6;0;0.4\" position_tolerance=\"0.03\"/>",
      "<CheckRobotState position=\"0.5;0;0.3\"/>"},
     nullptr, nullptr, nullptr, kHome},
    {"READ-04", "no joint states: identity pose and SUCCESS after ~1 s; CheckRobotState fails far, passes at 0;0;0",
     nullptr, {}, {},
     {"<StoreCurrentPose pose_key=\"cur\"/>",
      "<ComputeTcpTarget target_xyz=\"0.4;0.1;0.3\" out_key=\"t_current\"/>",
      "<CheckRobotState position=\"0.5;0;0.4\"/>",
      "<CheckRobotState position=\"0;0;0\"/>"},
     nullptr, nullptr, nullptr, kHome, false, 90, report({"cur", "t_current"})},
  });
}

// --- SERVO: ServoTrack against a fake moveit_servo node ----------------------------------------------
namespace
{
std::unique_ptr<m25::FakeServo> g_servo;
std::chrono::steady_clock::time_point g_t0;

Strings target(const std::string& wp, const std::string& position, const Strings& extra = {})
{
  Strings s{{wp + ".type", "tcp"}, {wp + ".position", position}};
  for (const auto& [k, v] : extra) s.emplace_back(wp + "." + k, v);
  return s;
}

Scenario servo(Scenario s, const std::string& node, bool triggers = true, bool twist = true,
               std::optional<int8_t> inject = std::nullopt)
{
  m25::World& w = m25::world();
  s.before_tick = [&w, node, triggers, twist, inject] {
    if (!node.empty()) g_servo = std::make_unique<m25::FakeServo>(w.log, "/", node, triggers, twist);
    if (inject && g_servo) g_servo->injectAfterFirstTwist(*inject);
    g_t0 = std::chrono::steady_clock::now();
  };
  auto inner = s.report;
  s.report = [inner](BT::Blackboard& bb) {
    std::string out = g_servo ? g_servo->twists() : std::string("twists: no servo node\n");
    if (inner) out += inner(bb);
    g_t0 = std::chrono::steady_clock::now();
    return out;
  };
  s.after = [] { g_servo.reset(); };
  return s;
}
}  // namespace

TEST(M25StateReads, servo_track)
{
  m25::World& w = m25::world();
  // SERVO-08: an execution in flight, held by the fake execute_trajectory server
  using Execute = moveit_msgs::action::ExecuteTrajectory;
  auto client = rclcpp_action::create_client<Execute>(w.rig.node(), "/execute_trajectory");
  std::shared_future<rclcpp_action::ClientGoalHandle<Execute>::SharedPtr> held;

  Scenario s08 = servo({"SERVO-08", "arbitration ON: an execution in flight refuses the stream before any call; "
                        "the port unset (tick 2) streams",
                        nullptr, target("t", "0.5;0;0.4"), {},
                        {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s8\" "
                         "execution_status_topic=\"/execute_trajectory/_action/status\"/>",
                         "<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s8\"/>"}},
                       "m25_servo_s8");
  auto s08_before = s08.before_tick;
  s08.before_tick = [&, s08_before] {
    s08_before();
    w.mg.ex_script.push_back(m25::Reply::hold());
    client->wait_for_action_server(std::chrono::seconds(5));
    Execute::Goal goal;
    goal.trajectory.joint_trajectory.joint_names = m25::kJoints;
    held = client->async_send_goal(goal);
    held.wait_for(std::chrono::seconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));   // the status array is latched
  };
  auto s08_report = s08.report;
  int s08_tick = 0;
  s08.report = [&, s08_report](BT::Blackboard& bb) {
    if (++s08_tick == 1) w.mg.releaseHeld();
    return s08_report(bb);
  };

  // The margin is wide on purpose: the halt is seen in tens of ms, but the release's first
  // pause_servo call can lose its RESPONSE to the fresh client's discovery race (the request
  // arrives; the client waits its 2 s budget) -- pre-existing, recorded as debt.
  Scenario s04 = servo({"SERVO-04", "a servo HALT status (4, collision) after the first twist fails the move, then releases",
                        nullptr, target("t", "0.6;0;0.4"), {},
                        {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s4\" timeout_ms=\"20000\"/>"}},
                       "m25_servo_s4", true, true, int8_t{4});
  auto s04_report = s04.report;
  s04.report = [s04_report](BT::Blackboard& bb) {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - g_t0).count();
    return s04_report(bb) + "halted before half the timeout: " + (ms < 10000 ? "yes" : "no") + "\n";
  };

  m25::Family f("servo_track");
  for (const Scenario& s : std::vector<Scenario>{
    servo({"SERVO-01", "target = the current TCP: start once, unpause, a zero twist in the model frame, pause",
           nullptr, target("t", "0.5;0;0.4"), {}, {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s1\"/>"}},
          "m25_servo_s1"),
    servo({"SERVO-02", "a second ServoTrack on the same servo node: no second start_servo",
           nullptr, target("t", "0.5;0;0.4"), {}, {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s1\"/>"}},
          "m25_servo_s1"),
    servo({"SERVO-03", "5 cm short in x: the first twist, then FAILURE on the timeout, a zero twist and pause",
           nullptr, target("t", "0.55;0;0.4"), {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s3\" timeout_ms=\"500\"/>"}},
          "m25_servo_s3"),
    s04,
    servo({"SERVO-05", "no servo node: FAILURE after the 2 s discovery, no call",
           nullptr, target("t", "0.55;0;0.4"), {}, {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_absent\"/>"}},
          ""),
    servo({"SERVO-06", "services but nothing subscribed to the twist: FAILURE after 2 s, then released",
           nullptr, target("t", "0.55;0;0.4"), {}, {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s6\"/>"}},
          "m25_servo_s6", true, false),
    servo({"SERVO-07", "a target in base_link while the arm is measured in world: FAILURE before any call",
           nullptr, target("t", "0.5;0;0.3", {{"frame", "base_link"}}), {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s7\"/>"}},
          "m25_servo_s7"),
    s08,
    servo({"SERVO-09", "a joint waypoint, or no position: FAILURE at once",
           nullptr, {{"j.type", "joint"}, {"j.named", "ready"}, {"n.type", "tcp"}}, {},
           {"<ServoTrack waypoint=\"j\" servo_node=\"m25_servo_s9\"/>",
            "<ServoTrack waypoint=\"n\" servo_node=\"m25_servo_s9\"/>"}},
          "m25_servo_s9"),
    servo({"SERVO-10", "start_servo=false: no start call",
           nullptr, target("t", "0.5;0;0.4"), {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s10\" start_servo=\"false\"/>"}},
          "m25_servo_s10"),
    servo({"SERVO-11", "command_in_type unitless: the twist is divided by linear_scale 0.4",
           nullptr, target("t", "0.55;0;0.4"), {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s11\" command_in_type=\"unitless\" timeout_ms=\"500\"/>"}},
          "m25_servo_s11"),
    servo({"SERVO-12", "an orientation target: the angular twist (90 deg about z, capped at 0.30 rad/s)",
           nullptr, target("t", "0.5;0;0.4", {{"orientation", "0;0;0.7071068;0.7071068"}}), {},
           {"<ServoTrack waypoint=\"t\" servo_node=\"m25_servo_s12\" timeout_ms=\"500\"/>"}},
          "m25_servo_s12"),
  })
  {
    f.add(m25::run(w, s));
  }
  f.check();
}

// The test arm's closed-form forward kinematics of the tcp, in the model frame (world): the
// shoulder at z 0.4, links 0.3 and 0.2, j1 about z, j2/j3 about y.
std::pair<Eigen::Vector3d, Eigen::Matrix3d> analyticTcp(const std::vector<double>& q)
{
  const Eigen::Matrix3d r = (Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(q[1] + q[2], Eigen::Vector3d::UnitY())).toRotationMatrix();
  const Eigen::Vector3d p = Eigen::Vector3d(0, 0, 0.4) +
    Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ()) *
      (Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()) *
       (Eigen::Vector3d(0.3, 0, 0) + Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitY()) * Eigen::Vector3d(0.2, 0, 0)));
  return {p, r};
}

// The harness checks itself: a state read equals the arm's closed-form forward kinematics.
TEST(M25StateReads, a_read_is_the_arms_analytic_fk)
{
  m25::World& w = m25::world();
  for (const auto& q : {kHome, kReady, std::vector<double>{-0.5, 0.25, -0.75}})
  {
    w.js.set(q);
    w.js.on(true);
    w.rig.build(m25::defaultConfig());
    auto bb = BT::Blackboard::create();
    ASSERT_EQ(w.rig.tick("<StoreCurrentPose pose_key=\"cur\"/>", bb), "SUCCESS");
    const auto got = bb->get<geometry_msgs::msg::Pose>("cur");
    const auto [p, r] = analyticTcp(q);
    EXPECT_NEAR(got.position.x, p.x(), 1e-9);
    EXPECT_NEAR(got.position.y, p.y(), 1e-9);
    EXPECT_NEAR(got.position.z, p.z(), 1e-9);
    const Eigen::Quaterniond gq(got.orientation.w, got.orientation.x, got.orientation.y, got.orientation.z);
    EXPECT_LT((gq.toRotationMatrix() - r).norm(), 1e-9);
  }
}

// READ-D (vNext M2.5, 4/7; oracle replaced in M2.6 when moveGroup() was removed): the door's
// read is the tip link's FK in the robot model frame with a fresh state. With none it returned the
// identity pose (the silent identity read, KNOWN_DEBT); since vNext M3.4 (R-M3-20, owner D3) it
// throws RobotStateUnavailable, naming the reason -- never a pose.
TEST(M25StateReads, current_tip_pose_is_the_tip_fk_in_the_model_frame)
{
  m25::World& w = m25::world();
  w.js.set(kReady);
  w.js.on(true);
  w.rig.build(m25::defaultConfig());
  const trainit::MotionRuntime& door = *w.rig.runtime();
  const geometry_msgs::msg::PoseStamped read = door.currentTipPose();
  EXPECT_EQ(read.header.frame_id, "world");
  const auto [p, r] = analyticTcp(kReady);
  EXPECT_NEAR(read.pose.position.x, p.x(), 1e-9);
  EXPECT_NEAR(read.pose.position.y, p.y(), 1e-9);
  EXPECT_NEAR(read.pose.position.z, p.z(), 1e-9);
  const Eigen::Quaterniond got(read.pose.orientation.w, read.pose.orientation.x,
                               read.pose.orientation.y, read.pose.orientation.z);
  EXPECT_NEAR(std::abs(got.dot(Eigen::Quaterniond(r))), 1.0, 1e-9);

  w.js.on(false);
  w.rig.build(m25::defaultConfig());
  const trainit::MotionRuntime& stale = *w.rig.runtime();
  try
  {
    const auto none = stale.currentTipPose();
    ADD_FAILURE() << "no robot state, yet a pose was returned: " << m25::describe(none.pose);
  }
  catch (const trainit::RobotStateUnavailable& e)
  {
    EXPECT_NE(std::string(e.what()).find("no fresh robot state"), std::string::npos) << e.what();
  }
  w.js.on(true);
}

int main(int argc, char** argv)
{
  return m25::runAll(argc, argv, TRAINIT_M25_DOMAIN);
}
