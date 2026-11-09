// test_m4_candidate_consumer (vNext M4, ADR-0026 amendment M4): the grasp-candidate CONSUMER hardened for the
// SurfaceGrasp provider -- DetectCandidates and SetWaypointFromCandidate ticked for real (private domain 76).
//
//   TS  TF at the OBSERVATION stamp: a camera that moves (on the arm) maps a candidate through the pose it had
//       at the candidate's stamp, never the latest; an unstamped candidate takes the latest (said in its log)
//   CP  the CAPTURE (trigger="true"): a producer sampled on demand; a result stamped before the capture began
//       is rejected; no service -> TRIGGER_UNAVAILABLE; trigger without a detector refused
//   OR  orientation="candidate": the producer's own orientation, used only when its +Z IS the approach
//   EN  max_tilt_deg: the FAIL-CLOSED envelope from the taught reference, inclusive boundary, both modes
//   NK  the per-topic feed is keyed on the node: a feed made by a node that is gone is never handed out
//   PF  the pure helpers (approachTiltDeg, candidateOrientationProblem)
//   LV  OPT-IN, live (M4-B / M4-C harnesses): M4_LIVE_PROVIDER=<name> consumes a REAL provider on the graph --
//       a capture, TF at the observation stamp from /tf(_static), orientation candidate within M4_LIVE_MAX_TILT
//       of a tool-down reference; skipped when the variable is unset
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trainit_interfaces/msg/grasp_candidate_array.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/grasp_candidates.hpp"

namespace
{
using Candidates = trainit_interfaces::msg::GraspCandidateArray;
using Candidate = trainit_interfaces::msg::GraspCandidate;

std::string bbStr(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  try { return bb->get<std::string>(key); } catch (const std::exception&) { return "<unset>"; }
}

Eigen::Vector3d vecOf(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  double x = NAN, y = NAN, z = NAN;
  if (std::sscanf(bbStr(bb, key).c_str(), "%lf;%lf;%lf", &x, &y, &z) != 3) return {NAN, NAN, NAN};
  return {x, y, z};
}

Eigen::Quaterniond quatOf(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  double x = NAN, y = NAN, z = NAN, w = NAN;
  if (std::sscanf(bbStr(bb, key).c_str(), "%lf;%lf;%lf;%lf", &x, &y, &z, &w) != 4)
    return Eigen::Quaterniond(NAN, NAN, NAN, NAN);
  return Eigen::Quaterniond(w, x, y, z);
}

builtin_interfaces::msg::Time at(double s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(s));
  t.nanosec = static_cast<uint32_t>(std::llround((s - std::floor(s)) * 1e9));
  return t;
}

// a candidate 0.5 m in front of a camera looking straight down, square-on (approach = optical +Z)
Candidate onCamera(const std::string& id, const builtin_interfaces::msg::Time& stamp)
{
  Candidate c;
  c.header.frame_id = "wrist_cam";
  c.header.stamp = stamp;
  c.id = id;
  c.provider = "tp.surface_grasp";
  c.grasp_pose.position.z = 0.5;
  c.grasp_pose.orientation.w = 1.0;    // its +Z = the approach: a full orientation consistent with it
  c.approach.z = 1.0;
  c.standoff_m = 0.05;
  c.confidence = 0.9;
  c.score = 0.8;
  c.eef = "main";
  c.site = "suction";
  return c;
}

// a producer with a latched array (published every 20 ms) and an on-demand service
class Producer
{
public:
  Producer()
  {
    node = std::make_shared<rclcpp::Node>("m4_producer", "/");
    pub = node->create_publisher<Candidates>("/perception/m4grasp/grasp_candidates",
                                             rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    timer = node->create_wall_timer(std::chrono::milliseconds(20), [this] {
      std::lock_guard<std::mutex> g(m);
      if (continuous && cands) pub->publish(*cands);
    });
    srv = node->create_service<std_srvs::srv::Trigger>(
      "/perception/m4grasp/detect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        std::lock_guard<std::mutex> g(m);
        ++triggers;
        if (!cands) { r->success = false; r->message = "no image yet"; return; }
        Candidates out = *cands;
        if (stamp_now)
        {
          out.header.stamp = node->now();
          for (auto& c : out.candidates) c.header.stamp = out.header.stamp;
        }
        pub->publish(out);
        r->success = true;
        r->message = "sampled";
      });
  }
  void set(std::optional<Candidates> c, bool cont, bool now_stamp)
  {
    std::lock_guard<std::mutex> g(m);
    cands = std::move(c);
    continuous = cont;
    stamp_now = now_stamp;
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Candidates>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv;
  std::mutex m;
  std::optional<Candidates> cands;
  bool continuous{true};
  bool stamp_now{false};
  std::atomic<int> triggers{0};
};

class Consumer : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("m4_consumer", "/");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // a camera on the ARM: at t = 100.0 s it is 0.6 m above base_link's origin, at t = 100.5 s 0.2 m further in +X;
    // looking straight down (optical z = -base z, x = -base y, y = -base x)
    for (const auto& [t, x] : {std::make_pair(100.0, 0.0), std::make_pair(100.5, 0.2)})
    {
      geometry_msgs::msg::TransformStamped T;
      T.header.frame_id = "base_link";
      T.header.stamp = at(t);
      T.child_frame_id = "wrist_cam";
      T.transform.translation.x = x;
      T.transform.translation.z = 0.6;
      T.transform.rotation.x = std::sqrt(0.5);
      T.transform.rotation.y = -std::sqrt(0.5);
      T.transform.rotation.w = 0.0;
      tf_->setTransform(T, "test", false);
    }
    tf_->setUsingDedicatedThread(true);
    ctx_.node = node_;
    ctx_.tf = tf_;
    producer_ = std::make_unique<Producer>();
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
    exec_->add_node(node_);
    exec_->add_node(producer_->node);
    spinner_ = std::thread([] { exec_->spin(); });
  }
  static void TearDownTestSuite()
  {
    exec_->cancel();
    spinner_.join();
    exec_.reset();
    producer_.reset();
    ctx_ = trainit::BtContext{};
    node_.reset();
    tf_.reset();
    rclcpp::shutdown();
  }
  static std::string tick(const std::string& node_xml, const BT::Blackboard::Ptr& bb,
                          trainit::BtContext* ctx = nullptr)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, ctx ? ctx : &ctx_);
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         node_xml + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    while (s == BT::NodeStatus::RUNNING) s = tree.tickOnce();
    return BT::toStr(s);
  }
  static std::string detect(const std::string& extra = "")
  {
    return "<DetectCandidates source=\"candidates\" detector=\"m4grasp\" target_frame=\"base_link\" timeout_ms=\"2000\" "
           "keep=\"1\" out_key=\"cands\" " + extra + " reason=\"{r}\" fault=\"{f}\"/>";
  }
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static trainit::BtContext ctx_;
  static std::unique_ptr<Producer> producer_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spinner_;
};
rclcpp::Node::SharedPtr Consumer::node_;
std::shared_ptr<tf2_ros::Buffer> Consumer::tf_;
trainit::BtContext Consumer::ctx_;
std::unique_ptr<Producer> Consumer::producer_;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> Consumer::exec_;
std::thread Consumer::spinner_;

Candidates arrayOf(const Candidate& c)
{
  Candidates a;
  a.header = c.header;
  a.candidates = {c};
  return a;
}
}  // namespace

// --- TS: TF at the observation stamp ----------------------------------------------------------------------------
TEST_F(Consumer, TS_01_a_candidate_is_mapped_through_the_camera_pose_at_its_own_stamp_not_the_latest)
{
  producer_->set(arrayOf(onCamera("early", at(100.0))), true, false);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect(), bb), "SUCCESS") << bbStr(bb, "r");
  // at 100.0 s the camera was over x = 0: the point is (0, 0, 0.1); the LATEST pose (x = 0.2) would say 0.2
  EXPECT_NEAR((vecOf(bb, "cands.0.position") - Eigen::Vector3d(0.0, 0.0, 0.1)).norm(), 0.0, 1e-9)
    << bbStr(bb, "cands.0.position");
  EXPECT_NEAR((vecOf(bb, "cands.0.approach") - Eigen::Vector3d(0, 0, -1)).norm(), 0.0, 1e-9);

  producer_->set(arrayOf(onCamera("mid", at(100.25))), true, false);
  auto bb2 = BT::Blackboard::create();
  ASSERT_EQ(tick(detect(), bb2), "SUCCESS");
  EXPECT_NEAR(vecOf(bb2, "cands.0.position").x(), 0.1, 1e-9);            // interpolated at its stamp
}

TEST_F(Consumer, TS_02_an_unstamped_candidate_takes_the_latest_transform)
{
  producer_->set(arrayOf(onCamera("unstamped", at(0.0))), true, false);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect(), bb), "SUCCESS");
  EXPECT_NEAR(vecOf(bb, "cands.0.position").x(), 0.2, 1e-9);
}

TEST_F(Consumer, TS_03_a_stamp_outside_the_tf_history_is_a_tf_failure_never_the_latest)
{
  producer_->set(arrayOf(onCamera("ancient", at(50.0))), true, false);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect(), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "TF_LOOKUP_FAILED");
  EXPECT_EQ(bbStr(bb, "cands.count"), "0");
}

// --- CP: the capture ---------------------------------------------------------------------------------------------
TEST_F(Consumer, CP_01_a_capture_triggers_the_producer_and_takes_its_fresh_result)
{
  // the candidate is published ONLY on demand, stamped when sampled (the node clock, in the TF's future: the
  // static-in-time camera of 100.x s cannot map it, so this test checks the capture and maps through no TF)
  Candidate c = onCamera("fresh", at(0.0));
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), false, true);
  const int before = producer_->triggers.load();
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect("trigger=\"true\""), bb), "SUCCESS") << bbStr(bb, "r");
  EXPECT_GT(producer_->triggers.load(), before);
  EXPECT_EQ(bbStr(bb, "cands.0.id"), "fresh");
}

TEST_F(Consumer, CP_02_a_result_stamped_before_the_capture_began_is_rejected)
{
  Candidate c = onCamera("stale", at(1.0));      // stamped long ago, republished on every trigger
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), false, false);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("trigger=\"true\""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
}

TEST_F(Consumer, CP_03_no_service_and_a_trigger_without_a_detector_are_refused)
{
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick("<DetectCandidates source=\"candidates\" detector=\"nobody\" timeout_ms=\"600\" out_key=\"cands\" "
                 "trigger=\"true\" reason=\"{r}\" fault=\"{f}\"/>", bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "TRIGGER_UNAVAILABLE");
  auto bb2 = BT::Blackboard::create();
  EXPECT_EQ(tick("<DetectCandidates source=\"candidates\" topic=\"/x\" out_key=\"cands\" trigger=\"true\" "
                 "reason=\"{r}\"/>", bb2), "FAILURE");
  EXPECT_EQ(bbStr(bb2, "r"), "INVALID_CONFIGURATION");
  auto bb3 = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("trigger=\"yes\""), bb3), "FAILURE");
  EXPECT_EQ(bbStr(bb3, "r"), "INVALID_CONFIGURATION");
}

// --- OR / EN: SetWaypointFromCandidate ---------------------------------------------------------------------------
namespace
{
// a candidate in base_link with approach (0.6, 0, -0.8), and a tool-down reference (+Z = -base z)
BT::Blackboard::Ptr candidateSet(const Eigen::Quaterniond& own)
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("cands.count", "1");
  bb->set<std::string>("cands.0.position", "0.4;0;0.1");
  bb->set<std::string>("cands.0.approach", "0.6;0;-0.8");
  bb->set<std::string>("cands.0.orientation", trainit::formatNumbers({own.x(), own.y(), own.z(), own.w()}));
  bb->set<std::string>("cands.0.frame", "base_link");
  bb->set<std::string>("cands.0.standoff_m", "0.05");
  bb->set<std::string>("cands.0.eef", "main");
  bb->set<std::string>("cands.0.site", "suction");
  bb->set<std::string>("cands.0.id", "c0");
  bb->set<std::string>("cands.0.provider", "tp.surface_grasp");
  bb->set<std::string>("ref.orientation", "1;0;0;0");
  bb->set<std::string>("ref.frame", "base_link");
  return bb;
}

const Eigen::Vector3d kApproach(0.6, 0.0, -0.8);

// an orientation whose +Z is the approach, with a yaw the producer chose (not the reference's)
Eigen::Quaterniond producerOwn()
{
  const Eigen::Quaterniond tilt = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), kApproach);
  return (tilt * Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()))).normalized();
}

std::string setWp(const std::string& orientation, const std::string& extra)
{
  return "<SetWaypointFromCandidate from=\"cands\" index=\"0\" grasp=\"g\" pregrasp=\"pg\" orientation=\"" +
         orientation + "\" eef=\"main\" site=\"suction\" " + extra + " reason=\"{r}\"/>";
}

const double kTiltDeg = std::acos(0.8) * 180.0 / M_PI;     // the approach vs the tool-down reference: 36.87 deg
}  // namespace

TEST_F(Consumer, OR_01_candidate_mode_uses_the_producers_own_orientation_exactly)
{
  const Eigen::Quaterniond own = producerOwn();
  auto bb = candidateSet(own);
  ASSERT_EQ(tick(setWp("candidate", ""), bb), "SUCCESS") << bbStr(bb, "r");
  const Eigen::Quaterniond q = quatOf(bb, "g.orientation");
  EXPECT_NEAR(q.angularDistance(own), 0.0, 1e-9);
  EXPECT_NEAR((q * Eigen::Vector3d::UnitZ() - kApproach).norm(), 0.0, 1e-9);
  EXPECT_NEAR((vecOf(bb, "pg.position") - Eigen::Vector3d(0.37, 0.0, 0.14)).norm(), 0.0, 1e-9);
  // align_approach on the same candidate REPLACES the yaw with the reference's (M3.6-1, unchanged)
  auto bb2 = candidateSet(own);
  ASSERT_EQ(tick(setWp("align_approach", "orientation_from=\"ref\""), bb2), "SUCCESS");
  EXPECT_GT(quatOf(bb2, "g.orientation").angularDistance(own), 0.1);
}

TEST_F(Consumer, OR_02_an_orientation_that_does_not_carry_the_approach_is_refused_never_used)
{
  auto bb = candidateSet(Eigen::Quaterniond::Identity());            // +Z = base +Z, the approach is not
  EXPECT_EQ(tick(setWp("candidate", ""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "CANDIDATE_ORIENTATION_INCONSISTENT");
  EXPECT_EQ(bbStr(bb, "g.position"), "<unset>");
  // just inside / just outside the 1e-3 rad consistency tolerance
  const Eigen::Quaterniond in = Eigen::Quaterniond(Eigen::AngleAxisd(0.9e-3, Eigen::Vector3d::UnitY())) * producerOwn();
  const Eigen::Quaterniond out = Eigen::Quaterniond(Eigen::AngleAxisd(1.1e-3, Eigen::Vector3d::UnitY())) * producerOwn();
  auto bb_in = candidateSet(in.normalized());
  EXPECT_EQ(tick(setWp("candidate", ""), bb_in), "SUCCESS") << bbStr(bb_in, "r");
  auto bb_out = candidateSet(out.normalized());
  EXPECT_EQ(tick(setWp("candidate", ""), bb_out), "FAILURE");
}

TEST_F(Consumer, OR_03_an_unknown_mode_is_refused)
{
  auto bb = candidateSet(producerOwn());
  EXPECT_EQ(tick(setWp("whatever", "orientation_from=\"ref\""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "INVALID_CONFIGURATION");
}

TEST_F(Consumer, EN_01_the_envelope_boundary_is_inclusive_and_past_it_is_refused_in_both_modes)
{
  for (const char* mode : {"align_approach", "candidate"})
  {
    auto ok = candidateSet(producerOwn());
    const std::string at_limit = trainit::formatNumbers({kTiltDeg + 1e-9});
    EXPECT_EQ(tick(setWp(mode, "orientation_from=\"ref\" max_tilt_deg=\"" + at_limit + "\""), ok), "SUCCESS")
      << mode << " " << bbStr(ok, "r");
    auto refused = candidateSet(producerOwn());
    const std::string below = trainit::formatNumbers({kTiltDeg - 1e-6});
    EXPECT_EQ(tick(setWp(mode, "orientation_from=\"ref\" max_tilt_deg=\"" + below + "\""), refused), "FAILURE") << mode;
    EXPECT_EQ(bbStr(refused, "r"), "APPROACH_OUTSIDE_ENVELOPE") << mode;
    EXPECT_EQ(bbStr(refused, "g.position"), "<unset>") << mode << ": never clamped, nothing written";
  }
}

TEST_F(Consumer, EN_02_the_envelope_needs_a_reference_and_a_sane_angle)
{
  for (const std::string& extra : {std::string("max_tilt_deg=\"30\""),                       // no reference
                                   std::string("orientation_from=\"ref\" max_tilt_deg=\"0\""),
                                   std::string("orientation_from=\"ref\" max_tilt_deg=\"180\""),
                                   std::string("orientation_from=\"ref\" max_tilt_deg=\"-5\""),
                                   std::string("orientation_from=\"ref\" max_tilt_deg=\"abc\"")})
  {
    auto bb = candidateSet(producerOwn());
    EXPECT_EQ(tick(setWp("candidate", extra), bb), "FAILURE") << extra;
    EXPECT_EQ(bbStr(bb, "r"), "INVALID_CONFIGURATION") << extra;
  }
}

TEST_F(Consumer, EN_03_a_candidate_for_another_site_is_still_refused)
{
  auto bb = candidateSet(producerOwn());
  bb->set<std::string>("cands.0.site", "jaw");
  EXPECT_EQ(tick(setWp("candidate", ""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "CANDIDATE_SITE_MISMATCH");
}

// --- NK: the node-keyed feed ---------------------------------------------------------------------------------------
TEST_F(Consumer, NK_01_a_second_node_gets_its_own_feed_after_the_first_is_gone)
{
  Candidate c = onCamera("nk", at(0.0));
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), true, false);
  for (int round = 0; round < 2; ++round)
  {
    auto other = std::make_shared<rclcpp::Node>("m4_consumer_" + std::to_string(round), "/");
    exec_->add_node(other);
    trainit::BtContext ctx;
    ctx.node = other;
    ctx.tf = tf_;
    auto bb = BT::Blackboard::create();
    EXPECT_EQ(tick(detect(), bb, &ctx), "SUCCESS") << "round " << round << ": " << bbStr(bb, "r");
    exec_->remove_node(other);
    other.reset();                                   // gone: its feed must never be handed to the next node
  }
}

// --- PF: the pure helpers ------------------------------------------------------------------------------------------
TEST(M4Pure, PF_01_tilt_and_orientation_consistency)
{
  const Eigen::Quaterniond down(0, 1, 0, 0);                       // tool +Z = base -Z
  EXPECT_NEAR(trainit::approachTiltDeg(down, Eigen::Vector3d(0, 0, -1)), 0.0, 1e-12);
  EXPECT_NEAR(trainit::approachTiltDeg(down, kApproach), kTiltDeg, 1e-9);
  EXPECT_NEAR(trainit::approachTiltDeg(down, Eigen::Vector3d(0, 0, 1)), 180.0, 1e-9);
  EXPECT_EQ(trainit::candidateOrientationProblem(producerOwn(), kApproach), "");
  EXPECT_NE(trainit::candidateOrientationProblem(Eigen::Quaterniond::Identity(), kApproach), "");
  EXPECT_NE(trainit::candidateOrientationProblem(Eigen::Quaterniond(0.5, 0, 0, 0), kApproach), "");    // not unit
  EXPECT_NE(trainit::candidateOrientationProblem(producerOwn(), Eigen::Vector3d(NAN, 0, 1)), "");
}

// --- LV: a live provider (opt-in) -------------------------------------------------------------------------------
TEST(M4Live, LV_01_a_live_provider_is_consumed_through_the_canonical_nodes)
{
  const char* name = std::getenv("M4_LIVE_PROVIDER");
  if (!name || !*name) GTEST_SKIP() << "M4_LIVE_PROVIDER is not set";
  const char* frame_env = std::getenv("M4_LIVE_TARGET_FRAME");
  const std::string target = frame_env && *frame_env ? frame_env : "base_link";
  const char* tilt_env = std::getenv("M4_LIVE_MAX_TILT");
  const std::string tilt = tilt_env && *tilt_env ? tilt_env : "60";
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  rclcpp::NodeOptions opts;
  const char* sim = std::getenv("M4_LIVE_SIM_TIME");      // the capture compares stamps with THIS clock
  if (sim && std::string(sim) == "1") opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("m4_live_consumer", "/", opts);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, false);
  tf->setUsingDedicatedThread(true);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread spin([&exec] { exec.spin(); });
  trainit::BtContext ctx;
  ctx.node = node;
  ctx.tf = tf;
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  bb->set<std::string>("ref.orientation", "1;0;0;0");          // tool down
  bb->set<std::string>("ref.frame", target);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\"><Sequence>"
    "<DetectCandidates source=\"candidates\" detector=\"" + std::string(name) + "\" target_frame=\"" + target +
    "\" timeout_ms=\"15000\" keep=\"5\" out_key=\"cands\" trigger=\"true\" reason=\"{r}\" fault=\"{f}\"/>"
    "<SetWaypointFromCandidate from=\"cands\" index=\"0\" grasp=\"g\" pregrasp=\"pg\" orientation=\"candidate\" "
    "orientation_from=\"ref\" max_tilt_deg=\"" + tilt + "\" eef=\"main\" site=\"suction\" reason=\"{r}\"/>"
    "</Sequence></BehaviorTree></root>";
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("T", bb);
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  while (st == BT::NodeStatus::RUNNING) st = tree.tickOnce();
  exec.cancel();
  spin.join();
  const std::string reason = bbStr(bb, "r");
  EXPECT_EQ(BT::toStr(st), "SUCCESS") << reason << " " << bbStr(bb, "f");
  if (st == BT::NodeStatus::SUCCESS)
  {
    const Eigen::Vector3d p = vecOf(bb, "g.position"), pg = vecOf(bb, "pg.position"), a = vecOf(bb, "cands.0.approach");
    const double s = std::stod(bbStr(bb, "cands.0.standoff_m"));
    EXPECT_TRUE(p.allFinite() && pg.allFinite());
    EXPECT_NEAR((pg - (p - s * a)).norm(), 0.0, 1e-9) << "PreGrasp = grasp - standoff * approach";
    const Eigen::Quaterniond q = quatOf(bb, "g.orientation");
    EXPECT_NEAR((q.normalized() * Eigen::Vector3d::UnitZ() - a).norm(), 0.0, 2e-3);
    std::printf("M4_LIVE candidate %s (provider %s) in %s: grasp %s pregrasp %s approach %s stamp %s count %s\n",
                bbStr(bb, "cands.0.id").c_str(), bbStr(bb, "cands.0.provider").c_str(), target.c_str(),
                bbStr(bb, "g.position").c_str(), bbStr(bb, "pg.position").c_str(), bbStr(bb, "cands.0.approach").c_str(),
                bbStr(bb, "cands.0.stamp").c_str(), bbStr(bb, "cands.count").c_str());
  }
  rclcpp::shutdown();
}
