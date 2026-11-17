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
        if (!cands || refuse_first > 0)
        {
          if (refuse_first > 0) --refuse_first;
          r->success = false;
          r->message = "no image yet";
          return;
        }
        Candidates out = *cands;
        if (stamp_now)
        {
          out.header.stamp = rclcpp::Time(node->now().nanoseconds() + static_cast<int64_t>(offset_s * 1e9));
          for (auto& c : out.candidates) c.header.stamp = out.header.stamp;
        }
        pub->publish(out);
        r->success = true;
        r->message = "sampled";
      });
  }
  void set(std::optional<Candidates> c, bool cont, bool now_stamp, double offset = 0.0, int refuse = 0)
  {
    std::lock_guard<std::mutex> g(m);
    cands = std::move(c);
    continuous = cont;
    stamp_now = now_stamp;
    offset_s = offset;
    refuse_first = refuse;
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Candidates>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv;
  std::mutex m;
  std::optional<Candidates> cands;
  bool continuous{true};
  bool stamp_now{false};
  double offset_s{0.0};     // a sampled result is stamped now + offset_s
  int refuse_first{0};      // answer "could not sample" to this many triggers first
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
  static std::string detect(const std::string& extra = "", int keep = 1)
  {
    return "<DetectCandidates source=\"candidates\" detector=\"m4grasp\" target_frame=\"base_link\" timeout_ms=\"2000\" "
           "keep=\"" + std::to_string(keep) + "\" out_key=\"cands\" " + extra + " reason=\"{r}\" fault=\"{f}\"/>";
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

TEST_F(Consumer, TS_04_two_candidates_of_one_frame_at_two_stamps_each_take_their_own_pose)
{
  Candidates a;
  a.header = onCamera("x", at(100.0)).header;
  a.candidates = {onCamera("early", at(100.0)), onCamera("late", at(100.5))};
  producer_->set(a, true, false);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect("", 2), bb), "SUCCESS") << bbStr(bb, "r");
  EXPECT_NEAR(vecOf(bb, "cands.0.position").x(), 0.0, 1e-9);           // the camera over x = 0 at 100.0 s
  EXPECT_NEAR(vecOf(bb, "cands.1.position").x(), 0.2, 1e-9);           // and over x = 0.2 at 100.5 s
}

TEST_F(Consumer, TS_05_the_candidates_own_orientation_is_transformed_with_it_and_used_by_candidate_mode)
{
  // in the camera frame: approach = optical +Z, orientation = 30 deg about +Z (a producer's own yaw); the camera
  // looks down, so in base_link the approach is -Z and the TCP's +Z must be that, its yaw the producer's
  Candidate c = onCamera("own", at(100.0));
  const Eigen::Quaterniond own(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
  c.grasp_pose.orientation.x = own.x();
  c.grasp_pose.orientation.y = own.y();
  c.grasp_pose.orientation.z = own.z();
  c.grasp_pose.orientation.w = own.w();
  producer_->set(arrayOf(c), true, false);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect(), bb), "SUCCESS") << bbStr(bb, "r");
  bb->set<std::string>("ref.orientation", "1;0;0;0");
  bb->set<std::string>("ref.frame", "base_link");
  ASSERT_EQ(tick("<SetWaypointFromCandidate from=\"cands\" index=\"0\" grasp=\"g\" pregrasp=\"pg\" "
                 "orientation=\"candidate\" orientation_from=\"ref\" max_tilt_deg=\"10\" eef=\"main\" "
                 "site=\"suction\" reason=\"{r}\"/>", bb), "SUCCESS") << bbStr(bb, "r");
  const Eigen::Quaterniond R_cam(0.0, std::sqrt(0.5), -std::sqrt(0.5), 0.0);     // base_link <- wrist_cam
  const Eigen::Quaterniond q = quatOf(bb, "g.orientation");
  EXPECT_NEAR(q.angularDistance(R_cam * own), 0.0, 1e-9);
  EXPECT_NEAR((q * Eigen::Vector3d::UnitZ() - Eigen::Vector3d(0, 0, -1)).norm(), 0.0, 1e-9);
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
  Candidate c = onCamera("stale", at(0.0));      // stamped 2 s before each trigger: a slow producer, not a clock
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), false, true, -2.0);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("trigger=\"true\""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
  EXPECT_EQ(bbStr(bb, "cands.count"), "0");
}

TEST_F(Consumer, CP_04_a_result_on_another_clock_is_a_clock_mismatch_ahead_or_long_behind)
{
  Candidate c = onCamera("other_clock", at(0.0));
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), false, true, +5.0);                       // 5 s AHEAD of this clock
  auto ahead = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("trigger=\"true\""), ahead), "FAILURE");
  EXPECT_EQ(bbStr(ahead, "f"), "CLOCK_MISMATCH");
  Candidate old = onCamera("sim_time", at(12.5));                      // a sim-time stamp seen by a wall clock
  old.header.frame_id = "base_link";
  producer_->set(arrayOf(old), false, false);
  auto behind = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("trigger=\"true\""), behind), "FAILURE");
  EXPECT_EQ(bbStr(behind, "f"), "CLOCK_MISMATCH");
}

TEST_F(Consumer, CP_05_a_producer_that_could_not_sample_is_triggered_again_and_its_fresh_result_taken)
{
  Candidate c = onCamera("after_refusal", at(0.0));
  c.header.frame_id = "base_link";
  producer_->set(arrayOf(c), false, true, 0.0, 1);                    // the first trigger: "no image yet"
  const int before = producer_->triggers.load();
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect("trigger=\"true\""), bb), "SUCCESS") << bbStr(bb, "r");
  EXPECT_GE(producer_->triggers.load() - before, 2);
  EXPECT_EQ(bbStr(bb, "cands.0.id"), "after_refusal");
}

TEST_F(Consumer, CP_06_a_capture_on_a_clock_that_is_not_running_is_refused)
{
  // in the suite's own context (a test that shut the context down and re-initialised it would destroy cached
  // entities of the dead context -- a test artefact a runner never meets)
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});  // no /clock on this domain: time 0
  auto node = std::make_shared<rclcpp::Node>("m4_no_clock", "/", opts);
  trainit::BtContext ctx;
  ctx.node = node;
  ctx.tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  BT::BehaviorTreeFactory factory;
  trainit::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx);
  factory.registerBehaviorTreeFromText(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\"><DetectCandidates source=\"candidates\" "
    "detector=\"m4_nobody\" timeout_ms=\"500\" out_key=\"cands\" trigger=\"true\" reason=\"{r}\" fault=\"{f}\"/>"
    "</BehaviorTree></root>");
  auto tree = factory.createTree("T", bb);
  EXPECT_EQ(BT::toStr(tree.tickOnce()), "FAILURE");
  EXPECT_EQ(bbStr(bb, "f"), "CLOCK_UNAVAILABLE");
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
  EXPECT_EQ(bbStr(bb, "pg.position"), "<unset>");
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

TEST_F(Consumer, EN_01b_an_approach_exactly_at_the_limit_is_accepted_and_a_hair_inside_refused)
{
  // a tool-down reference and a horizontal approach: the tilt is EXACTLY 90 deg (atan2(1, 0))
  for (const char* mode : {"align_approach", "candidate"})
  {
    const Eigen::Quaterniond own(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));   // +Z -> +X
    auto ok = candidateSet(own);
    ok->set<std::string>("cands.0.approach", "1;0;0");
    EXPECT_EQ(tick(setWp(mode, "orientation_from=\"ref\" max_tilt_deg=\"90\""), ok), "SUCCESS")
      << mode << " " << bbStr(ok, "r");
    auto no = candidateSet(own);
    no->set<std::string>("cands.0.approach", "1;0;0");
    EXPECT_EQ(tick(setWp(mode, "orientation_from=\"ref\" max_tilt_deg=\"89.999999\""), no), "FAILURE") << mode;
    EXPECT_EQ(bbStr(no, "r"), "APPROACH_OUTSIDE_ENVELOPE") << mode;
    EXPECT_EQ(bbStr(no, "pg.position"), "<unset>") << mode;
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
  // a topic NO other test (and not the fixture's node) ever subscribes to: with a per-topic cache, round 1 would be
  // handed round 0's feed -- a subscription on a node that is no longer spun -- and time out
  auto pnode = std::make_shared<rclcpp::Node>("m4_nk_producer", "/");
  auto pub = pnode->create_publisher<Candidates>("/perception/m4nk/grasp_candidates",
                                                  rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  Candidate c = onCamera("nk", at(0.0));
  c.header.frame_id = "base_link";
  auto timer = pnode->create_wall_timer(std::chrono::milliseconds(20), [&] { pub->publish(arrayOf(c)); });
  exec_->add_node(pnode);
  for (int round = 0; round < 3; ++round)
  {
    auto other = std::make_shared<rclcpp::Node>("m4_consumer_nk_" + std::to_string(round), "/");
    exec_->add_node(other);
    auto bb = BT::Blackboard::create();
    {
      trainit::BtContext ctx;
      ctx.node = other;
      ctx.tf = tf_;
      EXPECT_EQ(tick("<DetectCandidates source=\"candidates\" detector=\"m4nk\" target_frame=\"base_link\" "
                     "timeout_ms=\"1500\" out_key=\"cands\" reason=\"{r}\" fault=\"{f}\"/>", bb, &ctx), "SUCCESS")
        << "round " << round << ": " << bbStr(bb, "r");
    }
    exec_->remove_node(other);
    other.reset();                                   // gone: its feed must never be handed to the next node
  }
  exec_->remove_node(pnode);
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
  // on sim time the clock is 0 until the first /clock: a capture refuses that (CLOCK_UNAVAILABLE), so wait for it as a
  // runner's start-up does (bounded)
  for (int i = 0; i < 200 && node->now().nanoseconds() <= 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
