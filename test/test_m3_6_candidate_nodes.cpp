// test_m3_6_candidate_nodes (vNext M3.6, ADR-0026): DetectCandidates and SetWaypointFromCandidate
// ticked for real -- DetectCandidates against in-process producers on a private domain (a synthetic
// GraspCandidateArray producer and a synthetic Detection3DArray detector), SetWaypointFromCandidate on
// a bare blackboard. Explicit numbers: a sign or frame error is visible.
//
// The cell: base_link; a camera `cam_optical` at (0.4, 0, 0.6) looking DOWN (optical x = -y, y = -x,
// z = -z of base_link); a frame `cell` = base_link turned 90 deg about Z and moved by (0.2, 0, 0).
// The reference object: at (0.4, 0, 0.1) in base_link on a surface whose normal is (-0.6, 0, 0.8)
// (it faces the camera): its approach is (0.6, 0, -0.8), INTO the surface. In cam_optical the point is
// (0, 0, 0.5) and the normal (0, 0.6, -0.8); in `cell` the point is (0, -0.2, 0.1) and the approach
// (0, -0.6, -0.8).
//
//   DC  DetectCandidates: both sources land on the same keys in base_link; published order kept, no
//       sort; keep; an empty array is NO_CANDIDATES (no fault); one unusable candidate refuses the
//       array; no normal -> the default approach (base_link axes) or refused; timeout, TF and
//       configuration refusals; an older set never stays readable
//   SW  SetWaypointFromCandidate: grasp, PreGrasp = p - s a, Retract along the approach or up +Z,
//       align_approach on the reference, the standoff override, every refusal (nothing written)
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <trainit_interfaces/msg/grasp_candidate_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/grasp_candidates.hpp"

namespace
{
using Candidates = trainit_interfaces::msg::GraspCandidateArray;
using Candidate = trainit_interfaces::msg::GraspCandidate;
using Detections = vision_msgs::msg::Detection3DArray;

const Eigen::Vector3d kP(0.4, 0.0, 0.1);         // base_link
const Eigen::Vector3d kA(0.6, 0.0, -0.8);        // base_link: the approach
const Eigen::Vector3d kNormalOpt(0.0, 0.6, -0.8);   // cam_optical

// publishes whatever it is given, every 20 ms
class Producer
{
public:
  Producer()
  {
    node = std::make_shared<rclcpp::Node>("m36_producer", "/");
    cpub = node->create_publisher<Candidates>("/perception/m36grasp/grasp_candidates",
                                              rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    dpub = node->create_publisher<Detections>("/perception/m36cam/detections",
                                              rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    timer = node->create_wall_timer(std::chrono::milliseconds(20), [this] {
      std::lock_guard<std::mutex> g(m);
      if (cands) cpub->publish(*cands);
      if (dets) dpub->publish(*dets);
    });
  }
  void set(std::optional<Candidates> c, std::optional<Detections> d)
  {
    std::lock_guard<std::mutex> g(m);
    cands = std::move(c);
    dets = std::move(d);
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Candidates>::SharedPtr cpub;
  rclcpp::Publisher<Detections>::SharedPtr dpub;
  rclcpp::TimerBase::SharedPtr timer;
  std::mutex m;
  std::optional<Candidates> cands;
  std::optional<Detections> dets;
};

Candidate inCell(const std::string& id, double score)
{
  Candidate c;
  c.header.frame_id = "cell";
  c.id = id;
  c.provider = "synthetic";
  c.grasp_pose.position.y = -0.2;
  c.grasp_pose.position.z = 0.1;
  c.grasp_pose.orientation.w = 1.0;
  c.approach.y = -0.6;
  c.approach.z = -0.8;
  c.standoff_m = 0.05;
  c.score = score;
  c.confidence = 0.9;
  c.object_class = "part";
  return c;
}

vision_msgs::msg::Detection3D detection(const Eigen::Quaterniond& q, double score = 0.9)
{
  vision_msgs::msg::Detection3D d;
  d.header.frame_id = "cam_optical";
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = "part";
  h.hypothesis.score = score;
  h.pose.pose.position.z = 0.5;
  h.pose.pose.orientation.x = q.x();
  h.pose.pose.orientation.y = q.y();
  h.pose.pose.orientation.z = q.z();
  h.pose.pose.orientation.w = q.w();
  d.results.push_back(h);
  return d;
}

Eigen::Quaterniond normalQuat()
{
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), kNormalOpt.normalized());
}

std::string bbStr(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  try { return bb->get<std::string>(key); } catch (const std::exception&) { return "<unset>"; }
}

Eigen::Vector3d vecOf(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  const std::string s = bbStr(bb, key);
  double x = NAN, y = NAN, z = NAN;
  if (std::sscanf(s.c_str(), "%lf;%lf;%lf", &x, &y, &z) != 3) return {NAN, NAN, NAN};
  return {x, y, z};
}

Eigen::Quaterniond quatOf(const BT::Blackboard::Ptr& bb, const std::string& key)
{
  const std::string s = bbStr(bb, key);
  double x = NAN, y = NAN, z = NAN, w = NAN;
  if (std::sscanf(s.c_str(), "%lf;%lf;%lf;%lf", &x, &y, &z, &w) != 4) return Eigen::Quaterniond(NAN, NAN, NAN, NAN);
  return Eigen::Quaterniond(w, x, y, z);
}

class CandidateNodes : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("m36_candidate_nodes", "/");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped T;
    T.header.frame_id = "base_link";
    T.child_frame_id = "cam_optical";
    T.transform.translation.x = 0.4;
    T.transform.translation.z = 0.6;
    T.transform.rotation.x = std::sqrt(0.5);   // 180 deg about (1, -1, 0): x -> -y, y -> -x, z -> -z
    T.transform.rotation.y = -std::sqrt(0.5);
    T.transform.rotation.z = 0.0;
    T.transform.rotation.w = 0.0;
    tf_->setTransform(T, "test", true);
    T.child_frame_id = "cell";
    T.transform.translation.x = 0.2;
    T.transform.translation.z = 0.0;
    T.transform.rotation.x = 0.0;
    T.transform.rotation.y = 0.0;
    T.transform.rotation.z = std::sqrt(0.5);   // 90 deg about Z
    T.transform.rotation.w = std::sqrt(0.5);
    tf_->setTransform(T, "test", true);
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

  static std::string tick(const std::string& node_xml, const BT::Blackboard::Ptr& bb)
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    factory.registerBehaviorTreeFromText("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" +
                                         node_xml + "</BehaviorTree></root>");
    auto tree = factory.createTree("T", bb);
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    while (s == BT::NodeStatus::RUNNING) s = tree.tickOnce();
    return BT::toStr(s);
  }
  static std::string detect(const std::string& source, const std::string& extra = "", int keep = 1)
  {
    const std::string det = source == "candidates" ? "m36grasp" : "m36cam";
    return "<DetectCandidates source=\"" + source + "\" detector=\"" + det + "\" target_frame=\"base_link\" "
           "timeout_ms=\"1500\" keep=\"" + std::to_string(keep) + "\" out_key=\"cands\" " + extra +
           " reason=\"{r}\" fault=\"{f}\"/>";
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static trainit::BtContext ctx_;
  static std::unique_ptr<Producer> producer_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spinner_;
};
rclcpp::Node::SharedPtr CandidateNodes::node_;
std::shared_ptr<tf2_ros::Buffer> CandidateNodes::tf_;
trainit::BtContext CandidateNodes::ctx_;
std::unique_ptr<Producer> CandidateNodes::producer_;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> CandidateNodes::exec_;
std::thread CandidateNodes::spinner_;

void expectNear(const Eigen::Vector3d& got, const Eigen::Vector3d& want, const std::string& what, double tol = 1e-9)
{
  EXPECT_NEAR((got - want).norm(), 0.0, tol) << what << ": got " << got.transpose() << ", want " << want.transpose();
}
}  // namespace

// --- DC: DetectCandidates ---------------------------------------------------------------------------
TEST_F(CandidateNodes, DC_01_a_producers_candidates_land_in_base_link_in_published_order_never_sorted)
{
  Candidates a;
  a.candidates = {inCell("low_score_first", 0.5), inCell("best_score_second", 0.9), inCell("third", 0.7)};
  a.candidates[1].grasp_pose.position.z = 0.2;   // tells the entries apart
  producer_->set(a, std::nullopt);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect("candidates", "", 2), bb), "SUCCESS");
  EXPECT_EQ(bbStr(bb, "cands.count"), "2");                               // keep = 2 of 3
  EXPECT_EQ(bbStr(bb, "cands.0.id"), "low_score_first");                  // the producer's order IS the rank
  EXPECT_EQ(bbStr(bb, "cands.1.id"), "best_score_second");
  EXPECT_EQ(bbStr(bb, "cands.2.id"), "<unset>");
  expectNear(vecOf(bb, "cands.0.position"), kP, "position in base_link");
  expectNear(vecOf(bb, "cands.0.approach"), kA, "approach in base_link");
  expectNear(vecOf(bb, "cands.1.position"), Eigen::Vector3d(0.4, 0.0, 0.2), "candidate 1");
  EXPECT_EQ(bbStr(bb, "cands.0.frame"), "base_link");
  EXPECT_EQ(bbStr(bb, "cands.0.provider"), "synthetic");
  EXPECT_EQ(bbStr(bb, "cands.0.standoff_m"), "0.05");
  EXPECT_EQ(bbStr(bb, "cands.0.eef"), "");
  EXPECT_EQ(bbStr(bb, "cands.0.site"), "");
  EXPECT_EQ(bbStr(bb, "cands.0.score"), "0.5");
  EXPECT_EQ(bbStr(bb, "cands.0.object_class"), "part");
  // the candidate's identity orientation in `cell` is base_link's 90 deg about Z
  EXPECT_TRUE(quatOf(bb, "cands.0.orientation").isApprox(
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())), 1e-9));
  EXPECT_EQ(bbStr(bb, "r"), "<unset>");
}

TEST_F(CandidateNodes, DC_02_an_empty_array_is_no_candidate_a_verdict_without_a_fault)
{
  producer_->set(Candidates{}, std::nullopt);
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("cands.count", "3");   // an older set: never readable after this tick
  EXPECT_EQ(tick(detect("candidates"), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "NO_CANDIDATES");
  EXPECT_EQ(bbStr(bb, "f"), "<unset>");
  EXPECT_EQ(bbStr(bb, "cands.count"), "0");
}

TEST_F(CandidateNodes, DC_03_one_unusable_candidate_refuses_the_whole_array)
{
  Candidates a;
  a.candidates = {inCell("ok", 0.9), inCell("broken", 0.8)};
  a.candidates[1].approach.y = 0.0;
  a.candidates[1].approach.z = 0.0;   // a zero approach
  producer_->set(a, std::nullopt);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("candidates", "", 1), bb), "FAILURE");   // even with keep = 1: never skipped
  EXPECT_EQ(bbStr(bb, "r"), "INVALID_CANDIDATE");
  EXPECT_EQ(bbStr(bb, "f"), "INVALID_CANDIDATE");
  EXPECT_EQ(bbStr(bb, "cands.count"), "0");
  EXPECT_EQ(bbStr(bb, "cands.0.position"), "<unset>");
  a.candidates[1] = inCell("ok", 0.8);   // a duplicate id
  producer_->set(a, std::nullopt);
  EXPECT_EQ(tick(detect("candidates"), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "INVALID_CANDIDATE");
}

TEST_F(CandidateNodes, DC_04_a_fitted_detection_converges_on_the_same_candidate_keys)
{
  Detections d;
  d.header.frame_id = "cam_optical";
  d.detections = {detection(normalQuat())};
  producer_->set(std::nullopt, d);
  auto bb = BT::Blackboard::create();
  ASSERT_EQ(tick(detect("detections", "standoff_m=\"0.05\" eef=\"main\" site=\"main\""), bb), "SUCCESS");
  EXPECT_EQ(bbStr(bb, "cands.count"), "1");
  expectNear(vecOf(bb, "cands.0.position"), kP, "position in base_link");
  expectNear(vecOf(bb, "cands.0.approach"), kA, "approach = -normal, in base_link");
  expectNear(vecOf(bb, "cands.0.normal"), Eigen::Vector3d(-0.6, 0.0, 0.8), "normal in base_link");
  EXPECT_EQ(bbStr(bb, "cands.0.id"), "detection_0");
  EXPECT_EQ(bbStr(bb, "cands.0.provider"), "tmr.candidates_from_detections");
  EXPECT_EQ(bbStr(bb, "cands.0.eef"), "main");
  EXPECT_EQ(bbStr(bb, "cands.0.site"), "main");
  EXPECT_EQ(bbStr(bb, "cands.0.standoff_m"), "0.05");
}

TEST_F(CandidateNodes, DC_05_no_normal_takes_the_default_approach_in_target_frame_axes_else_it_is_refused)
{
  Detections d;
  d.header.frame_id = "cam_optical";
  d.detections = {detection(Eigen::Quaterniond::Identity())};
  producer_->set(std::nullopt, d);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("detections"), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "INVALID_CANDIDATE");
  ASSERT_EQ(tick(detect("detections", "default_approach=\"0;0;-1\""), bb), "SUCCESS");
  expectNear(vecOf(bb, "cands.0.approach"), Eigen::Vector3d(0, 0, -1), "the default, in base_link axes");
  expectNear(vecOf(bb, "cands.0.normal"), Eigen::Vector3d(0, 0, 0), "no normal");
  // a zero quaternion is a broken detection, never "no normal"
  d.detections = {detection(Eigen::Quaterniond(0, 0, 0, 0))};
  producer_->set(std::nullopt, d);
  EXPECT_EQ(tick(detect("detections", "default_approach=\"0;0;-1\""), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "INVALID_CANDIDATE");
}

TEST_F(CandidateNodes, DC_06_timeout_tf_and_configuration_refusals)
{
  producer_->set(std::nullopt, std::nullopt);
  auto bb = BT::Blackboard::create();
  EXPECT_EQ(tick(detect("candidates"), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "DETECTION_TIMEOUT");
  EXPECT_EQ(bbStr(bb, "f"), "DETECTION_TIMEOUT");
  Candidates a;
  a.candidates = {inCell("c", 0.9)};
  a.candidates[0].header.frame_id = "nowhere";
  producer_->set(a, std::nullopt);
  EXPECT_EQ(tick(detect("candidates"), bb), "FAILURE");
  EXPECT_EQ(bbStr(bb, "r"), "TF_LOOKUP_FAILED");
  for (const std::string& xml :
       {std::string("<DetectCandidates detector=\"m36grasp\" out_key=\"cands\" reason=\"{r}\"/>"),     // no source
        std::string("<DetectCandidates source=\"candidates\" detector=\"m36grasp\" reason=\"{r}\"/>"),  // no out_key
        detect("candidates", "class_id=\"part\""),               // a projection port on a producer's array
        detect("candidates", "standoff_m=\"0.05\""), detect("detections", "default_approach=\"0;0;-2\""),
        detect("candidates", "", 0),
        // an unreadable count is refused, never read as the default 1
        std::string("<DetectCandidates source=\"candidates\" detector=\"m36grasp\" keep=\"{unset}\" "
                    "out_key=\"cands\" reason=\"{r}\"/>"),
        std::string("<DetectCandidates source=\"weird\" detector=\"m36grasp\" out_key=\"cands\" reason=\"{r}\"/>")})
  {
    auto b = BT::Blackboard::create();
    b->set<std::string>("cands.count", "2");   // an older set
    EXPECT_EQ(tick(xml, b), "FAILURE") << xml;
    EXPECT_EQ(bbStr(b, "r"), "INVALID_CONFIGURATION") << xml;
    if (xml.find("out_key") != std::string::npos)
    {
      EXPECT_EQ(bbStr(b, "cands.count"), "0") << xml << ": a refused configuration leaves no set readable";
    }
  }
}

// --- SW: SetWaypointFromCandidate -------------------------------------------------------------------
namespace
{
// the reference object as DetectCandidates writes it, and a tool-down reference waypoint
BT::Blackboard::Ptr candidateSet(const std::string& standoff = "0.05", const std::string& site = "")
{
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("cands.count", "1");
  bb->set<std::string>("cands.0.position", "0.4;0;0.1");
  bb->set<std::string>("cands.0.approach", "0.6;0;-0.8");
  bb->set<std::string>("cands.0.frame", "base_link");
  bb->set<std::string>("cands.0.standoff_m", standoff);
  bb->set<std::string>("cands.0.eef", "");
  bb->set<std::string>("cands.0.site", site);
  bb->set<std::string>("cands.0.id", "c0");
  bb->set<std::string>("cands.0.provider", "synthetic");
  bb->set<std::string>("ref.orientation", "1;0;0;0");   // the tool pointing down
  bb->set<std::string>("ref.frame", "base_link");
  return bb;
}

std::string setWp(const std::string& extra, int index = 0)
{
  return "<SetWaypointFromCandidate from=\"cands\" index=\"" + std::to_string(index) + "\" grasp=\"g\" pregrasp=\"pg\" retract=\"rt\" "
         "orientation=\"align_approach\" orientation_from=\"ref\" eef=\"main\" site=\"main\" " + extra +
         " reason=\"{r}\"/>";
}
}  // namespace

TEST_F(CandidateNodes, SW_01_grasp_pregrasp_and_retract_along_the_approach)
{
  auto bb = candidateSet();
  ASSERT_EQ(tick(setWp("retract_along_approach=\"0.1\""), bb), "SUCCESS");
  expectNear(vecOf(bb, "g.position"), kP, "grasp");
  expectNear(vecOf(bb, "pg.position"), Eigen::Vector3d(0.37, 0.0, 0.14), "PreGrasp = p - 0.05 a");
  expectNear(vecOf(bb, "rt.position"), Eigen::Vector3d(0.34, 0.0, 0.18), "Retract = p - 0.1 a");
  for (const char* wp : {"g", "pg", "rt"})
  {
    EXPECT_EQ(bbStr(bb, std::string(wp) + ".type"), "tcp");
    EXPECT_EQ(bbStr(bb, std::string(wp) + ".frame"), "base_link");
    EXPECT_EQ(bbStr(bb, std::string(wp) + ".orientation"), bbStr(bb, "g.orientation"));
  }
  // align_approach: the tool's +Z travels along the approach; the reference tilted by acos(0.8) only
  const Eigen::Quaterniond q = quatOf(bb, "g.orientation");
  expectNear(q.normalized() * Eigen::Vector3d::UnitZ(), kA, "the TCP +Z = the approach");
  EXPECT_NEAR(q.normalized().angularDistance(Eigen::Quaterniond(0, 1, 0, 0)), std::acos(0.8), 1e-9);
}

TEST_F(CandidateNodes, SW_02_the_steps_standoff_replaces_the_candidates_and_retract_dz_goes_up)
{
  auto bb = candidateSet();
  ASSERT_EQ(tick(setWp("standoff_m=\"0.08\" retract_dz=\"0.1\""), bb), "SUCCESS");
  expectNear(vecOf(bb, "pg.position"), Eigen::Vector3d(0.352, 0.0, 0.164), "PreGrasp = p - 0.08 a");
  expectNear(vecOf(bb, "rt.position"), Eigen::Vector3d(0.4, 0.0, 0.2), "Retract = p + 0.1 z");
}

TEST_F(CandidateNodes, SW_03_every_refusal_writes_nothing)
{
  struct Case { std::string what; BT::Blackboard::Ptr bb; std::string xml; std::string reason; };
  auto opposed = candidateSet();
  opposed->set<std::string>("ref.orientation", "0;0;0;1");   // the tool's +Z up
  opposed->set<std::string>("cands.0.approach", "0;0;-1");   // the approach down: exactly against it
  auto wrong_site = candidateSet("0.05", "other");
  auto zero = candidateSet("0");
  auto stale = candidateSet();
  stale->set<std::string>("cands.count", "0");               // a failed DetectCandidates left the keys
  auto long_a = candidateSet();
  long_a->set<std::string>("cands.0.approach", "0;0;-2");
  auto other_frame = candidateSet();
  other_frame->set<std::string>("ref.frame", "world");
  auto no_frame = candidateSet();
  no_frame->set<std::string>("ref.frame", "");   // and no runtime to say which the base frame is
  const std::vector<Case> cases = {
    {"antiparallel reference", opposed, setWp("retract_dz=\"0.1\""), "APPROACH_OPPOSES_REFERENCE"},
    {"another site", wrong_site, setWp("retract_dz=\"0.1\""), "CANDIDATE_SITE_MISMATCH"},
    {"no standoff", zero, setWp("retract_dz=\"0.1\""), "INVALID_STANDOFF"},
    {"a zero override", candidateSet(), setWp("standoff_m=\"0\" retract_dz=\"0.1\""), "INVALID_STANDOFF"},
    {"index past the set", candidateSet(), setWp("retract_dz=\"0.1\"", 1), "NO_CANDIDATE"},
    {"an empty set", stale, setWp("retract_dz=\"0.1\""), "NO_CANDIDATE"},
    {"a non-unit approach", long_a, setWp("retract_dz=\"0.1\""), "INVALID_CANDIDATE"},
    {"the reference in another frame", other_frame, setWp("retract_dz=\"0.1\""), "INVALID_REFERENCE"},
    {"the reference in no frame, no runtime", no_frame, setWp("retract_dz=\"0.1\""), "INVALID_REFERENCE"},
    {"an unreadable index", candidateSet(),
     "<SetWaypointFromCandidate from=\"cands\" index=\"{unset}\" grasp=\"g\" pregrasp=\"pg\" "
     "orientation=\"align_approach\" orientation_from=\"ref\" reason=\"{r}\"/>", "INVALID_CONFIGURATION"},
    {"two retract forms", candidateSet(), setWp("retract_dz=\"0.1\" retract_along_approach=\"0.1\""),
     "INVALID_CONFIGURATION"},
    {"no retract form", candidateSet(), setWp(""), "INVALID_CONFIGURATION"},
    {"another orientation", candidateSet(),
     "<SetWaypointFromCandidate from=\"cands\" grasp=\"g\" pregrasp=\"pg\" orientation=\"candidate\" "
     "orientation_from=\"ref\" reason=\"{r}\"/>", "INVALID_CONFIGURATION"},
    {"no reference", candidateSet(),
     "<SetWaypointFromCandidate from=\"cands\" grasp=\"g\" pregrasp=\"pg\" orientation=\"align_approach\" "
     "reason=\"{r}\"/>", "INVALID_CONFIGURATION"},
    {"an unknown reference", candidateSet(),
     "<SetWaypointFromCandidate from=\"cands\" grasp=\"g\" pregrasp=\"pg\" orientation=\"align_approach\" "
     "orientation_from=\"nope\" reason=\"{r}\"/>", "INVALID_REFERENCE"},
  };
  for (const auto& c : cases)
  {
    EXPECT_EQ(tick(c.xml, c.bb), "FAILURE") << c.what;
    EXPECT_EQ(bbStr(c.bb, "r"), c.reason) << c.what;
    EXPECT_EQ(bbStr(c.bb, "g.position"), "<unset>") << c.what;
    EXPECT_EQ(bbStr(c.bb, "pg.position"), "<unset>") << c.what;
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
