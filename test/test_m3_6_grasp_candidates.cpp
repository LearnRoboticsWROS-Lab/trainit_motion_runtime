// test_m3_6_grasp_candidates (vNext M3.6, ADR-0026): the grasp-candidate contract's pure half
// (grasp_candidates.hpp) -- no ROS graph, explicit numbers.
//
//   V  validation: what makes a candidate unusable, each field, the array's unique ids
//   P  the Detection3D -> GraspCandidate projection (the minimal M3 bridge, never a grasp method):
//        A a fitted normal is read; B the HEMISPHERE: approach = -normal, INTO the surface (the
//        normal faces the camera); C no normal (identity) -> the default approach, else refused;
//        D a zero / NaN / non-unit quaternion is refused, never read as "no normal"; E the standoff;
//        F eef and site; G the frame and the stamp; H the id and the provider; I determinism;
//        J a detection with no hypothesis
//   R  the orientation of align_approach (owner decision M3.6-1): the minimal rotation, the
//      reference's residual rotation kept, the antiparallel refusal
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "trainit_motion_runtime/grasp_candidates.hpp"

namespace
{
using trainit::GraspCandidateArrayMsg;
using trainit::GraspCandidateMsg;
using trainit::ProjectionConfig;

constexpr double kEps = 1e-12;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// a usable synthetic candidate (frame, id, provider, unit approach)
GraspCandidateMsg good(const std::string& id = "c0")
{
  GraspCandidateMsg c;
  c.header.frame_id = "world";
  c.id = id;
  c.provider = "synthetic";
  c.grasp_pose.position.x = 0.5;
  c.grasp_pose.orientation.w = 1.0;
  c.approach.z = -1.0;
  c.standoff_m = 0.05;
  return c;
}

// the rotation taking +Z onto n -- how the perception contract encodes a fitted normal
Eigen::Quaterniond fromZ(const Eigen::Vector3d& n)
{
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n.normalized());
}

vision_msgs::msg::Detection3D detection(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
                                        const std::string& frame = "")
{
  vision_msgs::msg::Detection3D d;
  d.header.frame_id = frame;
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = "part";
  h.hypothesis.score = 0.875;
  h.pose.pose.position.x = p.x();
  h.pose.pose.position.y = p.y();
  h.pose.pose.position.z = p.z();
  h.pose.pose.orientation.x = q.x();
  h.pose.pose.orientation.y = q.y();
  h.pose.pose.orientation.z = q.z();
  h.pose.pose.orientation.w = q.w();
  d.results.push_back(h);
  return d;
}

std_msgs::msg::Header header(const std::string& frame = "cam_optical")
{
  std_msgs::msg::Header h;
  h.frame_id = frame;
  h.stamp.sec = 42;
  h.stamp.nanosec = 7;
  return h;
}

Eigen::Vector3d approachOf(const GraspCandidateMsg& c) { return {c.approach.x, c.approach.y, c.approach.z}; }

// a point 0.5 m in front of the camera (optical +Z forward), on a surface tilted toward the camera:
// its normal (0, 0.6, -0.8) faces the camera (it points back along -Z, toward the origin)
const Eigen::Vector3d kPoint(0.1, 0.0, 0.5);
const Eigen::Vector3d kNormal(0.0, 0.6, -0.8);
}  // namespace

// --- V: validation --------------------------------------------------------------------------------
TEST(M36Candidate, V_a_complete_candidate_is_usable)
{
  EXPECT_EQ(trainit::candidateProblem(good()), "");
}

TEST(M36Candidate, V_each_unusable_field_is_refused_by_name)
{
  struct Case { const char* what; void (*edit)(GraspCandidateMsg&); const char* contains; };
  const Case cases[] = {
    {"no frame", [](GraspCandidateMsg& c) { c.header.frame_id.clear(); }, "no frame"},
    {"no id", [](GraspCandidateMsg& c) { c.id.clear(); }, "no id"},
    {"no provider", [](GraspCandidateMsg& c) { c.provider.clear(); }, "no provider"},
    {"NaN position", [](GraspCandidateMsg& c) { c.grasp_pose.position.y = kNaN; }, "position is not finite"},
    {"NaN quaternion", [](GraspCandidateMsg& c) { c.grasp_pose.orientation.x = kNaN; }, "orientation is not finite"},
    {"zero quaternion", [](GraspCandidateMsg& c) { c.grasp_pose.orientation.w = 0.0; }, "not a unit quaternion"},
    {"zero approach", [](GraspCandidateMsg& c) { c.approach.z = 0.0; }, "approach is not a unit vector"},
    {"long approach", [](GraspCandidateMsg& c) { c.approach.z = -2.0; }, "approach is not a unit vector"},
    {"NaN approach", [](GraspCandidateMsg& c) { c.approach.x = kNaN; }, "approach is not finite"},
    {"negative standoff", [](GraspCandidateMsg& c) { c.standoff_m = -0.01; }, "standoff"},
    {"NaN standoff", [](GraspCandidateMsg& c) { c.standoff_m = kNaN; }, "standoff"},
    {"NaN score", [](GraspCandidateMsg& c) { c.score = kNaN; }, "score"},
    {"NaN normal", [](GraspCandidateMsg& c) { c.surface_normal.z = kNaN; }, "surface normal"},
    {"confidence above 1", [](GraspCandidateMsg& c) { c.confidence = 1.5; }, "is not in 0..1"},
    {"negative confidence", [](GraspCandidateMsg& c) { c.confidence = -0.1; }, "is not in 0..1"},
  };
  for (const auto& k : cases)
  {
    GraspCandidateMsg c = good();
    k.edit(c);
    const std::string why = trainit::candidateProblem(c);
    EXPECT_NE(why.find(k.contains), std::string::npos) << k.what << ": " << why;
  }
  // within the unit tolerance (1e-3) an approach is accepted as given, never "repaired" beyond it
  GraspCandidateMsg c = good();
  c.approach.z = -1.0005;
  EXPECT_EQ(trainit::candidateProblem(c), "");
}

TEST(M36Candidate, V_an_array_is_usable_only_whole_and_its_ids_are_unique)
{
  GraspCandidateArrayMsg a;
  EXPECT_EQ(trainit::arrayProblem(a), "");   // empty: "no candidate", a valid answer
  a.candidates = {good("a"), good("b")};
  EXPECT_EQ(trainit::arrayProblem(a), "");
  a.candidates.push_back(good("a"));
  EXPECT_NE(trainit::arrayProblem(a).find("candidate 2 ('a'): its id is not unique"), std::string::npos);
  a.candidates.back().id = "c";
  a.candidates[1].approach.z = 0.0;
  EXPECT_NE(trainit::arrayProblem(a).find("candidate 1 ('b')"), std::string::npos);
}

// --- P: the projection ------------------------------------------------------------------------------
TEST(M36Projection, A_B_a_fitted_normal_toward_the_camera_gives_an_approach_into_the_surface)
{
  ProjectionConfig cfg;
  const auto p = trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header(), 0, cfg);
  ASSERT_EQ(p.problem, "");
  EXPECT_TRUE(p.fitted_normal);
  const Eigen::Vector3d n = kNormal.normalized();
  const Eigen::Vector3d a = approachOf(p.candidate);
  // the normal is read back exactly, and the approach is its opposite: (0, -0.6, 0.8)
  EXPECT_NEAR(p.candidate.surface_normal.x, n.x(), kEps);
  EXPECT_NEAR(p.candidate.surface_normal.y, n.y(), kEps);
  EXPECT_NEAR(p.candidate.surface_normal.z, n.z(), kEps);
  EXPECT_NEAR(a.x(), 0.0, kEps);
  EXPECT_NEAR(a.y(), -0.6, kEps);
  EXPECT_NEAR(a.z(), 0.8, kEps);
  EXPECT_NEAR(a.norm(), 1.0, kEps);
  // the hemisphere: the normal faces the camera (at the optical origin), the approach points away
  // from it -- the tool travels INTO the surface
  const Eigen::Vector3d to_camera = -kPoint;
  EXPECT_GT(n.dot(to_camera), 0.0);
  EXPECT_LT(a.dot(to_camera), 0.0);
  EXPECT_NEAR(a.dot(n), -1.0, kEps);
}

TEST(M36Projection, B_the_sign_holds_for_every_tilt_and_the_frontal_case)
{
  for (const Eigen::Vector3d& n : {Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0.3, -0.2, -0.9),
                                  Eigen::Vector3d(-0.7, 0.1, -0.4)})
  {
    const auto p = trainit::projectDetection(detection(kPoint, fromZ(n)), header(), 0, {});
    ASSERT_EQ(p.problem, "") << n.transpose();
    EXPECT_NEAR((approachOf(p.candidate) + n.normalized()).norm(), 0.0, 1e-12) << n.transpose();
  }
}

TEST(M36Projection, C_no_normal_takes_the_configured_default_approach_else_it_is_refused)
{
  ProjectionConfig cfg;
  const auto none = trainit::projectDetection(detection(kPoint, Eigen::Quaterniond::Identity()), header(), 0, cfg);
  EXPECT_NE(none.problem.find("no fitted surface normal"), std::string::npos) << none.problem;
  cfg.default_approach = Eigen::Vector3d(0.0, 0.0, 1.0);   // straight ahead, in the detection's frame
  const auto p = trainit::projectDetection(detection(kPoint, Eigen::Quaterniond::Identity()), header(), 0, cfg);
  ASSERT_EQ(p.problem, "");
  EXPECT_FALSE(p.fitted_normal);
  EXPECT_NEAR((approachOf(p.candidate) - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, kEps);
  EXPECT_EQ(p.candidate.surface_normal.x, 0.0);   // zero = no normal
  EXPECT_EQ(p.candidate.surface_normal.y, 0.0);
  EXPECT_EQ(p.candidate.surface_normal.z, 0.0);
  // -identity is the identity rotation too
  const auto minus = trainit::projectDetection(detection(kPoint, Eigen::Quaterniond(-1, 0, 0, 0)), header(), 0, cfg);
  EXPECT_EQ(minus.problem, "");
  EXPECT_FALSE(minus.fitted_normal);
  // a default approach must itself be a unit vector
  cfg.default_approach = Eigen::Vector3d(0.0, 0.0, 0.0);
  EXPECT_NE(trainit::projectDetection(detection(kPoint, Eigen::Quaterniond::Identity()), header(), 0, cfg)
              .problem.find("default approach is not a unit vector"), std::string::npos);
}

TEST(M36Projection, D_a_zero_NaN_or_non_unit_orientation_is_refused_never_read_as_no_normal)
{
  ProjectionConfig cfg;
  cfg.default_approach = Eigen::Vector3d(0.0, 0.0, 1.0);   // so "no normal" WOULD succeed
  for (const auto& q : {Eigen::Quaterniond(0, 0, 0, 0), Eigen::Quaterniond(kNaN, 0, 0, 0),
                        Eigen::Quaterniond(2, 0, 0, 0), Eigen::Quaterniond(0.5, 0.5, 0, 0)})
  {
    const auto p = trainit::projectDetection(detection(kPoint, q), header(), 0, cfg);
    EXPECT_NE(p.problem.find("surface normal cannot be read"), std::string::npos) << q.coeffs().transpose()
                                                                                    << ": " << p.problem;
  }
  auto bad = detection(Eigen::Vector3d(kNaN, 0, 0), fromZ(kNormal));
  EXPECT_NE(trainit::projectDetection(bad, header(), 0, cfg).problem.find("position is not finite"), std::string::npos);
}

TEST(M36Projection, E_F_H_the_standoff_eef_site_id_and_provider)
{
  ProjectionConfig cfg;
  cfg.standoff_m = 0.04;
  cfg.eef = "main";
  cfg.site = "cup";
  cfg.detector = "bin_cam";
  const auto p = trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header(), 3, cfg);
  ASSERT_EQ(p.problem, "");
  EXPECT_EQ(p.candidate.standoff_m, 0.04);
  EXPECT_EQ(p.candidate.eef, "main");
  EXPECT_EQ(p.candidate.site, "cup");
  EXPECT_EQ(p.candidate.id, "detection_3");                        // its place in the published array
  EXPECT_EQ(p.candidate.provider, "tmr.candidates_from_detections");
  EXPECT_EQ(p.candidate.score, 0.875);
  EXPECT_EQ(p.candidate.confidence, 0.0);   // the projection states no confidence of its own
  EXPECT_EQ(p.candidate.object_class, "part");
  ASSERT_EQ(p.candidate.metadata.size(), 3u);
  EXPECT_EQ(p.candidate.metadata[1].key, "detector");
  EXPECT_EQ(p.candidate.metadata[1].value, "bin_cam");
  // a negative standoff is the projection's configuration error, refused like a producer's
  cfg.standoff_m = -1.0;
  EXPECT_NE(trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header(), 0, cfg).problem.find("standoff"),
            std::string::npos);
}

TEST(M36Projection, G_the_frame_and_stamp_are_kept_and_no_rotation_is_invented)
{
  const auto own = trainit::projectDetection(detection(kPoint, fromZ(kNormal), "cam_b_optical"), header(), 0, {});
  EXPECT_EQ(own.candidate.header.frame_id, "cam_b_optical");   // the detection's own frame wins
  const auto arr = trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header("cam_optical"), 0, {});
  EXPECT_EQ(arr.candidate.header.frame_id, "cam_optical");     // else the array's
  EXPECT_EQ(arr.candidate.header.stamp.sec, 42);
  EXPECT_EQ(arr.candidate.header.stamp.nanosec, 7u);
  EXPECT_EQ(arr.candidate.grasp_pose.position.x, 0.1);
  EXPECT_EQ(arr.candidate.grasp_pose.position.z, 0.5);
  // the projection proposes a point and an approach: its orientation is the identity, never a yaw
  EXPECT_EQ(arr.candidate.grasp_pose.orientation.w, 1.0);
  EXPECT_EQ(arr.candidate.grasp_pose.orientation.x, 0.0);
  EXPECT_EQ(arr.candidate.grasp_pose.orientation.y, 0.0);
  EXPECT_EQ(arr.candidate.grasp_pose.orientation.z, 0.0);
}

TEST(M36Projection, I_J_deterministic_and_a_detection_without_a_hypothesis_is_refused)
{
  const auto a = trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header(), 1, {});
  const auto b = trainit::projectDetection(detection(kPoint, fromZ(kNormal)), header(), 1, {});
  EXPECT_TRUE(a.candidate == b.candidate);
  vision_msgs::msg::Detection3D empty;
  EXPECT_NE(trainit::projectDetection(empty, header(), 0, {}).problem.find("no hypothesis"), std::string::npos);
}

// --- R: align_approach ------------------------------------------------------------------------------
TEST(M36AlignApproach, R_the_minimal_rotation_and_its_refusal)
{
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(trainit::minimalRotation(z, z)->isApprox(Eigen::Quaterniond::Identity()));
  const auto q = trainit::minimalRotation(z, Eigen::Vector3d::UnitX());
  ASSERT_TRUE(q);
  EXPECT_NEAR((*q * z - Eigen::Vector3d::UnitX()).norm(), 0.0, kEps);
  EXPECT_NEAR(Eigen::AngleAxisd(*q).angle(), M_PI / 2, kEps);
  EXPECT_NEAR((Eigen::AngleAxisd(*q).axis() - Eigen::Vector3d::UnitY()).norm(), 0.0, kEps);   // z x x
  EXPECT_FALSE(trainit::minimalRotation(z, -z));   // antiparallel: the axis is undefined
}

TEST(M36AlignApproach, R_a_reference_already_along_the_approach_is_kept_exactly)
{
  // the tool pointing down (180 deg about X): its +Z is -Z, the approach of a top-down pick
  const Eigen::Quaterniond down(0, 1, 0, 0);
  const auto r = trainit::alignApproach(down, Eigen::Vector3d(0, 0, -1));
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->isApprox(down, 1e-12));
}

TEST(M36AlignApproach, R_a_tilted_approach_tilts_the_reference_minimally_and_keeps_its_residual_rotation)
{
  const Eigen::Quaterniond down(0, 1, 0, 0);
  const double tilt = 30.0 * M_PI / 180.0;
  const Eigen::Vector3d a(std::sin(tilt), 0.0, -std::cos(tilt));   // 30 deg from straight down, toward +X
  const auto r = trainit::alignApproach(down, a);
  ASSERT_TRUE(r);
  EXPECT_NEAR((*r * Eigen::Vector3d::UnitZ() - a).norm(), 0.0, kEps);         // the tool's +Z travels along a
  EXPECT_NEAR(r->angularDistance(down), tilt, 1e-12);                        // and nothing more is rotated
  // the residual rotation is the reference's: a reference turned by psi about its own +Z gives the
  // result turned by psi about the approach
  const double psi = 0.7;
  const Eigen::Quaterniond turned = down * Eigen::Quaterniond(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
  const auto rt = trainit::alignApproach(turned, a);
  ASSERT_TRUE(rt);
  EXPECT_TRUE(rt->isApprox(*r * Eigen::Quaterniond(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())), 1e-12));
}

TEST(M36AlignApproach, R_a_reference_whose_tool_axis_opposes_the_approach_is_refused)
{
  const Eigen::Quaterniond down(0, 1, 0, 0);
  EXPECT_FALSE(trainit::alignApproach(down, Eigen::Vector3d(0, 0, 1)));          // exactly opposite
  EXPECT_FALSE(trainit::alignApproach(down, Eigen::Vector3d(1e-9, 0, 1).normalized()));
  EXPECT_TRUE(trainit::alignApproach(down, Eigen::Vector3d(1e-3, 0, 1).normalized()));   // defined, if large
}
