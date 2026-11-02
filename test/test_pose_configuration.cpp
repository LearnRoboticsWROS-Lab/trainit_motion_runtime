// Unit tests: which joint configuration a pose goal takes (TMR 2.6.0, ADR-0028). Pure.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "trainit_motion_runtime/pose_configuration.hpp"

using namespace trainit;

namespace
{
constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 2.0 * kPi;

// a UR-like arm: every joint +-2*pi (a wrist reaches each angle twice)
std::vector<JointBounds> wide(std::size_t n) { return std::vector<JointBounds>(n, JointBounds{-kTwoPi, kTwoPi, false}); }
}  // namespace

TEST(PoseConfiguration, ParameterTokens)
{
  EXPECT_EQ(poseGoalConfigurationFromString("nearest"), PoseGoalConfiguration::NEAREST);
  EXPECT_EQ(poseGoalConfigurationFromString("planner"), PoseGoalConfiguration::PLANNER);
  EXPECT_FALSE(poseGoalConfigurationFromString("Nearest"));
  EXPECT_FALSE(poseGoalConfigurationFromString(""));
  EXPECT_STREQ(toString(PoseGoalConfiguration::NEAREST), "nearest");
}

TEST(PoseConfiguration, AWristTurnIsTheEquivalentNearestTheReference)
{
  // the owner's Test C: the same pose, the wrist one turn away (13.4 rad of motion instead of 4.4)
  const auto q = nearestEquivalent({0.5, -1.0, 1.2, -1.5, 1.57, 0.3 + kTwoPi}, {0.4, -1.1, 1.0, -1.4, 1.5, 0.2}, wide(6));
  EXPECT_NEAR(q[5], 0.3, 1e-12);
  EXPECT_NEAR(q[0], 0.5, 1e-12);
}

TEST(PoseConfiguration, ABoundedJointKeepsAValueItCannotMove)
{
  // +-pi bounds: 3.0 rad has no other equivalent inside them, whatever the reference
  const auto q = nearestEquivalent({3.0}, {-3.0}, {JointBounds{-kPi, kPi, false}});
  EXPECT_NEAR(q[0], 3.0, 1e-12);
  // a continuous joint moves freely
  const auto c = nearestEquivalent({3.0}, {-3.0}, {JointBounds{-kPi, kPi, true}});
  EXPECT_NEAR(c[0], 3.0 - kTwoPi, 1e-12);
}

TEST(PoseConfiguration, TheSolutionNearestTheStartWinsWhateverOrderTheIkFoundThem)
{
  const std::vector<double> start{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  const std::vector<std::vector<double>> found{
    {2.9, -1.2, -1.6, 0.4, 1.5, 3.0},          // the other shoulder
    {0.1, -1.4, 1.5, -1.6, -1.57, 0.2 + kTwoPi},  // the near one, one wrist turn off
    {0.1, -1.4, 1.5, -1.6, 1.57, -2.9}};        // the flipped wrist
  const auto a = chooseConfiguration(found, start, wide(6), std::nullopt);
  ASSERT_TRUE(a.joints);
  EXPECT_NEAR((*a.joints)[5], 0.2, 1e-9);         // the turn removed
  EXPECT_NEAR((*a.joints)[4], -1.57, 1e-9);       // not the flipped wrist
  EXPECT_EQ(a.distinct, 3u);
  std::vector<std::vector<double>> reversed(found.rbegin(), found.rend());
  const auto b = chooseConfiguration(reversed, start, wide(6), std::nullopt);
  EXPECT_EQ(*a.joints, *b.joints);                // deterministic: the order IK returned them in is irrelevant
}

TEST(PoseConfiguration, OneSolutionFoundTwiceCountsOnce)
{
  const std::vector<double> start{0.0, 0.0};
  const auto c = chooseConfiguration({{0.5, 0.5}, {0.5 + kTwoPi, 0.5}, {0.50000001, 0.5}}, start, wide(2), std::nullopt);
  EXPECT_EQ(c.distinct, 1u);
}

TEST(PoseConfiguration, ATaughtConfigurationIsHonouredOrRefusedNeverReplaced)
{
  const std::vector<double> taught{0.0, -1.0, 1.0, -1.0, 1.57, 0.0};
  const std::vector<std::vector<double>> near_and_far{{0.05, -1.02, 1.01, -1.0, 1.55, 0.03}, {3.0, -2.0, -1.0, 1.0, -1.5, 3.0}};
  const auto ok = chooseConfiguration(near_and_far, taught, wide(6), kDefaultConfigurationTolerance);
  ASSERT_TRUE(ok.joints);
  EXPECT_FALSE(ok.refused);
  EXPECT_NEAR(ok.max_difference, 0.05, 1e-9);
  // only the far branch reaches the pose: refused, saying how far and on which joint
  const auto no = chooseConfiguration({{3.0, -1.0, 1.0, -1.0, -1.5, 0.0}}, taught, wide(6), kDefaultConfigurationTolerance);
  EXPECT_FALSE(no.joints);
  EXPECT_TRUE(no.refused);
  EXPECT_EQ(no.worst_joint, 4u);
  EXPECT_NEAR(no.max_difference, 3.07, 1e-9);
}

TEST(PoseConfiguration, NoSolutionIsNotARefusal)
{
  const auto c = chooseConfiguration({}, {0.0}, wide(1), kDefaultConfigurationTolerance);
  EXPECT_FALSE(c.joints);
  EXPECT_FALSE(c.refused);
  EXPECT_EQ(c.distinct, 0u);
  // a solution of the wrong size is ignored, never compared
  EXPECT_EQ(chooseConfiguration({{0.0, 0.0}}, {0.0}, wide(1), std::nullopt).distinct, 0u);
}

TEST(PoseConfiguration, APrismaticVariableIsNeverWrapped)
{
  // a linear axis in the group: 6.5 m from its reference is a distance, not a turn
  const JointBounds rail{-10.0, 10.0, false, false};
  const auto q = nearestEquivalent({6.5}, {0.0}, {rail});
  EXPECT_NEAR(q[0], 6.5, 1e-12);
}

TEST(PoseConfiguration, ANonFiniteSolutionIsNeverChosenAndANonFiniteReferenceIsCaught)
{
  const std::vector<double> ref{0.0, 0.0};
  const auto c = chooseConfiguration({{std::nan(""), 0.0}, {0.2, 0.1}}, ref, wide(2), std::nullopt);
  ASSERT_TRUE(c.joints);
  EXPECT_EQ(c.distinct, 1u);
  EXPECT_NEAR((*c.joints)[0], 0.2, 1e-12);
  EXPECT_FALSE(allFinite({0.0, std::nan("")}));
  EXPECT_FALSE(allFinite({INFINITY}));
  EXPECT_TRUE(allFinite({0.0, -1.0}));
}

TEST(PoseConfiguration, TiesAndTheToleranceBoundary)
{
  // the same largest difference: the smaller sum wins
  const auto t = chooseConfiguration({{0.5, 0.5}, {0.5, 0.1}}, {0.0, 0.0}, wide(2), std::nullopt);
  ASSERT_TRUE(t.joints);
  EXPECT_NEAR((*t.joints)[1], 0.1, 1e-12);
  // exactly at the tolerance: honoured; beyond it: refused
  EXPECT_TRUE(chooseConfiguration({{1.0}}, {0.0}, wide(1), 1.0).joints);
  EXPECT_TRUE(chooseConfiguration({{1.0 + 1e-6}}, {0.0}, wide(1), 1.0).refused);
}
