// The candidate LIST a detection array offers (ADR-0016 Decision 2), and the two
// properties that matter most about it:
//
//   * keep = 1 is the FIRST published match, on the FLAT keys. It used to be the single
//     HIGHEST-SCORING match: a re-rank by score inside the runtime that overrode the
//     detector's ranking for every single-candidate application (vNext M0C #4). The flat
//     key LAYOUT is unchanged, so every bundle generated before ADR-0016 still reads it.
//   * keep > 1 is the PUBLISHED order, and nothing else. ADR-0016 Decision 1 puts the
//     ranking in the detector's publication order, so a test that passes only because
//     the array happens to be sorted would prove nothing: every array below is
//     deliberately out of score order.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include "trainit_motion_runtime/detection_candidates.hpp"

using trainit::clampDetectionKeep;
using trainit::detectionCandidatePrefix;
using trainit::detectionCountKey;
using trainit::detectionIndexedPrefix;
using trainit::selectDetectionCandidates;

namespace
{
// one detection, with a class and a score (the two terms the filters read)
vision_msgs::msg::Detection3D det(const std::string& class_id, double score)
{
  vision_msgs::msg::Detection3D d;
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = class_id;
  h.hypothesis.score = score;
  d.results.push_back(h);
  return d;
}

// An array a RANKING detector would publish: best first, which here is NOT the
// highest score — the point of ADR-0014 is that the rank is not the score.
vision_msgs::msg::Detection3DArray ranked()
{
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.push_back(det("capsule", 0.40));   // 0: the detector's best
  msg.detections.push_back(det("capsule", 0.90));   // 1: higher score, ranked lower
  msg.detections.push_back(det("capsule", 0.70));   // 2
  return msg;
}
}  // namespace

TEST(DetectionCandidates, KeepOneIsTheFirstPublishedNotTheHighestScore)
{
  const auto msg = ranked();
  const auto out = selectDetectionCandidates(msg, "", 0.0, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], 0u);        // the detector's best (0.40), not the 0.90 it ranked lower
}

TEST(DetectionCandidates, KeepOneBreaksTiesOnTheFIRSTPublished)
{
  // equal scores change nothing: the first published is the first published
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.push_back(det("capsule", 0.80));
  msg.detections.push_back(det("capsule", 0.80));
  const auto out = selectDetectionCandidates(msg, "", 0.0, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], 0u);
}

TEST(DetectionCandidates, KeepManyPreservesThePublishedOrderAndDoesNotSort)
{
  const auto msg = ranked();
  const auto out = selectDetectionCandidates(msg, "", 0.0, 3);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 0u);        // 0.40 first, because the DETECTOR put it first
  EXPECT_EQ(out[1], 1u);
  EXPECT_EQ(out[2], 2u);
  // the whole claim in one line: candidate 0 is not the best score
  EXPECT_LT(msg.detections[out[0]].results.front().hypothesis.score,
            msg.detections[out[1]].results.front().hypothesis.score);
}

TEST(DetectionCandidates, KeepTruncatesFromTheFrontAndNeverPadsPastWhatArrived)
{
  const auto msg = ranked();
  const auto two = selectDetectionCandidates(msg, "", 0.0, 2);
  ASSERT_EQ(two.size(), 2u);
  EXPECT_EQ(two[0], 0u);
  EXPECT_EQ(two[1], 1u);
  // keep=5 over 3 detections is 3 candidates, not 5: a retry at index 3 must fail
  // rather than find a stale pose
  EXPECT_EQ(selectDetectionCandidates(msg, "", 0.0, 5).size(), 3u);
}

TEST(DetectionCandidates, ClassAndScoreFiltersApplyToBothPaths)
{
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.push_back(det("bottle", 0.99));          // wrong class
  msg.detections.push_back(det("capsule", 0.10));         // below min_score
  msg.detections.push_back(det("capsule", 0.60));
  msg.detections.push_back(det("capsule", 0.50));
  vision_msgs::msg::Detection3D empty;                     // no hypothesis at all
  msg.detections.push_back(empty);

  const auto many = selectDetectionCandidates(msg, "capsule", 0.2, 4);
  ASSERT_EQ(many.size(), 2u);
  EXPECT_EQ(many[0], 2u);
  EXPECT_EQ(many[1], 3u);

  const auto one = selectDetectionCandidates(msg, "capsule", 0.2, 1);
  ASSERT_EQ(one.size(), 1u);
  EXPECT_EQ(one[0], 2u);
}

TEST(DetectionCandidates, NothingMatchesIsAnEmptyList)
{
  vision_msgs::msg::Detection3DArray msg;
  EXPECT_TRUE(selectDetectionCandidates(msg, "", 0.0, 1).empty());
  EXPECT_TRUE(selectDetectionCandidates(msg, "", 0.0, 4).empty());
  msg.detections.push_back(det("bottle", 0.9));
  EXPECT_TRUE(selectDetectionCandidates(msg, "capsule", 0.0, 1).empty());
  EXPECT_TRUE(selectDetectionCandidates(msg, "capsule", 0.0, 4).empty());
}

TEST(DetectionCandidates, KeepBelowOneMeansTheSingleBest)
{
  EXPECT_EQ(clampDetectionKeep(0), 1);
  EXPECT_EQ(clampDetectionKeep(-7), 1);
  EXPECT_EQ(clampDetectionKeep(1), 1);
  EXPECT_EQ(clampDetectionKeep(3), 3);
  const auto msg = ranked();
  const auto out = selectDetectionCandidates(msg, "", 0.0, 0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], 0u);        // the single best is the detector's best: candidate 0
}

// The blackboard layout is the CONTRACT between the two nodes (DetectObject writes
// it, SetWaypointFromDetection reads it) and the thing a generated bundle depends on.
TEST(DetectionCandidates, KeepOneKeepsTheFlatKeysAndKeepManyIndexesThem)
{
  EXPECT_EQ(detectionCandidatePrefix("detected.capsule", 1, 0), "detected.capsule");
  EXPECT_EQ(detectionCandidatePrefix("detected.capsule", 0, 0), "detected.capsule");
  EXPECT_EQ(detectionCandidatePrefix("detected.capsule", 3, 0), "detected.capsule.0");
  EXPECT_EQ(detectionCandidatePrefix("detected.capsule", 3, 2), "detected.capsule.2");
  EXPECT_EQ(detectionCountKey("detected.capsule"), "detected.capsule.count");
  // the consumer's name for candidate i is the producer's name for it
  EXPECT_EQ(detectionIndexedPrefix("detected.capsule", 2),
            detectionCandidatePrefix("detected.capsule", 3, 2));
}
