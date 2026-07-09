// Unit tests: ManualMotionStrategySelector tables + guards (Sections 5-6, 14-15).
#include <gtest/gtest.h>
#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"

using namespace trainit;

namespace
{
const std::vector<std::string> ALL = {"ompl", "pilz_industrial_motion_planner", "chomp"};
}

// Approach/retreat (LINEAR) and CIRCULAR are ALWAYS Pilz, regardless of mode.
TEST(Selector, LinearAlwaysPilzLin)
{
  for (auto mode : {PlannerMode::PILZ, PlannerMode::OMPL, PlannerMode::OMPL_CHOMP})
  {
    ManualMotionStrategySelector s(mode, ProcessPathBackend::PILZ_SEQUENCE, ALL);
    auto p = s.selectPlanner(MotionIntent::LINEAR, "lin");
    EXPECT_EQ(p.pipeline_id, "pilz_industrial_motion_planner");
    EXPECT_EQ(p.planner_id, "LIN");
  }
}

TEST(Selector, CircularAlwaysPilzCirc)
{
  ManualMotionStrategySelector s(PlannerMode::OMPL, ProcessPathBackend::PILZ_SEQUENCE, ALL);
  auto p = s.selectPlanner(MotionIntent::CIRCULAR, "circ");
  EXPECT_EQ(p.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(p.planner_id, "CIRC");
}

TEST(Selector, PilzModeFreeSpaceIsPtp)
{
  ManualMotionStrategySelector s(PlannerMode::PILZ, ProcessPathBackend::PILZ_SEQUENCE, ALL);
  auto p = s.selectPlanner(MotionIntent::COLLISION_FREE, "t");
  EXPECT_EQ(p.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(p.planner_id, "PTP");
}

TEST(Selector, OmplModeFreeSpaceIsOmpl)
{
  ManualMotionStrategySelector s(PlannerMode::OMPL, ProcessPathBackend::PILZ_SEQUENCE, ALL);
  auto p = s.selectPlanner(MotionIntent::COLLISION_FREE, "t");
  EXPECT_EQ(p.pipeline_id, "ompl");
  EXPECT_FALSE(p.two_stage_optimize);
}

TEST(Selector, OmplChompIsTwoStage)
{
  ManualMotionStrategySelector s(PlannerMode::OMPL_CHOMP, ProcessPathBackend::PILZ_SEQUENCE, ALL);
  auto p = s.selectPlanner(MotionIntent::COLLISION_FREE, "t");
  EXPECT_EQ(p.pipeline_id, "ompl");
  EXPECT_TRUE(p.two_stage_optimize);
  EXPECT_EQ(p.optimizer_pipeline_id, "chomp");
}

// STOMP not installed -> degrade to OMPL, never emit a 'stomp' pipeline.
TEST(Selector, StompGuardDegradesToOmpl)
{
  ManualMotionStrategySelector s(PlannerMode::STOMP, ProcessPathBackend::PILZ_SEQUENCE,
                                 {"ompl", "pilz_industrial_motion_planner"});
  EXPECT_EQ(s.mode(), PlannerMode::OMPL);
  auto p = s.selectPlanner(MotionIntent::COLLISION_FREE, "t");
  EXPECT_NE(p.pipeline_id, "stomp");
}

// OMPL_CHOMP with no chomp pipeline -> degrade to OMPL.
TEST(Selector, OmplChompGuardDegradesIfNoChomp)
{
  ManualMotionStrategySelector s(PlannerMode::OMPL_CHOMP, ProcessPathBackend::PILZ_SEQUENCE,
                                 {"ompl", "pilz_industrial_motion_planner"});
  EXPECT_EQ(s.mode(), PlannerMode::OMPL);
}

TEST(Selector, ProcessBackendPassthrough)
{
  ManualMotionStrategySelector s(PlannerMode::PILZ, ProcessPathBackend::CARTESIAN_WAYPOINTS, ALL);
  CartesianProcessPath path;
  EXPECT_EQ(s.selectProcessPathBackend(path), ProcessPathBackend::CARTESIAN_WAYPOINTS);
}
