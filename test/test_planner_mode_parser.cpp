// Unit tests: launch-parameter parsing (planner_mode / process_path_backend).
#include <gtest/gtest.h>
#include "trainit_motion_runtime/motion_types.hpp"

using namespace trainit;

TEST(PlannerModeParser, ValidModes)
{
  EXPECT_EQ(plannerModeFromString("pilz").value(), PlannerMode::PILZ);
  EXPECT_EQ(plannerModeFromString("ompl").value(), PlannerMode::OMPL);
  EXPECT_EQ(plannerModeFromString("ompl_chomp").value(), PlannerMode::OMPL_CHOMP);
  EXPECT_EQ(plannerModeFromString("stomp").value(), PlannerMode::STOMP);
}

TEST(PlannerModeParser, InvalidMode)
{
  EXPECT_FALSE(plannerModeFromString("bogus").has_value());
  EXPECT_FALSE(plannerModeFromString("").has_value());
}

TEST(PlannerModeParser, RoundTrip)
{
  EXPECT_EQ(toString(PlannerMode::PILZ), "pilz");
  EXPECT_EQ(toString(PlannerMode::OMPL_CHOMP), "ompl_chomp");
  EXPECT_EQ(toString(PlannerMode::STOMP), "stomp");
}

TEST(ProcessBackendParser, Valid)
{
  EXPECT_EQ(processPathBackendFromString("pilz_sequence").value(), ProcessPathBackend::PILZ_SEQUENCE);
  EXPECT_EQ(processPathBackendFromString("cartesian_waypoints").value(),
            ProcessPathBackend::CARTESIAN_WAYPOINTS);
  EXPECT_FALSE(processPathBackendFromString("nope").has_value());
  EXPECT_EQ(toString(ProcessPathBackend::CARTESIAN_WAYPOINTS), "cartesian_waypoints");
}
