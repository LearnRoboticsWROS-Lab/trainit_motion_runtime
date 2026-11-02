// The runner parameters of the pose-goal configuration (TMR 2.6.0, ADR-0028): what both runners accept and
// what they refuse (a configuration error: their FATAL, exit 2). A parameters-only node, no MoveIt.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/moveit_motion_runtime.hpp"

using trainit::MoveItMotionRuntime;
using trainit::PoseGoalConfiguration;

namespace
{
std::shared_ptr<rclcpp::Node> nodeWith(std::vector<rclcpp::Parameter> overrides)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(overrides).automatically_declare_parameters_from_overrides(true);
  static int k = 0;
  return std::make_shared<rclcpp::Node>("pose_configuration_params_" + std::to_string(k++), o);
}

bool parse(std::vector<rclcpp::Parameter> overrides, MoveItMotionRuntime::Config& cfg, std::string& refused)
{
  return MoveItMotionRuntime::poseConfigurationParams(nodeWith(std::move(overrides)), cfg, refused);
}
}  // namespace

class PoseConfigurationParams : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PoseConfigurationParams, TheDefaultsAreNearestAndAQuarterTurn)
{
  MoveItMotionRuntime::Config cfg;
  std::string refused;
  ASSERT_TRUE(parse({}, cfg, refused)) << refused;
  EXPECT_EQ(cfg.pose_goal_configuration, PoseGoalConfiguration::NEAREST);
  EXPECT_DOUBLE_EQ(cfg.configuration_tolerance, M_PI / 2.0);
}

TEST_F(PoseConfigurationParams, PlannerAndAnIntegerToleranceAreAccepted)
{
  MoveItMotionRuntime::Config cfg;
  std::string refused;
  ASSERT_TRUE(parse({{"pose_goal_configuration", "planner"}, {"configuration_tolerance", 1}}, cfg, refused)) << refused;
  EXPECT_EQ(cfg.pose_goal_configuration, PoseGoalConfiguration::PLANNER);
  EXPECT_DOUBLE_EQ(cfg.configuration_tolerance, 1.0);
}

TEST_F(PoseConfigurationParams, AnythingElseIsRefusedNamingTheParameter)
{
  for (const auto& bad : std::vector<std::vector<rclcpp::Parameter>>{
         {{"pose_goal_configuration", "Nearest"}}, {{"pose_goal_configuration", true}},
         {{"configuration_tolerance", 0.0}}, {{"configuration_tolerance", -1.0}},
         {{"configuration_tolerance", std::nan("")}}, {{"configuration_tolerance", "wide"}}})
  {
    MoveItMotionRuntime::Config cfg;
    std::string refused;
    EXPECT_FALSE(parse(bad, cfg, refused)) << bad.front().get_name();
    EXPECT_NE(refused.find(bad.front().get_name()), std::string::npos) << refused;
  }
}
