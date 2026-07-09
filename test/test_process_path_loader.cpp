// Unit tests: process path loaders (CSV / YAML).
#include <gtest/gtest.h>
#include <fstream>
#include "trainit_motion_runtime/process_path_generator.hpp"
#include "trainit_motion_runtime/task_loader.hpp"

using namespace trainit;

TEST(CsvLoader, ParsesWaypointsAndDefaults)
{
  const std::string path = "/tmp/trainit_test_path.csv";
  {
    std::ofstream f(path);
    f << "# x,y,z,vel,acc,process\n";
    f << "0.5,0.0,0.1,0.05,0.05,1\n";
    f << "0.6,0.0,0.1\n";          // vel/acc default, process off
    f << "0.6,0.1,0.1\n";
  }
  PathFrame frame;
  frame.velocity = 0.07;
  auto p = CsvProcessPathLoader::fromFile(path, frame);
  ASSERT_EQ(p.waypoints.size(), 3u);
  EXPECT_NEAR(p.waypoints[0].pose.position.x, 0.5, 1e-9);
  EXPECT_NEAR(p.waypoints[0].path_velocity, 0.05, 1e-9);
  EXPECT_TRUE(p.waypoints[0].process_enabled);
  EXPECT_NEAR(p.waypoints[1].path_velocity, 0.07, 1e-9);  // default from frame
  EXPECT_FALSE(p.waypoints[1].process_enabled);
}

TEST(YamlLoader, ParsesWaypoints)
{
  const std::string path = "/tmp/trainit_test_path.yaml";
  {
    std::ofstream f(path);
    f << "path_name: t\nreference_frame: base_link\ntcp_link: tcp\n";
    f << "orientation: [0.0, 0.0, 0.0, 1.0]\n";
    f << "closed_path: false\nwaypoints:\n";
    f << "  - {position: [0.5, 0.0, 0.1]}\n";
    f << "  - {position: [0.6, 0.0, 0.1], velocity: 0.02}\n";
  }
  auto p = YamlProcessPathLoader::fromFile(path);
  ASSERT_EQ(p.waypoints.size(), 2u);
  EXPECT_EQ(p.reference_frame, "base_link");
  EXPECT_NEAR(p.waypoints[1].path_velocity, 0.02, 1e-9);
  EXPECT_NEAR(p.waypoints[0].pose.orientation.w, 1.0, 1e-9);
}

TEST(TaskLoader, ParsesStepsAndProcessPath)
{
  const std::string path = "/tmp/trainit_test_task.yaml";
  {
    std::ofstream f(path);
    f << "name: t\ngroup: g\nbase_frame: base_link\ntip_link: tcp\n";
    f << "orientation: [0.0, 0.0, 0.0, 1.0]\n";
    f << "steps:\n";
    f << "  - {name: home, motion: ptp, named: home}\n";
    f << "  - {name: go, motion: linear, position: [0.5, 0.0, 0.1], eef: {gripper: 1.0}}\n";
    f << "  - {name: rect, motion: process_path, path: {type: rectangle, center: [0.5,0,0.05], size_x: 0.1, size_y: 0.1}}\n";
  }
  auto t = TaskLoader::fromYamlFile(path);
  ASSERT_EQ(t.steps.size(), 3u);
  EXPECT_EQ(t.steps[0].motion, StepMotion::PTP);
  EXPECT_EQ(t.steps[0].named_state, "home");
  EXPECT_EQ(t.steps[1].motion, StepMotion::LINEAR);
  EXPECT_EQ(t.steps[1].eef.type, EefActionType::GRIPPER);
  EXPECT_NEAR(t.steps[1].eef.gripper_position, 1.0, 1e-9);
  EXPECT_EQ(t.steps[2].motion, StepMotion::PROCESS_PATH);
  ASSERT_TRUE(t.steps[2].process_path.has_value());
  EXPECT_EQ(t.steps[2].process_path->source, PathSource::RECTANGLE);
}
