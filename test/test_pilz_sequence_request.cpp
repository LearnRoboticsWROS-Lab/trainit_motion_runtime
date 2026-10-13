// The Pilz sequence backend's REQUEST (vNext M2.1): the path every circ move and every process
// path takes. The pipeline and planner it asks for are the planner registry's Cartesian entries
// (config/planner_profiles.yaml) -- checked here by capturing the goal a FAKE
// sequence_move_group receives. No MoveIt stack is needed; both nodes live in a private
// namespace, so a real move_group on the same ROS domain is never reached.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "trainit_motion_runtime/pilz_sequence_backend.hpp"

using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveGroupSequence>;

TEST(PilzSequenceRequest, EveryItemAsksForTheRegistrysCartesianPlanner)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto server_node = std::make_shared<rclcpp::Node>("fake_sequence_move_group", "/m21_pilz_request");
  auto client_node = std::make_shared<rclcpp::Node>("pilz_sequence_client", "/m21_pilz_request");

  std::mutex m;
  std::optional<MoveGroupSequence::Goal> received;
  auto server = rclcpp_action::create_server<MoveGroupSequence>(
    server_node, "sequence_move_group",
    [](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveGroupSequence::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [&](std::shared_ptr<GoalHandle> handle) {
      {
        std::lock_guard<std::mutex> lock(m);
        received = *handle->get_goal();
      }
      auto result = std::make_shared<MoveGroupSequence::Result>();
      result->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      handle->succeed(result);
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(server_node);
  exec.add_node(client_node);
  std::atomic<bool> stop{false};
  std::thread spinner([&] {
    while (!stop && rclcpp::ok()) exec.spin_some(std::chrono::milliseconds(10));
  });

  trainit::MotionResult result;
  {
    trainit::PilzSequenceBackend backend(client_node, "arm", "tcp", "base_link");
    trainit::ProcessSegment lin;
    lin.name = "lin";
    lin.type = trainit::ProcessSegmentType::LINEAR;
    lin.target_pose.orientation.w = 1.0;
    trainit::ProcessSegment circ;
    circ.name = "circ";
    circ.type = trainit::ProcessSegmentType::CIRCULAR;
    circ.target_pose.orientation.w = 1.0;
    circ.interim_point = geometry_msgs::msg::Point();
    trainit::MotionOptions options;
    options.plan_only = true;
    result = backend.run({lin, circ}, options);
  }
  stop = true;
  spinner.join();
  exec.remove_node(server_node);
  exec.remove_node(client_node);

  ASSERT_TRUE(result.success) << result.stage;
  std::lock_guard<std::mutex> lock(m);
  ASSERT_TRUE(received.has_value());
  const auto& items = received->request.items;
  ASSERT_EQ(items.size(), 2u);
  const YAML::Node cartesian = YAML::LoadFile(TRAINIT_PLANNER_REGISTRY)["cartesian"];
  EXPECT_EQ(items[0].req.pipeline_id, cartesian["linear"]["pipeline"].as<std::string>());
  EXPECT_EQ(items[0].req.planner_id, cartesian["linear"]["planner"].as<std::string>());
  EXPECT_EQ(items[1].req.pipeline_id, cartesian["circular"]["pipeline"].as<std::string>());
  EXPECT_EQ(items[1].req.planner_id, cartesian["circular"]["planner"].as<std::string>());
  EXPECT_TRUE(received->planning_options.plan_only);

  server.reset();
  server_node.reset();
  client_node.reset();
  rclcpp::shutdown();
}
