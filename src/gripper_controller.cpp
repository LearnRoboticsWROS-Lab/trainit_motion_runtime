// =============================================================================
// gripper_controller.cpp
// =============================================================================
#include "trainit_motion_runtime/gripper_controller.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace trainit
{

GripperController::GripperController(rclcpp::Node::SharedPtr node, std::string action_name)
: node_(std::move(node)), action_name_(std::move(action_name))
{
  client_ = rclcpp_action::create_client<GripperCommand>(node_, action_name_);
}

bool GripperController::waitForServer(double timeout_s)
{
  return client_->wait_for_action_server(
    std::chrono::duration<double>(timeout_s));
}

bool GripperController::command(double position, double timeout_s)
{
  if (!client_->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "gripper action server '%s' not available",
                 action_name_.c_str());
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 0.0;

  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::duration<double>(5.0)) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "gripper goal send timed out");
    return false;
  }
  auto handle = goal_future.get();
  if (!handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "gripper goal rejected");
    return false;
  }

  auto result_future = client_->async_get_result(handle);
  if (result_future.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "gripper result timed out");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "gripper command position=%.2f done", position);
  return true;
}

}  // namespace trainit
