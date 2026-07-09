// =============================================================================
// gripper_controller.hpp — generic GripperCommand action client.
//
// Sends a GripperCommand goal (position 1.0 close/on .. 0.0 open/off) to a
// controller action. Robot- and application-agnostic: the action name is a
// runtime parameter provided by the application bring-up; whatever physical
// physical effect it has lives entirely in the controller / hardware bridge.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__GRIPPER_CONTROLLER_HPP_
#define TRAINIT_MOTION_RUNTIME__GRIPPER_CONTROLLER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

namespace trainit
{

class GripperController
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;

  GripperController(rclcpp::Node::SharedPtr node, std::string action_name);

  bool waitForServer(double timeout_s = 5.0);

  // position: 1.0 (close/ON) .. 0.0 (open/OFF). Blocks until the result.
  bool command(double position, double timeout_s = 10.0);

  bool open(double timeout_s = 10.0)  { return command(0.0, timeout_s); }
  bool close(double timeout_s = 10.0) { return command(1.0, timeout_s); }

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<GripperCommand>::SharedPtr client_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__GRIPPER_CONTROLLER_HPP_
