// =============================================================================
// moveit_runtime_probe.hpp -- the RobotTaskServer's runtime probe on a MoveIt deployment (P0.a, ADR-0032 Decision 5
// "ready"). ready = this robot's MoveIt action servers answer AND every canonical end-effector device's state arrived
// within `max_age` (the adapters publish a 10 Hz heartbeat, ADR-0022). The reason it gives is backend-neutral (it
// reaches the requester in RobotTaskStatus.detail, Decision 11); the MoveIt names go to the log, once per change.
// A MoveIt-specific probe behind the RuntimeProbe interface: test_m2_5_seam_scan classifies it B (it names MoveIt's
// action servers), like the runners' MoveIt start-up wait.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOVEIT_RUNTIME_PROBE_HPP_
#define TRAINIT_MOTION_RUNTIME__MOVEIT_RUNTIME_PROBE_HPP_

#include <chrono>
#include <string>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/robot_task_server.hpp"

namespace trainit::rti
{

class MoveItRuntimeProbe : public RuntimeProbe
{
public:
  MoveItRuntimeProbe(const rclcpp::Node::SharedPtr& node, const EndEffectorRegistry* eefs,
                     std::chrono::milliseconds max_age);
  bool ready(std::string& why) override;

private:
  void note(const std::string& specific);

  const EndEffectorRegistry* eefs_;
  std::chrono::milliseconds max_age_;
  rclcpp::Logger log_;
  std::string last_specific_;
  std::string move_name_, exec_name_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr exec_;
};

}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__MOVEIT_RUNTIME_PROBE_HPP_
