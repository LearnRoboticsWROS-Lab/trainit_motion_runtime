// =============================================================================
// process_controller.cpp — generic process / tool I/O.
//
// LIMITATION (documented): events fire at SEGMENT BOUNDARIES, not at an exact
// TCP position/time. Precise sync is a future robot-adapter / PLC capability.
// =============================================================================
#include "trainit_motion_runtime/process_controller.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace trainit
{

namespace
{
const char* toStr(ProcessEventType t)
{
  switch (t)
  {
    case ProcessEventType::ENABLE_PROCESS:  return "ENABLE";
    case ProcessEventType::DISABLE_PROCESS: return "DISABLE";
    case ProcessEventType::TRIGGER:         return "TRIGGER";
    case ProcessEventType::SET_ANALOG_VALUE:return "SET_ANALOG";
  }
  return "?";
}
}  // namespace

// --- mock --------------------------------------------------------------------
MockProcessController::MockProcessController(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

bool MockProcessController::executeEvent(const ProcessEvent& event)
{
  RCLCPP_INFO(logger_, "[mock process] %s channel='%s' value=%.3f",
              toStr(event.type), event.channel.c_str(), event.value);
  return true;
}

// --- ROS publisher -----------------------------------------------------------
RosProcessController::RosProcessController(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
}

bool RosProcessController::executeEvent(const ProcessEvent& event)
{
  // Latched so a late-joining process driver still receives the last command.
  const rclcpp::QoS qos = rclcpp::QoS(1).transient_local();

  if (event.type == ProcessEventType::SET_ANALOG_VALUE)
  {
    auto pub = node_->create_publisher<std_msgs::msg::Float64>(event.channel + "/process_value", qos);
    std_msgs::msg::Float64 m;
    m.data = event.value;
    pub->publish(m);
  }
  else
  {
    auto pub = node_->create_publisher<std_msgs::msg::Bool>(event.channel + "/process_enabled", qos);
    std_msgs::msg::Bool m;
    m.data = (event.type == ProcessEventType::ENABLE_PROCESS ||
              event.type == ProcessEventType::TRIGGER);
    pub->publish(m);
  }
  RCLCPP_INFO(node_->get_logger(), "[process] %s channel='%s' value=%.3f",
              toStr(event.type), event.channel.c_str(), event.value);
  return true;
}

}  // namespace trainit
