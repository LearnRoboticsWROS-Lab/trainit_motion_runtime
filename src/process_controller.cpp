// =============================================================================
// process_controller.cpp — generic process / tool I/O.
//
// LIMITATION (documented): events fire at SEGMENT BOUNDARIES, not at an exact
// TCP position/time. Precise sync is a future robot-adapter / PLC capability.
// =============================================================================
#include "trainit_motion_runtime/process_controller.hpp"


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

// Latched so a late-joining process driver still receives the last command: the publisher is kept,
// so its one-sample history is there when the driver subscribes (vNext M3.5).
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr RosProcessController::enabledPublisher(const std::string& channel)
{
  std::lock_guard<std::mutex> lk(m_);
  auto& pub = enabled_[channel];
  if (!pub) pub = node_->create_publisher<std_msgs::msg::Bool>(channel + "/process_enabled", rclcpp::QoS(1).transient_local());
  return pub;
}

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr RosProcessController::valuePublisher(const std::string& channel)
{
  std::lock_guard<std::mutex> lk(m_);
  auto& pub = value_[channel];
  if (!pub) pub = node_->create_publisher<std_msgs::msg::Float64>(channel + "/process_value", rclcpp::QoS(1).transient_local());
  return pub;
}

void RosProcessController::prepare(const std::string& channel, bool analog)
{
  if (analog) valuePublisher(channel);
  else enabledPublisher(channel);
}

bool RosProcessController::executeEvent(const ProcessEvent& event)
{
  if (event.type == ProcessEventType::SET_ANALOG_VALUE)
  {
    std_msgs::msg::Float64 m;
    m.data = event.value;
    valuePublisher(event.channel)->publish(m);
  }
  else
  {
    std_msgs::msg::Bool m;
    m.data = (event.type == ProcessEventType::ENABLE_PROCESS ||
              event.type == ProcessEventType::TRIGGER);
    enabledPublisher(event.channel)->publish(m);
  }
  RCLCPP_INFO(node_->get_logger(), "[process] %s channel='%s' value=%.3f",
              toStr(event.type), event.channel.c_str(), event.value);
  return true;
}

}  // namespace trainit
