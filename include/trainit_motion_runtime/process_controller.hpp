// =============================================================================
// process_controller.hpp — abstraction for process / tool I/O (Section 10).
//
// Activates/deactivates a generic process or tool output without binding to a
// specific digital output or PLC. The meaning of a channel is defined by the
// application and its hardware bridge, not by the framework.
//
// LIMITATION (documented): in this iteration events are synchronized at SEGMENT
// BOUNDARIES, not at an exact TCP position/time. Precise position/time
// synchronization is a future capability of the robot adapter or the PLC.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PROCESS_CONTROLLER_HPP_
#define TRAINIT_MOTION_RUNTIME__PROCESS_CONTROLLER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

// vNext M3.5 (CELL_SCENE_IMPORT_REVIEW decision 4): a process channel is a LOGICAL id of the
// application -- dispenser, washer_start, inspection_light -- a letter, then letters, digits or '_'.
// Never a ROS topic path, a PLC address or a controller symbol: the controller binds it (this
// file's RosProcessController to "<channel>/process_*", a future PLC binding to its I/O).
inline bool isLogicalChannel(const std::string& c)
{
  if (c.empty() || !((c[0] >= 'a' && c[0] <= 'z') || (c[0] >= 'A' && c[0] <= 'Z'))) return false;
  for (const char ch : c)
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_'))
      return false;
  return true;
}

class ProcessController
{
public:
  virtual ~ProcessController() = default;
  // Returns true if the event was dispatched successfully.
  virtual bool executeEvent(const ProcessEvent& event) = 0;
  // vNext M3.5: make a channel ready BEFORE its first event (the runner calls it for the channels
  // its tree names), so a driver that subscribed early has matched by then. `analog`: the channel's
  // value signal (SET_ANALOG_VALUE) rather than its on/off signal. A controller with nothing to
  // prepare ignores it.
  virtual void prepare(const std::string& /*channel*/, bool /*analog*/) {}
};

// Logs the event only — for simulation / dry runs.
class MockProcessController : public ProcessController
{
public:
  explicit MockProcessController(rclcpp::Logger logger);
  bool executeEvent(const ProcessEvent& event) override;

private:
  rclcpp::Logger logger_;
};

// Publishes a latched std_msgs/Bool on "<channel>/process_enabled" (enable/
// disable/trigger) or std_msgs/Float64 on "<channel>/process_value" (analog).
// Any concrete process driver / PLC bridge subscribes to those topics.
//
// vNext M3.5: ONE publisher per channel and signal, created before its first use (prepare(), or the
// first event) and kept for the controller's lifetime, so the latched (transient_local, depth 1) last
// command reaches a driver that subscribes later, and repeated events reuse it. Until 2.3.0 a
// publisher was created per event and destroyed at once: a driver that had not matched yet missed the
// command. The topic names and the QoS are unchanged; a relative channel resolves in the node's
// namespace (ADR-0023).
class RosProcessController : public ProcessController
{
public:
  explicit RosProcessController(rclcpp::Node::SharedPtr node);
  bool executeEvent(const ProcessEvent& event) override;
  void prepare(const std::string& channel, bool analog) override;

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabledPublisher(const std::string& channel);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr valuePublisher(const std::string& channel);

  rclcpp::Node::SharedPtr node_;
  std::mutex m_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> enabled_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> value_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PROCESS_CONTROLLER_HPP_
