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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

class ProcessController
{
public:
  virtual ~ProcessController() = default;
  // Returns true if the event was dispatched successfully.
  virtual bool executeEvent(const ProcessEvent& event) = 0;
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
class RosProcessController : public ProcessController
{
public:
  explicit RosProcessController(rclcpp::Node::SharedPtr node);
  bool executeEvent(const ProcessEvent& event) override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PROCESS_CONTROLLER_HPP_
