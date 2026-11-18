// moveit_runtime_probe.cpp -- the RobotTaskServer's runtime probe on a MoveIt deployment (P0.a). See the header.
#include "trainit_motion_runtime/moveit_runtime_probe.hpp"

#include <moveit/move_group/capability_names.h>
#include <moveit/utils/rclcpp_utils.h>

namespace trainit::rti
{

MoveItRuntimeProbe::MoveItRuntimeProbe(const rclcpp::Node::SharedPtr& node, const EndEffectorRegistry* eefs,
                                       std::chrono::milliseconds max_age)
: eefs_(eefs), max_age_(max_age), log_(node->get_logger())
{
  const std::string ns = node->get_namespace();
  move_name_ = rclcpp::names::append(ns, move_group::MOVE_ACTION);
  exec_name_ = rclcpp::names::append(ns, move_group::EXECUTE_ACTION_NAME);
  move_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, move_name_);
  exec_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(node, exec_name_);
}

void MoveItRuntimeProbe::note(const std::string& specific)
{
  if (specific == last_specific_) return;   // logged when it changes, not every period
  last_specific_ = specific;
  if (!specific.empty()) RCLCPP_WARN(log_, "runtime not ready: %s", specific.c_str());
}

bool MoveItRuntimeProbe::ready(std::string& why)
{
  // `why` goes on the wire (RobotTaskStatus.detail): backend-neutral words; the backend's names go to the log
  if (!move_->action_server_is_ready())
  {
    note("MoveIt action " + move_name_ + " not served");
    why = "the robot's motion system is not answering";
    return false;
  }
  if (!exec_->action_server_is_ready())
  {
    note("MoveIt action " + exec_name_ + " not served");
    why = "the robot's motion system is not answering";
    return false;
  }
  if (eefs_)
  {
    const auto now = std::chrono::steady_clock::now();
    for (const auto& spec : eefs_->contract().devices)
    {
      const EndEffectorDevice* dev = eefs_->device(spec.id);
      if (!dev) continue;
      const auto snap = dev->latest();
      if (!snap.sample)
      {
        why = "end effector " + spec.id + ": no state yet";
        return false;
      }
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - snap.arrival);
      if (age > max_age_)
      {
        why = "end effector " + spec.id + ": state stale (" + std::to_string(age.count()) + " ms)";
        return false;
      }
    }
  }
  note("");
  return true;
}

}  // namespace trainit::rti
