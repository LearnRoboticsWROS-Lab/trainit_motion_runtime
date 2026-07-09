// =============================================================================
// pilz_sequence_backend.hpp — process path via the Pilz MoveGroupSequence action.
//
// Chains LIN / CIRC (and PTP) segments with optional blending into ONE executed
// motion (no stop between segments). Uses the `sequence_move_group` action
// provided by the Pilz capability (MoveGroupSequenceAction). This is the robust
// industrial path backend (deterministic MoveL/MoveC), preferred over the
// KDL-based cartesian backend for production straight lines / arcs.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PILZ_SEQUENCE_BACKEND_HPP_
#define TRAINIT_MOTION_RUNTIME__PILZ_SEQUENCE_BACKEND_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include "trainit_motion_runtime/process_path_backend.hpp"
#include "trainit_motion_runtime/motion_task.hpp"

namespace trainit
{

class PilzSequenceBackend : public ProcessPathBackendBase
{
public:
  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

  PilzSequenceBackend(rclcpp::Node::SharedPtr node,
                      std::string group, std::string tip_link, std::string base_frame);

  std::string name() const override { return "pilz_sequence"; }
  bool supportsCircular() const override { return true; }

  // Plan (+execute unless options.plan_only) a sequence of LIN/CIRC segments.
  MotionResult run(const std::vector<ProcessSegment>& segments, const MotionOptions& options);

private:
  moveit_msgs::msg::MotionPlanRequest buildRequest(const ProcessSegment& seg,
                                                   const MotionOptions& options) const;

  rclcpp::Node::SharedPtr node_;
  std::string group_, tip_, base_;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr client_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PILZ_SEQUENCE_BACKEND_HPP_
