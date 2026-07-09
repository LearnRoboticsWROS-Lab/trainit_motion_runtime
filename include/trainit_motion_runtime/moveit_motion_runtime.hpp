// =============================================================================
// moveit_motion_runtime.hpp — MoveIt-backed implementation of MotionRuntime.
//
// Owns the MoveGroupInterface, the strategy selector, the planning-scene manager
// and the trajectory validator. All MoveIt details (setPlanningPipelineId,
// setPlannerId, computeCartesianPath, sequence action) are confined here and in
// the process-path backends — the application never sees them.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOVEIT_MOTION_RUNTIME_HPP_
#define TRAINIT_MOTION_RUNTIME__MOVEIT_MOTION_RUNTIME_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "trainit_motion_runtime/motion_runtime.hpp"
#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"
#include "trainit_motion_runtime/planning_scene_manager.hpp"
#include "trainit_motion_runtime/trajectory_validator.hpp"

namespace trainit
{

class MoveItMotionRuntime : public MotionRuntime
{
public:
  struct Config
  {
    std::string group{"manipulator"};   // generic default; the application overrides it
    std::string base_frame{"base_link"};
    std::string tip_link{"tcp"};
    PlannerMode mode{PlannerMode::PILZ};
    ProcessPathBackend process_backend{ProcessPathBackend::PILZ_SEQUENCE};
    std::vector<std::string> available_pipelines{"ompl", "pilz_industrial_motion_planner", "chomp"};
    bool enforce_validation{true};   // abort execution if validation fails
    double max_joint_jump{0.5};      // [rad] between consecutive waypoints
    double cartesian_fraction_threshold{1.0};   // accept cartesian path only if fraction >= this
    bool   allow_partial_cartesian{false};      // execute a partial cartesian path
  };

  MoveItMotionRuntime(rclcpp::Node::SharedPtr node, Config config);

  // --- MotionRuntime interface ----------------------------------------------
  MotionResult movePtp(const JointTarget& target, const MotionOptions& options) override;
  MotionResult moveLinear(const CartesianTarget& target, const MotionOptions& options) override;
  MotionResult moveCircular(const CircularTarget& target, const MotionOptions& options) override;
  MotionResult moveCollisionFree(const MotionTarget& target, const PlanningOptions& options) override;
  MotionResult executeProcessSequence(const std::vector<ProcessSegment>& segments,
                                      const MotionOptions& options) override;
  MotionResult executeCartesianProcess(const CartesianProcessPath& path,
                                       const MotionOptions& options) override;
  MotionResult executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override;
  MotionResult stop() override;
  void setPlannerMode(PlannerMode mode) override;
  PlannerMode plannerMode() const override;

  // --- accessors for application coordinators (scene / move group) -----------
  PlanningSceneManager& scene() { return *scene_; }
  moveit::planning_interface::MoveGroupInterface& moveGroup() { return *arm_; }
  const std::string& tipLink() const { return config_.tip_link; }
  const std::string& baseFrame() const { return config_.base_frame; }
  const std::string& group() const { return config_.group; }
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  // Plan with the given profile (already-set target on arm_), validate, execute.
  MotionResult planAndExecute(const PlannerProfile& profile, const MotionOptions& options,
                              MotionIntent intent, const std::string& stage);

  // Single plan attempt with an explicit pipeline/planner (target already set).
  bool doPlan(const std::string& pipeline, const std::string& planner,
              const MotionOptions& options,
              moveit::planning_interface::MoveGroupInterface::Plan& plan,
              double& planning_time, int& error_code);

  void fillFreeSpaceMetrics(FreeSpaceMetrics& m, const PlannerProfile& profile,
                            const std::string& stage,
                            const moveit_msgs::msg::RobotTrajectory& traj,
                            double planning_time, bool planning_ok) const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<ManualMotionStrategySelector> selector_;
  std::unique_ptr<PlanningSceneManager> scene_;
  std::unique_ptr<TrajectoryValidator> validator_;
  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOVEIT_MOTION_RUNTIME_HPP_
