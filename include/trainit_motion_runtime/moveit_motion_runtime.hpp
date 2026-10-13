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
#include <optional>
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

namespace eef
{
class Contract;   // end_effector_contract.hpp
}

class MoveItMotionRuntime : public MotionRuntime
{
public:
  struct Config
  {
    std::string group{"manipulator"};   // generic default; the application overrides it
    std::string base_frame{"base_link"};
    std::string tip_link{"tcp"};
    PlannerMode mode{kDefaultPlannerMode};
    ProcessPathBackend process_backend{ProcessPathBackend::PILZ_SEQUENCE};
    std::vector<std::string> available_pipelines{defaultAvailablePipelines()};
    bool enforce_validation{true};   // abort execution if validation fails
    int  plan_retries{3};            // extra re-plans on stochastic OMPL failure (RRTConnect
                                     // + "invalid after postprocessing") before giving up
    double max_joint_jump{0.5};      // [rad] between consecutive waypoints
    double cartesian_fraction_threshold{1.0};   // accept cartesian path only if fraction >= this
    bool   allow_partial_cartesian{false};      // execute a partial cartesian path
  };

  // The move_group this runtime is a client of is the one in the node's OWN namespace (vNext M2.7,
  // ADR-0023 Decision 3): /r3/move_action for a node in /r3, /move_action at the root.
  MoveItMotionRuntime(rclcpp::Node::SharedPtr node, Config config);

  // --- the runners' MoveIt start-up wait (vNext M2.7, owner decision D1, ADR-0023 Decision 5) -----
  // The constructor above blocks on four MoveIt endpoints of the node's namespace: the move_group
  // actions move_action and execute_trajectory (MoveGroupInterface: an unbounded, silent wait) and
  // the services get_planning_scene and apply_planning_scene (PlanningSceneInterface: 5 s, then an
  // unbounded wait). trainit_run_bt and trainit_run_task call waitForMoveIt() before it, so a
  // missing or misplaced move_group is NAMED, and the wait is bounded.
  static constexpr const char* kMoveItWaitParam = "moveit_wait_timeout_s";
  static constexpr double kMoveItWaitDefaultS = 60.0;   // the default bound
  static constexpr double kMoveItWaitForever = -1.0;    // the legacy unbounded wait (still WARNs)
  static constexpr double kMoveItWarnPeriodS = 5.0;
  enum class MoveItWait { READY, TIMED_OUT, INTERRUPTED };
  // The runner parameter moveit_wait_timeout_s, declared with kMoveItWaitDefaultS when the project
  // does not set it. An integer or a double (60, 60.0, -1: a params file, a launch dict or -p): a
  // finite number > 0 is the bound in seconds, exactly -1 waits without a bound. Anything else (0,
  // -2, nan, inf, a string, a bool, an array) is std::nullopt, `refused` naming the value and its
  // type: a configuration error (the runner's FATAL, exit 2).
  static std::optional<double> moveItWaitTimeout(const rclcpp::Node::SharedPtr& node, std::string& refused);
  // READY at once, with no output, when the four endpoints are served. Otherwise a WARN every
  // kMoveItWarnPeriodS names those still missing, fully resolved (/r3/move_action); after
  // `timeout_s` a FATAL names them and TIMED_OUT is returned (never for kMoveItWaitForever);
  // INTERRUPTED, silently, once the context shuts down (Ctrl-C). An INFO line closes a wait that
  // WARNed. The bound runs on a steady clock (a stopped sim clock does not stop it). Its probe
  // clients are destroyed on return: nothing it creates outlives the call.
  static MoveItWait waitForMoveIt(const rclcpp::Node::SharedPtr& node, double timeout_s);
  // ADR-0023 Decision 5: under a namespace N every robot-scoped name a runner resolves must stay
  // inside N. One entry per name a remap sends elsewhere, in the list's order ("service
  // /r1/get_planning_scene -> /r2/get_planning_scene", "topic /r1/collision_object ->
  // /r2/collision_object"); empty at the root (a root robot is unchanged) and when none escapes. An
  // action counts as its five entities (Humble remaps an action's entities, never its name). The
  // runners refuse a non-empty result (FATAL, exit 2). Not checked: the cell-global /tf, /tf_static,
  // /parameter_events, /rosout and /clock; the names a tree passes in its ports; the LEGACY absolute
  // endpoints as the project writes them (ADR-0023 Decision 6).
  //
  // The 32 names BOTH runners resolve (trainit_run_task checks only these): MoveGroupInterface's
  // services, publishers and actions, the planning-scene names of the PlanningSceneInterface and the
  // PlanningSceneMonitor (its scene publisher monitored_planning_scene, which MoveIt starts from the
  // publish_planning_scene parameter every generated runner gets, and the octomap services it serves
  // included), joint_states, the Pilz sequence action and TMR's two scene-manager clients.
  static std::vector<std::string> crossNamespaceRemaps(const rclcpp::Node::SharedPtr& node);
  // ... and, after those 32, the names only trainit_run_bt resolves: ServoTrack's DEFAULT servo node
  // (servo_node/delta_twist_cmds and servo_node/status, topics; servo_node/start_servo, pause_servo
  // and unpause_servo, services), checked whether or not the tree runs a ServoTrack, then every name
  // `contract` gives its end-effector devices (eefEndpoints(); none for a LEGACY contract). A servo
  // node a tree names in ServoTrack's servo_node port is not checked.
  static std::vector<std::string> crossNamespaceRemaps(const rclcpp::Node::SharedPtr& node,
                                                       const eef::Contract& contract);

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
  // MoveGroupInterface::getCurrentPose(tip): waits up to 1 s for a fresh joint state and, when
  // none arrives, returns the identity pose (still stamped in the model frame) without reporting
  // a failure -- KNOWN_DEBT, kept as it is by vNext M2.5.
  geometry_msgs::msg::PoseStamped currentTipPose() const override;

  // --- the frames and the named-state FK: MotionRuntime methods since vNext M2.7 ---------
  const std::string& tipLink() const override { return config_.tip_link; }
  const std::string& baseFrame() const override { return config_.base_frame; }
  // FK: tip-link pose of a NAMED group state — lets a LIN/CIRC move target a
  // waypoint captured as joints (blind mode). std::nullopt if the state is unknown.
  std::optional<geometry_msgs::msg::Pose> namedPose(const std::string& named) const override;

  // --- concrete accessors for the composition roots: the runners hand scene() to BtContext::scene
  //     and to TaskRunner (vNext M2.7). The raw MoveGroupInterface is not exposed: moveGroup(),
  //     deprecated in 1.9.0, was removed in vNext M2.6 (no caller in any workspace); read the TCP
  //     with currentTipPose() ------------------------------------------------------------
  PlanningSceneManager& scene() { return *scene_; }
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
