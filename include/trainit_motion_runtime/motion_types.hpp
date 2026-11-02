// =============================================================================
// motion_types.hpp — pure data contract for the TrainIt Motion Runtime.
//
// Robot-agnostic value types (enums, targets, options, process-path data,
// metrics, results). No MoveIt runtime objects here, only message types, so the
// contract can be reused by application code, tests and (future) Behavior Tree
// nodes without pulling the planning stack.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_TYPES_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_TYPES_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace trainit
{

// --- Intent: WHAT kind of motion the application asks for (Section 4) --------
enum class MotionIntent
{
  PTP,                        // joint-space point-to-point (MoveJ)
  LINEAR,                     // cartesian straight line (MoveL)
  CIRCULAR,                   // cartesian arc (MoveC)
  COLLISION_FREE,             // free-space with collision avoidance
  COLLISION_FREE_OPTIMIZED,   // free-space with the optimizer stage (CHOMP first, OMPL fallback)
  CARTESIAN_PROCESS_PATH,     // dense cartesian process path
  ONLINE_SERVO                // online cartesian/joint servoing (future)
};

// --- Planner mode: HOW free-space motions are planned (Section 14) -----------
// The vocabulary is TMR's planner registry, config/planner_profiles.yaml (ADR-0021).
// This enum, kPlannerModes and ManualMotionStrategySelector are its MAPPING onto
// MoveIt, in code; test_planner_registry checks them against it, entry by entry.
enum class PlannerMode
{
  PILZ,
  OMPL,
  OMPL_CHOMP,
  STOMP   // declared, never loaded: MoveIt on Humble has no STOMP plugin (degrades to OMPL)
};

// Every PlannerMode, in registry order: the parser accepts exactly toString() of these.
inline constexpr std::array<PlannerMode, 4> kPlannerModes{
  PlannerMode::PILZ, PlannerMode::OMPL, PlannerMode::OMPL_CHOMP, PlannerMode::STOMP};

// The mode when nothing sets planner_mode (registry: default_planner_mode).
inline constexpr PlannerMode kDefaultPlannerMode = PlannerMode::PILZ;

// --- Process-path backend selection (Section 14) -----------------------------
enum class ProcessPathBackend
{
  PILZ_SEQUENCE,
  CARTESIAN_WAYPOINTS
};

// --- Process segment kind for the Pilz sequence backend (Section 7) ----------
enum class ProcessSegmentType
{
  LINEAR,
  CIRCULAR
};

// --- Process I/O event kind (Section 10) -------------------------------------
enum class ProcessEventType
{
  ENABLE_PROCESS,
  DISABLE_PROCESS,
  TRIGGER,
  SET_ANALOG_VALUE
};

// --- Planner profile: the concrete pipeline/planner a selector resolves to ---
struct PlannerProfile
{
  std::string pipeline_id;      // e.g. "ompl", "pilz_industrial_motion_planner", "chomp"
  std::string planner_id;       // e.g. "RRTConnectkConfigDefault", "PTP", "LIN", "CIRC"
  // True for OMPL_CHOMP, and for OMPL under intent COLLISION_FREE_OPTIMIZED (the
  // selector's upgrade; the runtime asks for that intent only in ompl_chomp mode). MoveIt
  // 2.5 ships no CHOMP optimizer adapter, so the runtime does not chain OMPL into CHOMP:
  // it tries `optimizer_pipeline_id` (chomp) FIRST, standalone, and falls back to
  // pipeline_id/planner_id (ompl) when it fails.
  bool two_stage_optimize{false};
  std::string optimizer_pipeline_id;   // e.g. "chomp"
};

// --- Common motion options (per call) ----------------------------------------
struct MotionOptions
{
  double velocity_scaling{0.1};
  double acceleration_scaling{0.1};
  double planning_time{5.0};
  int    planning_attempts{1};
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-2};
  bool   plan_only{false};              // plan + validate, do not execute
  // post-processing of the planned trajectory before execution:
  bool   retime_with_totg{false};       // TimeOptimalTrajectoryGeneration
  bool   smooth_with_ruckig{false};     // RuckigSmoothing
  // vNext M3.4 (owner D3): whether the REQUEST asked for the goal tolerance above -- a canonical
  // move whose NAMED motion profile carries it. The MoveGroupInterface routes send a requested
  // tolerance and otherwise MoveGroupInterface's own defaults, as before; the Pilz sequence route
  // always sends goal_position_tolerance / goal_orientation_tolerance (unchanged).
  bool   goal_position_tolerance_requested{false};
  bool   goal_orientation_tolerance_requested{false};
};

// --- Planning options for collision-free moves (Section 13) ------------------
struct PlanningOptions
{
  MotionOptions motion;
  PlannerMode   mode{kDefaultPlannerMode};
};

// --- Targets (Section 13) ----------------------------------------------------
struct JointTarget
{
  std::vector<double> positions;        // explicit joint goal, OR
  std::string         named_state;      // SRDF named state (e.g. "home"); used if positions empty
};

struct CartesianTarget
{
  geometry_msgs::msg::Pose pose;
  std::string tip_link;                 // empty => runtime default TCP
};

struct CircularTarget
{
  geometry_msgs::msg::Pose  goal;
  geometry_msgs::msg::Point aux;        // interim OR center point
  bool aux_is_center{false};            // false => interim/via point, true => center
  std::string tip_link;
};

struct MotionTarget
{
  // exactly one of the two is used (pose first if both set)
  std::optional<geometry_msgs::msg::Pose> pose;
  std::optional<JointTarget>              joint;
  std::string tip_link;
  // TMR 2.6.0 (ADR-0028): the joint configuration a pose was taught in (the group's joints, in its
  // order): the move reaches the pose in the configuration nearest it, or is refused
  std::optional<std::vector<double>>      configuration;
};

// --- Process path: Pilz sequence segment (Section 7) -------------------------
struct ProcessSegment
{
  std::string name;
  ProcessSegmentType type{ProcessSegmentType::LINEAR};

  geometry_msgs::msg::Pose target_pose;

  std::optional<geometry_msgs::msg::Point> circle_center;   // CIRCULAR (center variant)
  std::optional<geometry_msgs::msg::Point> interim_point;   // CIRCULAR (interim variant)

  double velocity_scaling{0.1};
  double acceleration_scaling{0.1};
  double blend_radius{0.0};

  bool process_enabled{false};   // process active during this segment

  // vNext M3.4: the link this segment's goal (and a CIRC's aux point) is for; empty => the
  // backend's tip link. MotionRuntime::moveCircular sets it from CircularTarget::tip_link.
  std::string tip_link;
};

// --- Process path: cartesian waypoint (Section 8) ----------------------------
struct CartesianWaypoint
{
  geometry_msgs::msg::Pose pose;
  double path_velocity{0.1};
  double path_acceleration{0.1};
  bool   process_enabled{false};
};

// --- Process path: full cartesian path description (Section 8) ----------------
struct CartesianProcessPath
{
  std::string path_name;
  std::string reference_frame;          // empty => runtime base frame
  std::string tcp_link;                 // empty => runtime default TCP

  std::vector<CartesianWaypoint> waypoints;

  bool closed_path{false};
  bool maintain_orientation{true};
  bool collision_checking_enabled{true};

  double max_tcp_deviation{0.005};
  double max_orientation_deviation{0.05};
  double waypoint_resolution{0.005};    // eef_step for cartesian computation
};

// --- Process event (Section 10) ----------------------------------------------
struct ProcessEvent
{
  ProcessEventType type{ProcessEventType::TRIGGER};
  std::string channel;                  // a process / tool output channel name
  double value{0.0};                    // for SET_ANALOG_VALUE
};

// --- Metrics: free-space planning (Section 20) -------------------------------
struct FreeSpaceMetrics
{
  std::string planner_mode;
  std::string segment_name;
  std::string pipeline_id;
  std::string planner_id;
  bool   planning_success{false};
  double planning_duration{0.0};        // [s]
  double trajectory_duration{0.0};      // [s]
  std::size_t waypoint_count{0};
  double joint_path_length{0.0};        // sum |dq| over the path [rad]
  double maximum_joint_step{0.0};       // max |dq| between consecutive points [rad]
  bool   execution_success{false};
  int    error_code{0};                 // moveit_msgs::MoveItErrorCodes::val
};

// --- Metrics: process path (Section 21) --------------------------------------
struct ProcessPathMetrics
{
  std::string path_name;
  std::string process_path_backend;
  std::size_t number_of_input_waypoints{0};
  std::size_t number_of_output_waypoints{0};
  double cartesian_fraction{0.0};
  double trajectory_duration{0.0};      // [s]
  double cartesian_path_length{0.0};    // [m]
  double average_tcp_speed{0.0};        // [m/s]
  double maximum_tcp_speed{0.0};        // [m/s]
  double maximum_tcp_acceleration{0.0}; // [m/s^2]
  double maximum_tcp_line_deviation{0.0};     // [m]
  double maximum_orientation_deviation{0.0};  // [rad]
  double minimum_obstacle_clearance{-1.0};    // [m], -1 if not computed
  bool   joint_jump_detected{false};
  bool   execution_success{false};
};

// --- Where a motion failed (vNext M2.7, ADR-0015 amendment M2.7) ----------------------------------
// FACTS the runtime records on a FAILED MotionResult (NONE on success). Whose failure it is -- this
// target's, or the motion infrastructure's -- is decided from them by classifyMotionFailure()
// (motion_failure.hpp), never here.
enum class MotionPhase
{
  NONE,              // success, or a failure no phase was recorded for
  REQUEST,           // refused before MoveIt was asked (MoveWaypoint's own checks)
  PLAN,              // the planning request (MoveGroupInterface::plan, every retry)
  VALIDATE,          // TMR's TrajectoryValidator refused the plan MoveIt returned
  EXECUTE,           // the execution request
  PLAN_AND_EXECUTE,  // ONE request that plans and executes (the Pilz sequence action)
};

// --- Unified result ----------------------------------------------------------
struct MotionResult
{
  bool success{false};
  moveit_msgs::msg::MoveItErrorCodes error_code;
  std::string stage;                    // human-readable failing/last stage
  FreeSpaceMetrics   free_space;
  ProcessPathMetrics process;
  // vNext M2.7: where a failed motion failed, and a cause token when error_code is not the cause
  // (motion_failure.hpp kCause*). Both stay empty on success.
  MotionPhase phase{MotionPhase::NONE};
  std::string cause;

  static MotionResult failure(const std::string& stage_msg, int code = 0)
  {
    MotionResult r;
    r.success = false;
    r.stage = stage_msg;
    r.error_code.val = code;
    return r;
  }
  static MotionResult ok(const std::string& stage_msg = "")
  {
    MotionResult r;
    r.success = true;
    r.stage = stage_msg;
    r.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return r;
  }
};

// --- String <-> enum helpers (used by launch-parameter parsing) --------------
std::optional<PlannerMode>        plannerModeFromString(const std::string& s);
std::string                       toString(PlannerMode m);
std::string                       plannerModeList(const std::string& separator = " | ");
std::optional<ProcessPathBackend> processPathBackendFromString(const std::string& s);
std::string                       toString(ProcessPathBackend b);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_TYPES_HPP_
