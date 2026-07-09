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
  COLLISION_FREE_OPTIMIZED,   // free-space + optimization (OMPL then CHOMP)
  CARTESIAN_PROCESS_PATH,     // dense cartesian process path
  ONLINE_SERVO                // online cartesian/joint servoing (future)
};

// --- Planner mode: HOW free-space motions are planned (Section 14) -----------
enum class PlannerMode
{
  PILZ,
  OMPL,
  OMPL_CHOMP,
  STOMP   // experimental / guarded (not installed in MoveIt 2.5.9 by default)
};

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
  // True only for OMPL_CHOMP: plan with `pipeline_id` (ompl) then re-optimize
  // with `optimizer_pipeline_id` (chomp), seeding CHOMP with the OMPL result.
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
};

// --- Planning options for collision-free moves (Section 13) ------------------
struct PlanningOptions
{
  MotionOptions motion;
  PlannerMode   mode{PlannerMode::PILZ};
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

// --- Unified result ----------------------------------------------------------
struct MotionResult
{
  bool success{false};
  moveit_msgs::msg::MoveItErrorCodes error_code;
  std::string stage;                    // human-readable failing/last stage
  FreeSpaceMetrics   free_space;
  ProcessPathMetrics process;

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
std::optional<ProcessPathBackend> processPathBackendFromString(const std::string& s);
std::string                       toString(ProcessPathBackend b);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_TYPES_HPP_
