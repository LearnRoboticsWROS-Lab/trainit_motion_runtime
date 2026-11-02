// =============================================================================
// motion_providers.hpp -- the motion provider capability table (vNext M3.1, ADR-0025).
//
// A motion PROVIDER is one MoveIt route as TMR drives it, described by the capability tokens a
// step served by that route may require. The registry (config/planner_profiles.yaml, schema 2:
// `capabilities`, `providers`, `provider_defaults`) is THE definition; this table is its C++
// mapping, checked entry by entry by test_planner_registry (ADR-0021 Decision 4, option A: data
// beside code, parity-tested). There is no provider class hierarchy: a provider is a row.
// Pure: no ROS, no MoveIt C++ API, no function-static, no absolute name.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_PROVIDERS_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_PROVIDERS_HPP_

#include <string>
#include <utility>
#include <vector>

namespace trainit
{

// --- the capability vocabulary (registry `capabilities.live`) -----------------------------------
inline constexpr const char* kCapJointPtp              = "joint_ptp";
inline constexpr const char* kCapCollisionFreePlanning = "collision_free_planning";
inline constexpr const char* kCapCartesianLinear       = "cartesian_linear";
inline constexpr const char* kCapCartesianCircular     = "cartesian_circular";
inline constexpr const char* kCapMultiSegmentPath      = "multi_segment_path";
inline constexpr const char* kCapBlending              = "blending";
inline constexpr const char* kCapContinuousPath        = "continuous_path";
inline constexpr const char* kCapVelocityScaling       = "velocity_scaling";
inline constexpr const char* kCapAccelerationScaling   = "acceleration_scaling";
inline constexpr const char* kCapTcpLimits             = "tcp_limits";
inline constexpr const char* kCapGoalTolerance         = "goal_tolerance";
inline constexpr const char* kCapProcessPath           = "process_path";
// ... declared and provided by no row (registry `capabilities.reserved`): a request is refused
inline constexpr const char* kCapOrientationConstraints = "orientation_constraints";
inline constexpr const char* kCapJerkConstraints        = "jerk_constraints";
inline constexpr const char* kCapPrecomputedTrajectory  = "precomputed_trajectory";

// --- the provider ids (registry `providers`) -------------------------------------------------------
inline constexpr const char* kProviderPilzPtp            = "pilz_ptp";
inline constexpr const char* kProviderOmpl               = "ompl";
inline constexpr const char* kProviderOmplChomp          = "ompl_chomp";
inline constexpr const char* kProviderPilzLin            = "pilz_lin";
inline constexpr const char* kProviderPilzCirc           = "pilz_circ";
inline constexpr const char* kProviderPilzSequence       = "pilz_sequence";
inline constexpr const char* kProviderCartesianWaypoints = "cartesian_waypoints";

struct MotionProvider
{
  std::string id;
  std::string route;          // plan_and_execute | motion_sequence | cartesian_path
  std::string planner_mode;   // the planner_modes entry it is ("" if none)
  std::string cartesian;      // the cartesian entry it is ("" if none)
  std::vector<std::string> provides;
  std::string time_parameterization;
  std::string stability;
};

// In registry order.
const std::vector<std::string>& liveCapabilities();
const std::vector<std::string>& reservedCapabilities();
const std::vector<MotionProvider>& motionProviders();

// The row of `id`, or nullptr.
const MotionProvider* findMotionProvider(const std::string& id);

// `provider_defaults`: the row a step of `kind` gets when nothing chooses one
// (ptp | free | lin | circ | path | dense_path); "" for an unknown kind.
std::string defaultMotionProvider(const std::string& kind);
const std::vector<std::pair<std::string, std::string>>& motionProviderDefaults();

// The tokens of `required` that `provider` does not provide, in the order given (a reserved
// token is always missing).
std::vector<std::string> missingCapabilities(const MotionProvider& provider,
                                             const std::vector<std::string>& required);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_PROVIDERS_HPP_
