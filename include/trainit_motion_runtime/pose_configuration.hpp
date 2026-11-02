// =============================================================================
// pose_configuration.hpp -- which joint configuration a PTP / free move to a pose takes (TMR 2.6.0,
// ADR-0028). An arm reaches a pose in several configurations (its shoulder, elbow and wrist branches,
// and every 2*pi on a joint whose range allows it); left to the planner's IK (random restarts) the
// configuration can change from one cycle to the next. The runtime chooses ONE, deterministically:
// the configuration nearest a reference -- the one the pose was taught in when the waypoint names it,
// else the robot's start state -- and a taught configuration the pose cannot be reached near is
// refused, never silently replaced (what an industrial controller does with its stored configuration).
// Pure: no ROS, no MoveIt.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_HPP_
#define TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace trainit
{

// How a PTP / free move to a pose gets its joint configuration (the runner parameter
// pose_goal_configuration): NEAREST the reference (the default), or PLANNER (the pose goal is sent
// as it is and the planner's IK chooses: TMR 2.5.1 and before).
enum class PoseGoalConfiguration { NEAREST, PLANNER };
inline constexpr const char* kPoseGoalConfigurationParam = "pose_goal_configuration";
inline constexpr const char* kConfigurationToleranceParam = "configuration_tolerance";
inline constexpr double kDefaultConfigurationTolerance = 1.5707963267948966;   // pi/2 [rad]
std::optional<PoseGoalConfiguration> poseGoalConfigurationFromString(const std::string& text);
const char* toString(PoseGoalConfiguration mode);

// A joint variable's position bounds. `angular`: a revolute variable (a multiple of 2*pi is the same
// position; a prismatic one never moves); `continuous`: no bounds.
struct JointBounds
{
  double lower{0.0};
  double upper{0.0};
  bool continuous{false};
  bool angular{true};
};

// Each angular value moved by a multiple of 2*pi to the equivalent nearest the reference's, within its
// joint's bounds (a value whose joint allows no other equivalent is kept; a non-angular one is kept).
std::vector<double> nearestEquivalent(const std::vector<double>& joints, const std::vector<double>& reference,
                                      const std::vector<JointBounds>& bounds);

struct ConfigurationChoice
{
  std::optional<std::vector<double>> joints;   // the configuration chosen; nullopt: none, or refused
  std::size_t distinct{0};                     // the distinct solutions it was chosen from
  double max_difference{0.0};                  // the nearest one's largest joint difference to the reference
  std::size_t worst_joint{0};                  // ... on this joint (index)
  bool refused{false};                         // the nearest is beyond the tolerance (a taught configuration)
};

// The solution nearest the reference: each solution first moved to its nearest equivalent; nearest =
// the smallest largest joint difference, then the smallest sum. Solutions equal within 1e-4 rad on
// every joint count once; a solution with a non-finite value, or of another size, is never chosen.
// With a tolerance (a taught configuration), a nearest one beyond it is refused (joints nullopt, refused
// true; max_difference / worst_joint say how far, on which joint). The caller refuses a non-finite
// reference before (allFinite).
ConfigurationChoice chooseConfiguration(const std::vector<std::vector<double>>& solutions,
                                        const std::vector<double>& reference,
                                        const std::vector<JointBounds>& bounds,
                                        std::optional<double> tolerance);

// Every value finite (a taught configuration with a NaN would pass any tolerance: refused instead).
bool allFinite(const std::vector<double>& values);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_HPP_
