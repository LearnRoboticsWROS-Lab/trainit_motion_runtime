// =============================================================================
// pose_configuration_search.hpp -- the IK solutions a pose goal's joint configuration is chosen from
// (TMR 2.6.2, D7; the choice itself: pose_configuration.hpp, ADR-0028).
//
// KDL makes ONE attempt from the seed it is given, then re-seeds at RANDOM for as long as its time
// budget lasts. The 2.6.0 search gave every seed such a budget, so it found a different set of
// solutions from one run to the next -- and more or fewer under load -- and the nearest of them, the
// configuration the arm moved in, changed (2 runs in 8 of the ROS2ML course cell, 2026-10-28).
//
// For a pose with no taught configuration and MoveIt's plain KDL plugin, one attempt from each seed: the
// start state; the start with its first joint turned to face the goal, and that configuration carried
// to the goal in small steps (as the start itself); then a fixed series of 64 -- spread seeds alone
// found 2 solutions of the course cell's 'place', always the elbow branch, never the arm's own. A
// function of the start state, the pose and the scene
// (residual: KDL's own escape from a singularity draws std::rand(), on a stuck attempt only).
// Everything else keeps the 2.6.0 timed search, unchanged: a taught configuration (its own seed is
// the answer, and keeps its time budget),
// another solver (cached_ik seeds from its cache; others may need time for any answer), or a pose no
// single attempt reached.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_SEARCH_HPP_
#define TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_SEARCH_HPP_

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include "trainit_motion_runtime/pose_configuration.hpp"

namespace trainit
{

// The solutions found, in seed order; `timed`: a pose with no taught configuration resolved by the
// time-limited search, whose solutions -- and choice -- may differ from run to run.
struct ConfigurationSearch
{
  std::vector<std::vector<double>> solutions;
  bool timed{false};
};

// `head`: the taught configuration (when `taught`), then the start state. The seeds are `head`, then
// a fixed series spread over the group's bounds; solutions pass `valid`; the search stops at one
// within 0.01 rad of `reference` (no nearer one exists).
ConfigurationSearch findConfigurations(const moveit::core::RobotState& start,
                                       const moveit::core::JointModelGroup* jmg,
                                       const Eigen::Isometry3d& goal, const std::string& tip,
                                       const std::vector<std::vector<double>>& head, bool taught,
                                       const moveit::core::GroupStateValidityCallbackFn& valid,
                                       const std::vector<double>& reference,
                                       const std::vector<JointBounds>& bounds);

// The seeds of the single-attempt search (no taught configuration, plain KDL), in order: `head`, the
// facing configuration and the facing / start configurations carried to the goal (each when there is
// one), then the fixed spread series.
std::vector<std::vector<double>> singleAttemptSeeds(const moveit::core::RobotState& start,
                                                   const moveit::core::JointModelGroup* jmg,
                                                   const Eigen::Isometry3d& goal, const std::string& tip,
                                                   const std::vector<std::vector<double>>& head,
                                                   const std::vector<JointBounds>& bounds);

// The group's IK solver class, demangled ("" without one): which search a pose goal gets.
std::string solverClassName(const moveit::core::JointModelGroup* jmg);

// `start` with the group's first joint turned about its axis to face the goal's position, the other
// joints as they are, within that joint's bounds: the arm's own branch pointed where the move goes --
// the seed that finds it (nullopt: the first joint is not revolute, the goal is on its axis, the turn
// leaves the joint's range, or `tip` is no frame of the state).
std::optional<std::vector<double>> facingConfiguration(const moveit::core::RobotState& start,
                                                       const moveit::core::JointModelGroup* jmg,
                                                       const Eigen::Isometry3d& goal, const std::string& tip,
                                                       const std::vector<JointBounds>& bounds);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__POSE_CONFIGURATION_SEARCH_HPP_
