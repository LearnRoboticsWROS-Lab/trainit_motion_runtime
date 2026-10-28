#include "trainit_motion_runtime/pose_configuration_search.hpp"

#include <cxxabi.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <typeinfo>
#include <utility>

#include <moveit/robot_model/revolute_joint_model.h>
#include <random_numbers/random_numbers.h>

namespace trainit
{

namespace
{
constexpr std::uint32_t kConfigurationSeed = 20261027u;   // fixed: the same seeds, the same choice
constexpr int kConfigurationSpreadSeeds = 64;              // single attempts, spread over the joint limits
constexpr int kConfigurationTimedSpreadSeeds = 8;          // the 2.6.0 timed search
constexpr double kConfigurationIkTimeoutS = 0.05;          // per seed, the timed search [s]
// > 0: RobotState::setFromIK turns a zero timeout into the group's default; KDL makes its first attempt
// from the seed before it looks at the clock, so this is that one attempt and no random re-seed
constexpr double kConfigurationSingleAttemptS = 1e-9;
constexpr double kConfigurationNearEnough = 0.01;          // [rad] a solution this near the reference ends the search
constexpr int kConfigurationCarrySteps = 20;               // the start's tip pose to the goal, in this many steps
constexpr const char* kPlainKdl = "kdl_kinematics_plugin::KDLKinematicsPlugin";
constexpr double kTwoPi = 6.283185307179586;

std::vector<std::vector<double>> seriesSeeds(const moveit::core::RobotState& start,
                                             const moveit::core::JointModelGroup* jmg,
                                             const std::vector<std::vector<double>>& head, int count)
{
  std::vector<std::vector<double>> seeds = head;
  seeds.reserve(head.size() + static_cast<std::size_t>(count));
  random_numbers::RandomNumberGenerator rng(kConfigurationSeed);   // a fixed series of spread seeds
  moveit::core::RobotState spread(start);
  for (int i = 0; i < count; ++i)
  {
    spread.setToRandomPositions(jmg, rng);
    std::vector<double> q;
    spread.copyJointGroupPositions(jmg, q);
    seeds.push_back(std::move(q));
  }
  return seeds;
}

}  // namespace

std::optional<std::vector<double>> facingConfiguration(const moveit::core::RobotState& start,
                                                       const moveit::core::JointModelGroup* jmg,
                                                       const Eigen::Isometry3d& goal, const std::string& tip,
                                                       const std::vector<JointBounds>& bounds)
{
  const std::vector<const moveit::core::JointModel*>& active = jmg->getActiveJointModels();
  const auto* first = active.empty() ? nullptr : dynamic_cast<const moveit::core::RevoluteJointModel*>(active.front());
  const int i = first ? jmg->getVariableGroupIndex(first->getName()) : -1;
  if (i < 0 || static_cast<std::size_t>(i) >= bounds.size() || !first->getParentLinkModel()) return std::nullopt;
  moveit::core::RobotState s(start);
  s.update();
  if (!s.knowsFrameTransform(tip)) return std::nullopt;
  const Eigen::Isometry3d frame =
    s.getGlobalLinkTransform(first->getParentLinkModel()) * first->getChildLinkModel()->getJointOriginTransform();
  const Eigen::Vector3d axis = (frame.linear() * first->getAxis()).normalized();
  const auto across = [&](const Eigen::Vector3d& p) {
    const Eigen::Vector3d v = p - frame.translation();
    return Eigen::Vector3d(v - v.dot(axis) * axis);
  };
  const Eigen::Vector3d from = across(s.getFrameTransform(tip).translation());
  const Eigen::Vector3d to = across(goal.translation());
  if (from.norm() < 1e-6 || to.norm() < 1e-6) return std::nullopt;   // on the axis: no way to face
  std::vector<double> q;
  s.copyJointGroupPositions(jmg, q);
  double v = q[i] + std::atan2(axis.dot(from.cross(to)), from.dot(to));
  const JointBounds& b = bounds[i];
  if (!b.continuous)
  {
    if (v > b.upper) v -= kTwoPi;
    else if (v < b.lower) v += kTwoPi;
    if (v > b.upper || v < b.lower) return std::nullopt;
  }
  q[i] = v;
  return q;
}

namespace
{
// A configuration carried to the goal along the straight line from its tip pose (position linear,
// orientation slerp): one KDL attempt per step, each from the step before -- the branch a move from
// there stays in. One attempt from each spread seed may never land in it (the course cell's 'place':
// 2 distinct solutions from 64 seeds, the arm's own branch not one of them).
std::optional<std::vector<double>> carried(const moveit::core::RobotState& from_state,
                                           const moveit::core::JointModelGroup* jmg,
                                           const Eigen::Isometry3d& goal, const std::string& tip)
{
  moveit::core::RobotState s(from_state);
  s.update();
  if (!s.knowsFrameTransform(tip)) return std::nullopt;
  const Eigen::Isometry3d from = s.getFrameTransform(tip);
  const Eigen::Quaterniond q0(from.rotation());
  const Eigen::Quaterniond q1(goal.rotation());
  for (int k = 1; k <= kConfigurationCarrySteps; ++k)
  {
    const double t = static_cast<double>(k) / kConfigurationCarrySteps;
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    step.translation() = (1.0 - t) * from.translation() + t * goal.translation();
    step.linear() = q0.slerp(t, q1).toRotationMatrix();   // the shorter way round
    if (!s.setFromIK(jmg, step, tip, kConfigurationSingleAttemptS)) return std::nullopt;   // out of reach on the way
  }
  std::vector<double> q;
  s.copyJointGroupPositions(jmg, q);
  return q;
}

std::vector<std::vector<double>> search(const moveit::core::RobotState& start,
                                        const moveit::core::JointModelGroup* jmg,
                                        const Eigen::Isometry3d& goal, const std::string& tip,
                                        const std::vector<std::vector<double>>& seeds, double timeout,
                                        const moveit::core::GroupStateValidityCallbackFn& valid,
                                        const std::vector<double>& reference,
                                        const std::vector<JointBounds>& bounds)
{
  std::vector<std::vector<double>> solutions;
  for (const auto& seed : seeds)
  {
    moveit::core::RobotState s(start);
    s.setJointGroupPositions(jmg, seed);
    s.update();
    if (!s.setFromIK(jmg, goal, tip, timeout, valid)) continue;
    std::vector<double> q;
    s.copyJointGroupPositions(jmg, q);
    // a solution this near the reference is the nearest there can be: no more seeds (the latency of a move)
    const std::vector<double> eq = nearestEquivalent(q, reference, bounds);
    double far = 0.0;
    for (std::size_t i = 0; i < eq.size() && i < reference.size(); ++i) far = std::max(far, std::fabs(eq[i] - reference[i]));
    solutions.push_back(std::move(q));
    if (far < kConfigurationNearEnough) break;
  }
  return solutions;
}
}  // namespace

std::string solverClassName(const moveit::core::JointModelGroup* jmg)
{
  const kinematics::KinematicsBaseConstPtr solver = jmg ? jmg->getSolverInstance() : nullptr;
  if (!solver) return "";
  const kinematics::KinematicsBase& instance = *solver;
  int status = 0;
  const std::unique_ptr<char, void (*)(void*)> name(
    abi::__cxa_demangle(typeid(instance).name(), nullptr, nullptr, &status), std::free);
  return status == 0 && name ? std::string(name.get()) : std::string(typeid(instance).name());
}

std::vector<std::vector<double>> singleAttemptSeeds(const moveit::core::RobotState& start,
                                                   const moveit::core::JointModelGroup* jmg,
                                                   const Eigen::Isometry3d& goal, const std::string& tip,
                                                   const std::vector<std::vector<double>>& head,
                                                   const std::vector<JointBounds>& bounds)
{
  // the start state; the arm's own branch turned to face the goal, and carried there from that turn
  // and from the start (a seed AT a solution: found again, and checked against the scene there);
  // then the spread series
  std::vector<std::vector<double>> first = head;
  if (std::optional<std::vector<double>> t = facingConfiguration(start, jmg, goal, tip, bounds))
  {
    moveit::core::RobotState facing(start);
    facing.setJointGroupPositions(jmg, *t);
    facing.update();
    first.push_back(std::move(*t));
    if (std::optional<std::vector<double>> q = carried(facing, jmg, goal, tip)) first.push_back(std::move(*q));
  }
  if (std::optional<std::vector<double>> q = carried(start, jmg, goal, tip)) first.push_back(std::move(*q));
  return seriesSeeds(start, jmg, first, kConfigurationSpreadSeeds);
}

ConfigurationSearch findConfigurations(const moveit::core::RobotState& start,
                                       const moveit::core::JointModelGroup* jmg,
                                       const Eigen::Isometry3d& goal, const std::string& tip,
                                       const std::vector<std::vector<double>>& head, bool taught,
                                       const moveit::core::GroupStateValidityCallbackFn& valid,
                                       const std::vector<double>& reference,
                                       const std::vector<JointBounds>& bounds)
{
  ConfigurationSearch out;
  if (!taught && solverClassName(jmg) == kPlainKdl)
  {
    out.solutions = search(start, jmg, goal, tip, singleAttemptSeeds(start, jmg, goal, tip, head, bounds),
                           kConfigurationSingleAttemptS, valid, reference, bounds);
    if (!out.solutions.empty()) return out;
  }
  // the 2.6.0 search, exactly: a taught configuration (its own seed keeps its time budget), another
  // solver (one attempt may be no attempt, or not KDL's), or no single attempt found the pose
  out.solutions = search(start, jmg, goal, tip, seriesSeeds(start, jmg, head, kConfigurationTimedSpreadSeeds),
                         kConfigurationIkTimeoutS, valid, reference, bounds);
  out.timed = !taught && !out.solutions.empty();
  return out;
}

}  // namespace trainit
