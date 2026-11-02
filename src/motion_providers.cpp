// =============================================================================
// motion_providers.cpp -- the C++ mapping of the registry's schema-2 provider rows (ADR-0025).
// Pure data; test_planner_registry checks it against config/planner_profiles.yaml.
// =============================================================================
#include "trainit_motion_runtime/motion_providers.hpp"

#include <algorithm>
#include <utility>

namespace trainit
{

namespace
{
// Namespace-scope constants (no function-static), returned by reference by the accessors.
const std::vector<std::string> kLive = {
  kCapJointPtp, kCapCollisionFreePlanning, kCapCartesianLinear, kCapCartesianCircular,
  kCapMultiSegmentPath, kCapBlending, kCapContinuousPath, kCapVelocityScaling,
  kCapAccelerationScaling, kCapTcpLimits, kCapGoalTolerance, kCapProcessPath};

const std::vector<std::string> kReserved = {
  kCapOrientationConstraints, kCapJerkConstraints, kCapPrecomputedTrajectory};

const std::vector<MotionProvider> kProviders = {
  // vNext M3.4: goal_tolerance on the plan_and_execute rows, with the MoveGroupInterface routes
  // honouring it (MoveItMotionRuntime::doPlan)
  {kProviderPilzPtp, "plan_and_execute", "pilz", "",
   {kCapJointPtp, kCapVelocityScaling, kCapAccelerationScaling, kCapGoalTolerance}, "pilz_trapezoidal", "stable"},
  {kProviderOmpl, "plan_and_execute", "ompl", "",
   {kCapCollisionFreePlanning, kCapVelocityScaling, kCapAccelerationScaling, kCapGoalTolerance}, "totg", "stable"},
  {kProviderOmplChomp, "plan_and_execute", "ompl_chomp", "",
   {kCapCollisionFreePlanning, kCapVelocityScaling, kCapAccelerationScaling, kCapGoalTolerance}, "totg", "stable"},
  {kProviderPilzLin, "plan_and_execute", "", "linear",
   {kCapCartesianLinear, kCapVelocityScaling, kCapAccelerationScaling, kCapTcpLimits, kCapGoalTolerance},
   "pilz_trapezoidal", "stable"},
  {kProviderPilzCirc, "motion_sequence", "", "circular",
   {kCapCartesianCircular, kCapVelocityScaling, kCapAccelerationScaling, kCapTcpLimits,
    kCapGoalTolerance},
   "pilz_trapezoidal", "stable"},
  {kProviderPilzSequence, "motion_sequence", "", "process_path",
   {kCapMultiSegmentPath, kCapBlending, kCapCartesianLinear, kCapCartesianCircular,
    kCapVelocityScaling, kCapAccelerationScaling, kCapTcpLimits, kCapGoalTolerance,
    kCapProcessPath},
   "pilz_trapezoidal", "stable"},
  {kProviderCartesianWaypoints, "cartesian_path", "", "",
   {kCapContinuousPath, kCapVelocityScaling, kCapAccelerationScaling}, "totg", "stable"},
};

const std::vector<std::pair<std::string, std::string>> kDefaults = {
  {"ptp", kProviderPilzPtp},       {"free", kProviderOmpl},
  {"lin", kProviderPilzLin},       {"circ", kProviderPilzCirc},
  {"path", kProviderPilzSequence}, {"dense_path", kProviderCartesianWaypoints}};
}  // namespace

const std::vector<std::string>& liveCapabilities() { return kLive; }
const std::vector<std::string>& reservedCapabilities() { return kReserved; }
const std::vector<MotionProvider>& motionProviders() { return kProviders; }
const std::vector<std::pair<std::string, std::string>>& motionProviderDefaults() { return kDefaults; }

const MotionProvider* findMotionProvider(const std::string& id)
{
  const auto it = std::find_if(kProviders.begin(), kProviders.end(),
                               [&id](const MotionProvider& p) { return p.id == id; });
  return it == kProviders.end() ? nullptr : &*it;
}

std::string defaultMotionProvider(const std::string& kind)
{
  for (const auto& [k, id] : kDefaults)
    if (k == kind) return id;
  return "";
}

std::vector<std::string> missingCapabilities(const MotionProvider& provider,
                                             const std::vector<std::string>& required)
{
  std::vector<std::string> missing;
  for (const auto& token : required)
    if (std::find(provider.provides.begin(), provider.provides.end(), token) == provider.provides.end())
      missing.push_back(token);
  return missing;
}

}  // namespace trainit
