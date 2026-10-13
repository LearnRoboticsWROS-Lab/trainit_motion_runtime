// =============================================================================
// motion_types.cpp — string <-> enum helpers for launch-parameter parsing.
// =============================================================================
#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

// One token table: toString(). The parser accepts exactly the tokens of kPlannerModes,
// so a token is never typed twice (test_planner_registry checks them against the registry).
std::optional<PlannerMode> plannerModeFromString(const std::string& s)
{
  for (const PlannerMode m : kPlannerModes)
    if (s == toString(m)) return m;
  return std::nullopt;
}

std::string plannerModeList(const std::string& separator)
{
  std::string out;
  for (const PlannerMode m : kPlannerModes)
    out += (out.empty() ? std::string() : separator) + toString(m);
  return out;
}

std::string toString(PlannerMode m)
{
  switch (m)
  {
    case PlannerMode::PILZ:       return "pilz";
    case PlannerMode::OMPL:       return "ompl";
    case PlannerMode::OMPL_CHOMP: return "ompl_chomp";
    case PlannerMode::STOMP:      return "stomp";
  }
  return "pilz";
}

std::optional<ProcessPathBackend> processPathBackendFromString(const std::string& s)
{
  if (s == "pilz_sequence")       return ProcessPathBackend::PILZ_SEQUENCE;
  if (s == "cartesian_waypoints") return ProcessPathBackend::CARTESIAN_WAYPOINTS;
  return std::nullopt;
}

std::string toString(ProcessPathBackend b)
{
  switch (b)
  {
    case ProcessPathBackend::PILZ_SEQUENCE:       return "pilz_sequence";
    case ProcessPathBackend::CARTESIAN_WAYPOINTS: return "cartesian_waypoints";
  }
  return "pilz_sequence";
}

}  // namespace trainit
