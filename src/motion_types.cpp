// =============================================================================
// motion_types.cpp — string <-> enum helpers for launch-parameter parsing.
// =============================================================================
#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

std::optional<PlannerMode> plannerModeFromString(const std::string& s)
{
  if (s == "pilz")       return PlannerMode::PILZ;
  if (s == "ompl")       return PlannerMode::OMPL;
  if (s == "ompl_chomp") return PlannerMode::OMPL_CHOMP;
  if (s == "stomp")      return PlannerMode::STOMP;
  return std::nullopt;
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
