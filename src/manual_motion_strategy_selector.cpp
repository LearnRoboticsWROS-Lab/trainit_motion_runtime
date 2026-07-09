// =============================================================================
// manual_motion_strategy_selector.cpp — strategy tables (Sections 5-6) + guards.
//
// Key design rule: APPROACH/RETREAT and CIRCULAR segments are ALWAYS Pilz
// LIN/CIRC regardless of planner_mode (deterministic MoveL/MoveC; KDL Cartesian
// stalls). Only the free-space transfer (PTP / COLLISION_FREE) follows the mode.
// =============================================================================
#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"

#include <algorithm>

namespace trainit
{

ManualMotionStrategySelector::ManualMotionStrategySelector(
  PlannerMode mode, ProcessPathBackend backend, std::vector<std::string> available_pipelines)
: backend_(backend), available_pipelines_(std::move(available_pipelines))
{
  mode_ = effectiveMode(mode);
}

bool ManualMotionStrategySelector::pipelineAvailable(const std::string& pipeline_id) const
{
  if (available_pipelines_.empty())
    return true;  // unknown set => do not block
  return std::find(available_pipelines_.begin(), available_pipelines_.end(), pipeline_id)
         != available_pipelines_.end();
}

PlannerMode ManualMotionStrategySelector::effectiveMode(PlannerMode requested) const
{
  // Degrade unavailable modes to OMPL (the universally available free-space planner).
  if (requested == PlannerMode::STOMP && !pipelineAvailable(PIPELINE_STOMP))
    return PlannerMode::OMPL;
  if (requested == PlannerMode::OMPL_CHOMP && !pipelineAvailable(PIPELINE_CHOMP))
    return PlannerMode::OMPL;
  return requested;
}

ProcessPathBackend ManualMotionStrategySelector::selectProcessPathBackend(
  const CartesianProcessPath& /*path*/) const
{
  return backend_;
}

PlannerProfile ManualMotionStrategySelector::selectPlanner(
  MotionIntent intent, const std::string& /*segment_name*/) const
{
  // --- Cartesian-critical intents: deterministic Pilz, mode-independent -------
  if (intent == MotionIntent::LINEAR)
    return PlannerProfile{PIPELINE_PILZ, "LIN", false, ""};
  if (intent == MotionIntent::CIRCULAR)
    return PlannerProfile{PIPELINE_PILZ, "CIRC", false, ""};
  // CARTESIAN_PROCESS_PATH is handled by a process-path backend, not a single
  // planner; default a process LIN segment if a profile is requested.
  if (intent == MotionIntent::CARTESIAN_PROCESS_PATH)
    return PlannerProfile{PIPELINE_PILZ, "LIN", false, ""};

  // --- Free-space transfer (PTP / COLLISION_FREE[_OPTIMIZED]) by mode ---------
  const bool force_optimize = (intent == MotionIntent::COLLISION_FREE_OPTIMIZED);

  switch (mode_)
  {
    case PlannerMode::PILZ:
      // Pilz PTP: joint-space, collision-checked (no avoidance routing).
      return PlannerProfile{PIPELINE_PILZ, "PTP", false, ""};

    case PlannerMode::OMPL:
      if (force_optimize && pipelineAvailable(PIPELINE_CHOMP))
        return PlannerProfile{PIPELINE_OMPL, OMPL_DEFAULT_PLANNER, true, PIPELINE_CHOMP};
      return PlannerProfile{PIPELINE_OMPL, OMPL_DEFAULT_PLANNER, false, ""};

    case PlannerMode::OMPL_CHOMP:
      return PlannerProfile{PIPELINE_OMPL, OMPL_DEFAULT_PLANNER, true, PIPELINE_CHOMP};

    case PlannerMode::STOMP:
      // Reached only if STOMP pipeline is available (else effectiveMode degraded it).
      return PlannerProfile{PIPELINE_STOMP, "STOMP", false, ""};
  }
  return PlannerProfile{PIPELINE_PILZ, "PTP", false, ""};
}

}  // namespace trainit
