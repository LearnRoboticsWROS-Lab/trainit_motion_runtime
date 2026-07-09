// =============================================================================
// manual_motion_strategy_selector.hpp — launch-parameter driven selector.
//
// Holds a fixed PlannerMode and ProcessPathBackend (from the launch file) and
// resolves them per MotionIntent following the strategy tables in Sections 5-6.
// Guards unavailable pipelines (e.g. STOMP not installed) by degrading to OMPL.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MANUAL_MOTION_STRATEGY_SELECTOR_HPP_
#define TRAINIT_MOTION_RUNTIME__MANUAL_MOTION_STRATEGY_SELECTOR_HPP_

#include <string>
#include <vector>

#include "trainit_motion_runtime/motion_strategy_selector.hpp"

namespace trainit
{

// Canonical pipeline ids used by the framework.
inline constexpr const char* PIPELINE_OMPL  = "ompl";
inline constexpr const char* PIPELINE_PILZ  = "pilz_industrial_motion_planner";
inline constexpr const char* PIPELINE_CHOMP = "chomp";
inline constexpr const char* PIPELINE_STOMP = "stomp";
inline constexpr const char* OMPL_DEFAULT_PLANNER = "RRTConnectkConfigDefault";

class ManualMotionStrategySelector : public MotionStrategySelector
{
public:
  // available_pipelines: pipeline ids actually loaded by move_group at runtime
  // (used to guard STOMP / CHOMP). Empty => assume all requested are available.
  ManualMotionStrategySelector(PlannerMode mode,
                               ProcessPathBackend backend,
                               std::vector<std::string> available_pipelines = {});

  PlannerProfile selectPlanner(MotionIntent intent,
                               const std::string& segment_name) const override;

  ProcessPathBackend selectProcessPathBackend(const CartesianProcessPath& path) const override;

  void        setMode(PlannerMode mode) { mode_ = effectiveMode(mode); }
  PlannerMode mode() const { return mode_; }
  ProcessPathBackend backend() const { return backend_; }

  bool pipelineAvailable(const std::string& pipeline_id) const;

private:
  // Degrade unavailable modes (e.g. STOMP -> OMPL) with the available pipelines.
  PlannerMode effectiveMode(PlannerMode requested) const;

  PlannerMode        mode_;
  ProcessPathBackend backend_;
  std::vector<std::string> available_pipelines_;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MANUAL_MOTION_STRATEGY_SELECTOR_HPP_
