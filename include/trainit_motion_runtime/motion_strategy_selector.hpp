// =============================================================================
// motion_strategy_selector.hpp — maps intent + mode to a concrete planner
// profile, and chooses a process-path backend (Section 14).
//
// This iteration ships ManualMotionStrategySelector (driven by launch params).
// Future selectors (rule-based, metrics-based, parallel, policy-based) implement
// the same interface.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__MOTION_STRATEGY_SELECTOR_HPP_
#define TRAINIT_MOTION_RUNTIME__MOTION_STRATEGY_SELECTOR_HPP_

#include <string>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

class MotionStrategySelector
{
public:
  virtual ~MotionStrategySelector() = default;

  // Resolve the pipeline/planner for a free-space or discrete motion.
  virtual PlannerProfile selectPlanner(MotionIntent intent,
                                       const std::string& segment_name) const = 0;

  // Resolve which process-path backend to use for a cartesian path.
  virtual ProcessPathBackend selectProcessPathBackend(const CartesianProcessPath& path) const = 0;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__MOTION_STRATEGY_SELECTOR_HPP_
