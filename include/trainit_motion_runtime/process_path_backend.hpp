// =============================================================================
// process_path_backend.hpp — base for process-path planning backends.
//
// Two concrete backends (Section 6):
//   * PilzSequenceBackend     (pilz_sequence_backend.hpp)      — LIN/CIRC blended
//   * CartesianWaypointBackend (cartesian_waypoint_backend.hpp) — dense cartesian
//
// They take different inputs (ProcessSegment list vs CartesianProcessPath), so
// the shared base only carries identification; the runtime calls the concrete
// entry point for the selected backend.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PROCESS_PATH_BACKEND_HPP_
#define TRAINIT_MOTION_RUNTIME__PROCESS_PATH_BACKEND_HPP_

#include <string>

namespace trainit
{

class ProcessPathBackendBase
{
public:
  virtual ~ProcessPathBackendBase() = default;
  virtual std::string name() const = 0;
  virtual bool supportsCircular() const = 0;
};

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PROCESS_PATH_BACKEND_HPP_
