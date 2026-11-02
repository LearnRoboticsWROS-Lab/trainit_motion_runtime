// =============================================================================
// path_library.hpp -- the canonical PATH model and its loader (vNext M3.1, ADR-0024).
//
// TARGET = a desired end state; WAYPOINT = a named, reusable target; PATH = an ordered set of
// STATES joined by PATH SEGMENTS (lin | circ), with a CONTINUITY (stop at every state, or blend
// through them) and path-level data (frame, tcp, profile, scaling, tolerances). The motion type
// belongs to the SEGMENT -- the relation between two states -- never to the state.
//
// A path is DATA: `paths.yaml` (schema 1) names the paths of an application; FollowPath executes
// one as ONE motion through the MotionRuntime door. States[0] is the path's START: FollowPath
// requires the robot to be there (within `start_tolerance`) and moves through states[1..].
//
// Two kinds of path:
//   SEGMENT path  `states` (>= 2) + `segments` (= states - 1), each lin or circ (via | center);
//                 executed by the multi-segment provider (pilz_sequence: one Pilz sequence goal,
//                 a blend radius per segment when `continuity: blend`).
//   DENSE path    `source`: a file (TMR's process-path YAML or CSV) or a generator (polyline,
//                 circle, rectangle, raster) -- states are the samples, segments implicit lin;
//                 `continuity: stop` -> pilz_sequence (a stop at every sample), `blend` -> a
//                 continuous Cartesian path (cartesian_waypoints).
//
// Every required capability (motion_providers.hpp) is derived from the path and checked against
// its provider when the library is LOADED: a request the provider cannot honour never reaches the
// robot. Nothing unsupported is coerced: a ptp segment inside a path, an unknown key, a blend on
// the last segment, a tcp other than the runtime's tip link, a provider that does not provide
// what the path needs -- each is refused by name.
//
// vNext M3.5 (ADR-0024, M3_DESIGN §9): a segment path may carry PROCESS EVENTS, anchored at a STATE
// (after the robot arrives there, or before it departs). The event itself is orchestration -- a BT
// node between two FollowPath runs of the path (FollowPath from= to=) -- and this file records only
// WHERE the path stops for it and its kind: the loader refuses an event the model cannot synchronise
// (a dense path's samples, a blend into an event state, a departure from the last state), and
// FollowPath refuses a run that would drive THROUGH an event state. Every event is blocking: M3 has no
// asynchronous event and no event inside a segment.
//
// Pure: yaml-cpp and the existing path generators/loaders only; no ROS graph, no MoveIt C++ API,
// no function-static, no absolute name.
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__PATH_LIBRARY_HPP_
#define TRAINIT_MOTION_RUNTIME__PATH_LIBRARY_HPP_

#include <algorithm>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "trainit_motion_runtime/motion_types.hpp"

namespace trainit
{

inline constexpr int kPathsSchemaVersion = 1;

// A path definition that cannot be executed as written. `path` is the path's name ("" for a
// file-level problem), `what` says what is wrong and how to fix it.
struct PathError : std::runtime_error
{
  PathError(std::string path_name, const std::string& what);
  std::string path;
};

enum class PathContinuity { STOP, BLEND };
enum class PathSegmentType { LIN, CIRC };

// A path state or a circle's auxiliary point: a WAYPOINT of the application (resolved at tick
// time from the blackboard, `<name>.position` / `.orientation` / `.frame`), or an inline pose in
// the path's frame (an auxiliary point uses only its position).
struct PathPointRef
{
  std::string waypoint;
  std::optional<geometry_msgs::msg::Pose> pose;
  bool isWaypoint() const { return !waypoint.empty(); }
};

struct PathSegmentSpec
{
  PathSegmentType type{PathSegmentType::LIN};
  std::optional<PathPointRef> via;      // circ: a point the arc passes through ...
  std::optional<PathPointRef> center;   // ... or its centre (exactly one of the two)
  std::string profile;                  // a motion profile name ("" = the path's)
  std::optional<double> velocity_scaling;
  std::optional<double> acceleration_scaling;
  double blend_radius_m{0.0};           // into the NEXT segment; the last one is always 0
};

// vNext M3.5: when a process event happens at its state.
enum class PathEventWhen { AFTER_ARRIVAL, BEFORE_DEPARTURE };

// The process event kinds of the canonical model (M3_DESIGN §9): the action itself is a BT node.
inline constexpr const char* kPathEventKinds[] = {"process_on", "process_off", "set_value", "capture",
                                                  "dwell", "external"};

// One event of a path, as paths.yaml records it: {at: {state: <index>, when: ...}, kind: <kind>}.
struct PathEventSpec
{
  std::size_t state{0};
  PathEventWhen when{PathEventWhen::AFTER_ARRIVAL};
  std::string kind;
};

struct PathTolerance
{
  double position_m{0.0};
  double orientation_rad{0.0};
};

struct PathSpec
{
  std::string name;
  std::string provider;                 // a registry provider id (motion_providers.hpp)
  PathContinuity continuity{PathContinuity::STOP};
  std::string frame;                    // inline poses' frame ("" = the runtime's base frame)
  std::string tcp;                      // "" = the runtime's tip link (the only one in M3.1)
  std::string profile;                  // "" = FollowPath's `profile` port
  std::optional<double> velocity_scaling;
  std::optional<double> acceleration_scaling;
  std::optional<PathTolerance> goal_tolerance;
  PathTolerance start_tolerance{0.005, 0.035};   // where the robot must be before it moves

  // SEGMENT path
  std::vector<PathPointRef> states;
  std::vector<PathSegmentSpec> segments;

  // vNext M3.5: the path's process events, in file order, and the states they anchor (sorted, unique):
  // the robot stops at every event state, and a FollowPath run never drives through one.
  std::vector<PathEventSpec> events;
  std::vector<std::size_t> event_states;
  bool isEventState(std::size_t i) const
  { return std::binary_search(event_states.begin(), event_states.end(), i); }

  // DENSE path (loaded and sampled when the library is loaded; `states` stays empty)
  bool dense{false};
  CartesianProcessPath samples;         // reference_frame "" = `frame`, tcp_link "" = `tcp`

  // The capability tokens this path requires of its provider (derived; motion_providers.hpp).
  std::vector<std::string> required;
};

// What the loader must know of the runtime that will execute the paths.
struct PathRuntimeFacts
{
  // The runner's process_path_backend: a continuous dense path (provider cartesian_waypoints)
  // runs only on a runtime configured with that backend, because the door's
  // executeCartesianProcess takes the backend from the runtime's configuration.
  ProcessPathBackend backend{ProcessPathBackend::PILZ_SEQUENCE};
  // The runtime's tip link: M3.1 executes a path for this TCP only (the Pilz sequence backend
  // builds every goal for the configured tip); another `tcp` is refused, not ignored.
  std::string tip_link;
};

class PathLibrary
{
public:
  PathLibrary() = default;
  explicit PathLibrary(std::map<std::string, PathSpec> paths) : paths_(std::move(paths)) {}

  const PathSpec* find(const std::string& name) const;
  std::vector<std::string> names() const;
  std::size_t size() const { return paths_.size(); }

private:
  std::map<std::string, PathSpec> paths_;
};

// Parse and validate `paths.yaml`; relative dense-source files resolve against the file's
// directory. Throws PathError naming the path and the problem.
PathLibrary loadPathLibrary(const std::string& file, const PathRuntimeFacts& runtime);
PathLibrary parsePathLibrary(const std::string& text, const std::string& base_dir,
                             const PathRuntimeFacts& runtime);

// The capability tokens `path` requires of its provider (what the loader checks).
std::vector<std::string> requiredCapabilities(const PathSpec& path);

// The geometric checks that need RESOLVED poses (all in one frame, states[0] = the start), run by
// the loader when every state is inline and by FollowPath once waypoints are resolved:
//   - consecutive states are distinct (a zero-length segment cannot be planned);
//   - a circ via point is distinct from both ends and not collinear with them; a circ centre is
//     equidistant (1 mm) from both ends and not collinear with them (a half circle has no plane);
//   - a blend radius is at most half of each adjacent segment's chord (so blend spheres neither
//     contain a neighbouring state nor overlap).
// `aux[i]` is segment i's via or centre point (nullopt for lin). Returns "" or the reason.
std::string checkPathGeometry(const PathSpec& path,
                              const std::vector<geometry_msgs::msg::Pose>& states,
                              const std::vector<std::optional<geometry_msgs::msg::Point>>& aux);

}  // namespace trainit

#endif  // TRAINIT_MOTION_RUNTIME__PATH_LIBRARY_HPP_
