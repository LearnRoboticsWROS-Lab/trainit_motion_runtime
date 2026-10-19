// =============================================================================
// path_library.cpp -- paths.yaml (schema 1) -> validated PathSpecs (vNext M3.1, ADR-0024).
// =============================================================================
#include "trainit_motion_runtime/path_library.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "trainit_motion_runtime/motion_providers.hpp"
#include "trainit_motion_runtime/process_path_generator.hpp"

namespace trainit
{

PathError::PathError(std::string path_name, const std::string& what)
: std::runtime_error(path_name.empty() ? "paths: " + what : "path '" + path_name + "': " + what),
  path(std::move(path_name))
{
}

namespace
{
namespace fs = std::filesystem;

// The keys each level accepts. Anything else is refused: a typo must never be a silent default.
const std::set<std::string> kRootKeys = {"schema_version", "paths"};
const std::set<std::string> kPathKeys = {
  "provider", "continuity", "frame", "tcp", "profile", "velocity_scaling", "acceleration_scaling",
  "goal_tolerance", "start_tolerance", "states", "segments", "source", "events"};
// vNext M3.5: a path event, and its anchor. The event's action (channel, detector, value) is the BT
// node between the runs, not path data: only where and what kind.
const std::set<std::string> kEventKeys = {"at", "kind"};
const std::set<std::string> kAnchorKeys = {"state", "when"};
const std::set<std::string> kSegmentKeys = {
  "type", "via", "center", "profile", "velocity_scaling", "acceleration_scaling", "blend_radius_m"};
const std::set<std::string> kToleranceKeys = {"position_m", "orientation_rad"};
const std::set<std::string> kSourceKeys = {"file", "generator", "orientation"};
const std::set<std::string> kGeneratorKeys = {
  "type", "points", "closed", "center", "radius", "segments", "size_x", "size_y", "rows"};
// TMR's process-path YAML, as FollowPath honours it. The per-sample `velocity`, `acceleration` and
// `process` keys and the `maintain_orientation` / `max_*_deviation` keys exist in that format, but
// nothing in the path model honours them (the path's profile sets the speed; process events are
// M3.5): a dense file that carries one is refused rather than read as if it had none.
const std::set<std::string> kDenseFileKeys = {
  "path_name", "reference_frame", "tcp_link", "closed_path", "collision_checking",
  "waypoint_resolution", "orientation", "waypoints"};
const std::set<std::string> kDenseWaypointKeys = {"position", "orientation"};

void checkKeys(const YAML::Node& n, const std::set<std::string>& allowed, const std::string& path,
               const std::string& where)
{
  if (!n.IsMap())
    throw PathError(path, where + " must be a map");
  std::set<std::string> seen;
  for (const auto& kv : n)
  {
    const std::string key = kv.first.as<std::string>();
    // yaml-cpp keeps a repeated key and a lookup returns one of them: a repeated key is refused
    if (!seen.insert(key).second)
      throw PathError(path, "key '" + key + "' is given twice in " + where);
    if (!allowed.count(key))
    {
      std::string list;
      for (const auto& a : allowed) list += (list.empty() ? "" : ", ") + a;
      throw PathError(path, "unknown key '" + key + "' in " + where + " (accepted: " + list + ")");
    }
  }
}

double finite(const YAML::Node& n, const std::string& path, const std::string& what)
{
  double v = 0.0;
  try { v = n.as<double>(); }
  catch (const std::exception&) { throw PathError(path, what + " must be a number"); }
  if (!std::isfinite(v)) throw PathError(path, what + " must be finite");
  return v;
}

double scaling(const YAML::Node& n, const std::string& path, const std::string& what)
{
  const double v = finite(n, path, what);
  if (v <= 0.0 || v > 1.0)
    throw PathError(path, what + " must be in (0, 1], got " + std::to_string(v));
  return v;
}

std::vector<double> numbers(const YAML::Node& n, std::size_t count, const std::string& path,
                            const std::string& what)
{
  if (!n || !n.IsSequence() || n.size() != count)
    throw PathError(path, what + " must be a list of " + std::to_string(count) + " numbers");
  std::vector<double> v;
  for (const auto& e : n) v.push_back(finite(e, path, what));
  return v;
}

geometry_msgs::msg::Point point(const YAML::Node& n, const std::string& path, const std::string& what)
{
  const auto v = numbers(n, 3, path, what);
  geometry_msgs::msg::Point p;
  p.x = v[0]; p.y = v[1]; p.z = v[2];
  return p;
}

geometry_msgs::msg::Quaternion quaternion(const YAML::Node& n, const std::string& path,
                                          const std::string& what)
{
  const auto v = numbers(n, 4, path, what + " (qx, qy, qz, qw)");
  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
  if (std::abs(norm - 1.0) > 1e-3)
    throw PathError(path, what + " is not a unit quaternion (norm " + std::to_string(norm) +
                          "): it is not normalised silently");
  geometry_msgs::msg::Quaternion q;
  q.x = v[0]; q.y = v[1]; q.z = v[2]; q.w = v[3];
  return q;
}

std::string text(const YAML::Node& n, const std::string& path, const std::string& what)
{
  if (!n.IsScalar()) throw PathError(path, what + " must be a string");
  const std::string s = n.as<std::string>();
  if (s.empty()) throw PathError(path, what + " must not be empty");
  return s;
}

PathTolerance tolerance(const YAML::Node& n, const std::string& path, const std::string& what)
{
  checkKeys(n, kToleranceKeys, path, what);
  if (!n["position_m"] || !n["orientation_rad"])
    throw PathError(path, what + " needs both position_m and orientation_rad");
  PathTolerance t;
  t.position_m = finite(n["position_m"], path, what + ".position_m");
  t.orientation_rad = finite(n["orientation_rad"], path, what + ".orientation_rad");
  if (t.position_m <= 0.0 || t.orientation_rad <= 0.0)
    throw PathError(path, what + " must be > 0");
  return t;
}

// A state: {waypoint: <name>} or {pose: {position: [3], orientation: [4]}}.
PathPointRef state(const YAML::Node& n, const std::string& path, const std::string& what)
{
  checkKeys(n, {"waypoint", "pose"}, path, what);
  if (static_cast<bool>(n["waypoint"]) == static_cast<bool>(n["pose"]))
    throw PathError(path, what + " is either {waypoint: <name>} or {pose: {position, orientation}}");
  PathPointRef r;
  if (n["waypoint"])
  {
    r.waypoint = text(n["waypoint"], path, what + ".waypoint");
    return r;
  }
  checkKeys(n["pose"], {"position", "orientation"}, path, what + ".pose");
  if (!n["pose"]["position"] || !n["pose"]["orientation"])
    throw PathError(path, what + ".pose needs position and orientation");
  geometry_msgs::msg::Pose p;
  p.position = point(n["pose"]["position"], path, what + ".pose.position");
  p.orientation = quaternion(n["pose"]["orientation"], path, what + ".pose.orientation");
  r.pose = p;
  return r;
}

// A circle's auxiliary point: {waypoint: <name>} (its position) or {position: [3]}.
PathPointRef auxPoint(const YAML::Node& n, const std::string& path, const std::string& what)
{
  checkKeys(n, {"waypoint", "position"}, path, what);
  if (static_cast<bool>(n["waypoint"]) == static_cast<bool>(n["position"]))
    throw PathError(path, what + " is either {waypoint: <name>} or {position: [x, y, z]}");
  PathPointRef r;
  if (n["waypoint"])
  {
    r.waypoint = text(n["waypoint"], path, what + ".waypoint");
    return r;
  }
  geometry_msgs::msg::Pose p;
  p.position = point(n["position"], path, what + ".position");
  p.orientation.w = 1.0;
  r.pose = p;
  return r;
}

PathSegmentSpec segment(const YAML::Node& n, const std::string& path, std::size_t i)
{
  const std::string where = "segments[" + std::to_string(i) + "]";
  checkKeys(n, kSegmentKeys, path, where);
  if (!n["type"]) throw PathError(path, where + " needs a type (lin | circ)");
  const std::string type = text(n["type"], path, where + ".type");
  PathSegmentSpec s;
  if (type == "lin") s.type = PathSegmentType::LIN;
  else if (type == "circ") s.type = PathSegmentType::CIRC;
  else if (type == "ptp")
    throw PathError(path, where + ": UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL: a joint-space (ptp) "
                          "segment inside a path is reserved; reach the path's start with a ptp move "
                          "and keep the path Cartesian (lin | circ)");
  else throw PathError(path, where + ": unknown segment type '" + type + "' (accepted: lin | circ)");

  if (s.type == PathSegmentType::LIN && (n["via"] || n["center"]))
    throw PathError(path, where + ": a lin segment has no via or center");
  if (s.type == PathSegmentType::CIRC)
  {
    if (static_cast<bool>(n["via"]) == static_cast<bool>(n["center"]))
      throw PathError(path, where + ": a circ segment needs exactly one of via or center");
    if (n["via"]) s.via = auxPoint(n["via"], path, where + ".via");
    else s.center = auxPoint(n["center"], path, where + ".center");
  }
  if (n["profile"]) s.profile = text(n["profile"], path, where + ".profile");
  if (n["velocity_scaling"]) s.velocity_scaling = scaling(n["velocity_scaling"], path, where + ".velocity_scaling");
  if (n["acceleration_scaling"])
    s.acceleration_scaling = scaling(n["acceleration_scaling"], path, where + ".acceleration_scaling");
  if (n["blend_radius_m"])
  {
    s.blend_radius_m = finite(n["blend_radius_m"], path, where + ".blend_radius_m");
    if (s.blend_radius_m < 0.0) throw PathError(path, where + ".blend_radius_m must be >= 0");
  }
  return s;
}

// A dense CSV: every non-comment line exactly x,y,z. More columns (velocity, acceleration,
// process) exist in TMR's CSV format but are not honoured by the path model: refused.
void checkDenseCsv(const std::string& file, const std::string& path)
{
  std::ifstream in(file);
  if (!in.is_open()) throw PathError(path, "cannot open the source file " + file);
  std::string line;
  int number = 0;
  while (std::getline(in, line))
  {
    ++number;
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string cell;
    int columns = 0;
    while (std::getline(ss, cell, ','))
    {
      ++columns;
      try { std::size_t used = 0; std::stod(cell, &used); }
      catch (const std::exception&)
      {
        throw PathError(path, file + ":" + std::to_string(number) + ": '" + cell +
                              "' is not a number (a dense CSV holds x,y,z lines and # comments)");
      }
    }
    if (columns != 3)
      throw PathError(path, file + ":" + std::to_string(number) + ": " + std::to_string(columns) +
                            " columns; a dense path's CSV is x,y,z (per-sample velocity, acceleration "
                            "and process flags are not part of the path model: use the path's profile)");
  }
}

void checkDenseYaml(const std::string& file, const std::string& path)
{
  YAML::Node root;
  try { root = YAML::LoadFile(file); }
  catch (const std::exception& e) { throw PathError(path, "cannot read the source file " + file + ": " + e.what()); }
  checkKeys(root, kDenseFileKeys, path, "the source file " + file);
  if (!root["waypoints"] || !root["waypoints"].IsSequence())
    throw PathError(path, "the source file " + file + " has no waypoints list");
  for (std::size_t i = 0; i < root["waypoints"].size(); ++i)
    checkKeys(root["waypoints"][i], kDenseWaypointKeys, path,
              "the source file's waypoints[" + std::to_string(i) + "]");
}

void denseSource(const YAML::Node& n, PathSpec& spec, const std::string& base_dir,
                 const std::string& tip_link)
{
  const std::string& path = spec.name;
  checkKeys(n, kSourceKeys, path, "source");
  if (static_cast<bool>(n["file"]) == static_cast<bool>(n["generator"]))
    throw PathError(path, "source is either {file: ...} or {generator: {...}}");

  PathFrame frame;
  frame.reference_frame = spec.frame;   // "" = the runtime's base frame
  frame.tcp_link = spec.tcp;
  const bool needs_orientation = n["generator"] ||
    (n["file"] && fs::path(n["file"].as<std::string>()).extension() == ".csv");
  if (n["orientation"]) frame.orientation = quaternion(n["orientation"], path, "source.orientation");
  else if (needs_orientation)
    throw PathError(path, "source.orientation is required: a generated or CSV path carries no orientation");

  if (n["file"])
  {
    if (n["orientation"] && !needs_orientation)
      throw PathError(path, "source.orientation is for a CSV or a generator; a YAML path file carries its own");
    fs::path file(text(n["file"], path, "source.file"));
    if (file.is_relative()) file = fs::path(base_dir) / file;
    const std::string f = file.string();
    if (file.extension() == ".csv")
    {
      checkDenseCsv(f, path);
      spec.samples = CsvProcessPathLoader::fromFile(f, frame, path);
    }
    else
    {
      checkDenseYaml(f, path);
      try { spec.samples = YamlProcessPathLoader::fromFile(f); }
      catch (const std::exception& e) { throw PathError(path, "the source file " + f + ": " + e.what()); }
      if (!spec.samples.reference_frame.empty() && !spec.frame.empty() &&
          spec.samples.reference_frame != spec.frame)
        throw PathError(path, "the source file's reference_frame '" + spec.samples.reference_frame +
                              "' differs from the path's frame '" + spec.frame + "'");
      if (spec.samples.reference_frame.empty()) spec.samples.reference_frame = spec.frame;
      const std::string want_tcp = spec.tcp.empty() ? tip_link : spec.tcp;
      if (!spec.samples.tcp_link.empty() && spec.samples.tcp_link != want_tcp)
        throw PathError(path, "the source file's tcp_link '" + spec.samples.tcp_link + "' is not the " +
                              (spec.tcp.empty() ? "runtime's tip link '" : "path's tcp '") + want_tcp + "'");
      spec.samples.tcp_link = spec.tcp;
    }
  }
  else
  {
    const YAML::Node g = n["generator"];
    checkKeys(g, kGeneratorKeys, path, "source.generator");
    if (!g["type"]) throw PathError(path, "source.generator needs a type (polyline | circle | rectangle | raster)");
    const std::string type = text(g["type"], path, "source.generator.type");
    std::set<std::string> used{"type"};
    auto need = [&](const char* key) {
      if (!g[key]) throw PathError(path, "a " + type + " generator needs '" + key + "'");
      used.insert(key);
      return g[key];
    };
    auto positive = [&](const char* key) {
      const double v = finite(need(key), path, std::string("source.generator.") + key);
      if (v <= 0.0) throw PathError(path, std::string("source.generator.") + key + " must be > 0");
      return v;
    };
    if (type == "polyline")
    {
      std::vector<geometry_msgs::msg::Point> pts;
      const YAML::Node list = need("points");
      if (!list.IsSequence()) throw PathError(path, "source.generator.points must be a list of [x, y, z]");
      for (const auto& p : list) pts.push_back(point(p, path, "source.generator.points[]"));
      bool closed = false;
      if (g["closed"]) { used.insert("closed"); closed = g["closed"].as<bool>(); }
      spec.samples = PolylinePathGenerator(frame, pts, closed, path).generate();
    }
    else if (type == "circle")
    {
      int segments = 36;
      if (g["segments"]) { used.insert("segments"); segments = g["segments"].as<int>(); }
      spec.samples = CirclePathGenerator(frame, point(need("center"), path, "source.generator.center"),
                                         positive("radius"), segments, path).generate();
    }
    else if (type == "rectangle")
    {
      const auto c = point(need("center"), path, "source.generator.center");
      spec.samples = RectanglePathGenerator(frame, c, positive("size_x"), positive("size_y"), path).generate();
    }
    else if (type == "raster")
    {
      const auto c = point(need("center"), path, "source.generator.center");
      const double sx = positive("size_x"), sy = positive("size_y");
      const int rows = need("rows").as<int>();
      spec.samples = RasterPathGenerator(frame, c, sx, sy, rows, path).generate();
    }
    else throw PathError(path, "unknown generator type '" + type + "' (accepted: polyline | circle | rectangle | raster)");
    for (const auto& kv : g)
      if (!used.count(kv.first.as<std::string>()))
        throw PathError(path, "source.generator: '" + kv.first.as<std::string>() +
                              "' is not a parameter of a " + type + " generator");
  }
  if (spec.samples.waypoints.size() < 2)
    throw PathError(path, "a dense source must give at least 2 samples (the start and one more)");
  for (std::size_t i = 0; i < spec.samples.waypoints.size(); ++i)
  {
    const auto& q = spec.samples.waypoints[i].pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (std::abs(norm - 1.0) > 1e-3)
      throw PathError(path, "sample " + std::to_string(i) + "'s orientation is not a unit quaternion (norm " +
                            std::to_string(norm) + "): it is not normalised silently");
  }
}

// vNext M3.5: `events: [{at: {state: <index>, when: after_arrival | before_departure}, kind: <k>}]`.
// Synchronisation is at a STATE only, and blocking: an event inside a segment (a fraction, a distance,
// a time) or an asynchronous one has no key here and is refused as an unknown key. The robot stops at
// an event state, so the segment that arrives there must not blend into the next one.
void parseEvents(const YAML::Node& n, PathSpec& spec)
{
  const std::string& name = spec.name;
  if (!n.IsSequence()) throw PathError(name, "events must be a list");
  if (spec.dense)
    throw PathError(name, "events on a dense path: UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL (an event is "
                          "synchronised at a state of a segment path; a dense path's samples are not "
                          "anchors): put the process steps before and after the path");
  const std::size_t last = spec.states.size() - 1;
  for (std::size_t i = 0; i < n.size(); ++i)
  {
    const std::string where = "events[" + std::to_string(i) + "]";
    const YAML::Node e = n[i];
    checkKeys(e, kEventKeys, name, where);
    if (!e["at"] || !e["kind"]) throw PathError(name, where + " needs at and kind");
    checkKeys(e["at"], kAnchorKeys, name, where + ".at");
    if (!e["at"]["state"] || !e["at"]["when"])
      throw PathError(name, where + ".at needs state (an index of states) and when");
    PathEventSpec ev;
    long long idx = -1;
    try { idx = e["at"]["state"].as<long long>(); }
    catch (const std::exception&) { throw PathError(name, where + ".at.state must be an index of states"); }
    if (idx < 0 || static_cast<std::size_t>(idx) > last)
      throw PathError(name, where + ".at.state " + std::to_string(idx) + " is not a state of the path (0.." +
                            std::to_string(last) + ")");
    ev.state = static_cast<std::size_t>(idx);
    const std::string when = text(e["at"]["when"], name, where + ".at.when");
    if (when == "after_arrival") ev.when = PathEventWhen::AFTER_ARRIVAL;
    else if (when == "before_departure") ev.when = PathEventWhen::BEFORE_DEPARTURE;
    else throw PathError(name, where + ".at.when '" + when + "' (accepted: after_arrival | before_departure)");
    if (ev.when == PathEventWhen::BEFORE_DEPARTURE && ev.state == last)
      throw PathError(name, where + ": before_departure at the last state: the path never departs from it");
    ev.kind = text(e["kind"], name, where + ".kind");
    const bool known = std::any_of(std::begin(kPathEventKinds), std::end(kPathEventKinds),
                                   [&](const char* k) { return ev.kind == k; });
    if (!known)
      throw PathError(name, where + ": unknown kind '" + ev.kind + "' (accepted: process_on, process_off, "
                            "set_value, capture, dwell, external)");
    if (ev.state > 0 && spec.segments[ev.state - 1].blend_radius_m > 0.0)
      throw PathError(name, where + ": the robot stops at state " + std::to_string(ev.state) +
                            " for this event, but segments[" + std::to_string(ev.state - 1) +
                            "] blends into the next one (blend_radius_m " +
                            std::to_string(spec.segments[ev.state - 1].blend_radius_m) + "): set it to 0");
    spec.events.push_back(ev);
    spec.event_states.push_back(ev.state);
  }
  std::sort(spec.event_states.begin(), spec.event_states.end());
  spec.event_states.erase(std::unique(spec.event_states.begin(), spec.event_states.end()), spec.event_states.end());
}

PathSpec parsePath(const std::string& name, const YAML::Node& n, const std::string& base_dir,
                   const PathRuntimeFacts& runtime)
{
  PathSpec spec;
  spec.name = name;
  checkKeys(n, kPathKeys, name, "the path");

  if (n["continuity"])
  {
    const std::string c = text(n["continuity"], name, "continuity");
    if (c == "stop") spec.continuity = PathContinuity::STOP;
    else if (c == "blend") spec.continuity = PathContinuity::BLEND;
    else throw PathError(name, "unknown continuity '" + c + "' (accepted: stop | blend)");
  }
  if (n["frame"]) spec.frame = text(n["frame"], name, "frame");
  if (n["tcp"]) spec.tcp = text(n["tcp"], name, "tcp");
  if (!spec.tcp.empty() && spec.tcp != runtime.tip_link)
    throw PathError(name, "tcp '" + spec.tcp + "' is not the runtime's tip link '" + runtime.tip_link +
                          "': UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL in M3.1 (a path runs for the "
                          "runtime's tip link; another TCP comes with the motion hardening, M3.4)");
  if (n["profile"]) spec.profile = text(n["profile"], name, "profile");
  if (n["velocity_scaling"]) spec.velocity_scaling = scaling(n["velocity_scaling"], name, "velocity_scaling");
  if (n["acceleration_scaling"])
    spec.acceleration_scaling = scaling(n["acceleration_scaling"], name, "acceleration_scaling");
  if (n["goal_tolerance"]) spec.goal_tolerance = tolerance(n["goal_tolerance"], name, "goal_tolerance");
  if (n["start_tolerance"]) spec.start_tolerance = tolerance(n["start_tolerance"], name, "start_tolerance");

  const bool has_states = n["states"] || n["segments"];
  if (has_states == static_cast<bool>(n["source"]))
    throw PathError(name, "a path is either states + segments, or a dense source");

  if (n["source"])
  {
    spec.dense = true;
    denseSource(n["source"], spec, base_dir, runtime.tip_link);
  }
  else
  {
    if (!n["states"] || !n["states"].IsSequence())
      throw PathError(name, "states must be a list");
    if (n["states"].size() < 2)
      throw PathError(name, "a path needs at least 2 states (its start and one more), got " +
                            std::to_string(n["states"].size()));
    for (std::size_t i = 0; i < n["states"].size(); ++i)
      spec.states.push_back(state(n["states"][i], name, "states[" + std::to_string(i) + "]"));
    if (!n["segments"] || !n["segments"].IsSequence())
      throw PathError(name, "segments must be a list (one per pair of consecutive states)");
    if (n["segments"].size() != spec.states.size() - 1)
      throw PathError(name, std::to_string(spec.states.size()) + " states need " +
                            std::to_string(spec.states.size() - 1) + " segments, got " +
                            std::to_string(n["segments"].size()));
    for (std::size_t i = 0; i < n["segments"].size(); ++i)
      spec.segments.push_back(segment(n["segments"][i], name, i));

    const auto& last = spec.segments.back();
    if (last.blend_radius_m > 0.0)
      throw PathError(name, "the last segment cannot blend into a next one (blend_radius_m " +
                            std::to_string(last.blend_radius_m) + "): a path ends at rest");
    const bool any_blend = std::any_of(spec.segments.begin(), spec.segments.end(),
                                       [](const PathSegmentSpec& s) { return s.blend_radius_m > 0.0; });
    if (spec.continuity == PathContinuity::STOP && any_blend)
      throw PathError(name, "a blend_radius_m on a 'continuity: stop' path: say 'continuity: blend', or "
                            "remove the radius");
    if (spec.continuity == PathContinuity::BLEND && !any_blend)
      throw PathError(name, "'continuity: blend' with no blend_radius_m > 0: the path would stop at "
                            "every state; say 'continuity: stop', or give the radii");
  }
  if (n["events"]) parseEvents(n["events"], spec);

  // the provider, and what it must provide
  if (n["provider"]) spec.provider = text(n["provider"], name, "provider");
  else
    spec.provider = defaultMotionProvider(
      spec.dense && spec.continuity == PathContinuity::BLEND ? "dense_path" : "path");
  const MotionProvider* provider = findMotionProvider(spec.provider);
  if (!provider) throw PathError(name, "unknown provider '" + spec.provider + "'");
  if (provider->stability != "stable")
    throw PathError(name, "provider '" + spec.provider + "' is " + provider->stability + ", not stable");
  spec.required = requiredCapabilities(spec);
  const auto missing = missingCapabilities(*provider, spec.required);
  if (!missing.empty())
  {
    std::string m, p;
    for (const auto& t : missing) m += (m.empty() ? "" : ", ") + t;
    for (const auto& t : provider->provides) p += (p.empty() ? "" : ", ") + t;
    throw PathError(name, "provider '" + spec.provider + "' cannot honour this path: it does not provide " +
                          m + " (it provides " + p + ")");
  }
  if (provider->route == "cartesian_path" && runtime.backend != ProcessPathBackend::CARTESIAN_WAYPOINTS)
    throw PathError(name, "provider '" + spec.provider + "' runs through the runtime's cartesian_waypoints "
                          "backend, but this runner's process_path_backend is '" + toString(runtime.backend) +
                          "' (set process_path_backend: cartesian_waypoints, or use a stop path)");
  if (provider->route != "motion_sequence" && provider->route != "cartesian_path")
    throw PathError(name, "provider '" + spec.provider + "' (route " + provider->route +
                          ") does not execute paths");

  // geometry, when it is all known now (inline poses only)
  if (!spec.dense)
  {
    const bool all_inline =
      std::none_of(spec.states.begin(), spec.states.end(), [](const PathPointRef& r) { return r.isWaypoint(); }) &&
      std::none_of(spec.segments.begin(), spec.segments.end(), [](const PathSegmentSpec& s) {
        return (s.via && s.via->isWaypoint()) || (s.center && s.center->isWaypoint());
      });
    if (all_inline)
    {
      std::vector<geometry_msgs::msg::Pose> poses;
      for (const auto& r : spec.states) poses.push_back(*r.pose);
      std::vector<std::optional<geometry_msgs::msg::Point>> aux;
      for (const auto& s : spec.segments)
      {
        if (s.via) aux.emplace_back(s.via->pose->position);
        else if (s.center) aux.emplace_back(s.center->pose->position);
        else aux.emplace_back(std::nullopt);
      }
      const std::string why = checkPathGeometry(spec, poses, aux);
      if (!why.empty()) throw PathError(name, why);
    }
  }
  else
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto& w : spec.samples.waypoints) poses.push_back(w.pose);
    const std::string why = checkPathGeometry(spec, poses, {});
    if (!why.empty()) throw PathError(name, why);
  }
  return spec;
}

Eigen::Vector3d vec(const geometry_msgs::msg::Point& p) { return {p.x, p.y, p.z}; }

double angleBetween(const geometry_msgs::msg::Quaternion& a, const geometry_msgs::msg::Quaternion& b)
{
  const Eigen::Quaterniond qa(a.w, a.x, a.y, a.z), qb(b.w, b.x, b.y, b.z);
  return qa.normalized().angularDistance(qb.normalized());
}

// sin of the angle between u and v (0 when either is degenerate)
double sinBetween(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
  const double n = u.norm() * v.norm();
  return n < 1e-12 ? 0.0 : u.cross(v).norm() / n;
}

void build(const YAML::Node& root, const std::string& base_dir, const PathRuntimeFacts& runtime,
                  std::map<std::string, PathSpec>& out)
{
  checkKeys(root, kRootKeys, "", "the file");
  if (!root["schema_version"] || root["schema_version"].as<std::string>() != std::to_string(kPathsSchemaVersion))
    throw PathError("", "schema_version must be " + std::to_string(kPathsSchemaVersion) +
                        " (this TMR reads paths.yaml schema " + std::to_string(kPathsSchemaVersion) + ")");
  if (!root["paths"] || !root["paths"].IsMap())
    throw PathError("", "'paths' must be a map of named paths");
  for (const auto& kv : root["paths"])
  {
    const std::string name = kv.first.as<std::string>();
    if (name.empty()) throw PathError("", "a path name is empty");
    if (out.count(name)) throw PathError(name, "is defined twice in the file");
    PathSpec spec;
    try { spec = parsePath(name, kv.second, base_dir, runtime); }
    catch (const PathError&) { throw; }
    catch (const std::exception& e)   // a value of the wrong type (yaml-cpp), a generator's own refusal
    {
      throw PathError(name, e.what());
    }
    out.emplace(name, std::move(spec));
  }
}
}  // namespace

std::vector<std::string> requiredCapabilities(const PathSpec& path)
{
  std::vector<std::string> req;
  auto add = [&req](const char* t) {
    if (std::find(req.begin(), req.end(), t) == req.end()) req.emplace_back(t);
  };
  if (path.dense)
  {
    if (path.continuity == PathContinuity::BLEND) add(kCapContinuousPath);
    else { add(kCapMultiSegmentPath); add(kCapCartesianLinear); }
  }
  else
  {
    add(kCapMultiSegmentPath);
    for (const auto& s : path.segments)
      add(s.type == PathSegmentType::CIRC ? kCapCartesianCircular : kCapCartesianLinear);
    if (path.continuity == PathContinuity::BLEND) add(kCapBlending);
  }
  add(kCapVelocityScaling);
  add(kCapAccelerationScaling);
  if (path.goal_tolerance) add(kCapGoalTolerance);
  return req;
}

std::string checkPathGeometry(const PathSpec& path,
                              const std::vector<geometry_msgs::msg::Pose>& states,
                              const std::vector<std::optional<geometry_msgs::msg::Point>>& aux)
{
  constexpr double kSame = 1e-6;        // m / rad: two states this close are one state
  constexpr double kAuxDistinct = 1e-4; // m: a circle's via point apart from its ends
  constexpr double kCollinear = 1e-3;   // sin(angle): below it three points define no plane
  constexpr double kRadiusMatch = 1e-3; // m: a centre equidistant from both ends
  if (states.size() < 2) return "a path needs at least 2 states";
  for (std::size_t i = 0; i + 1 < states.size(); ++i)
  {
    const double d = (vec(states[i + 1].position) - vec(states[i].position)).norm();
    if (d < kSame && angleBetween(states[i].orientation, states[i + 1].orientation) < kSame)
      return "states " + std::to_string(i) + " and " + std::to_string(i + 1) +
             " are the same pose: a zero-length segment cannot be planned";
  }
  for (std::size_t i = 0; i < path.segments.size() && i < aux.size(); ++i)
  {
    if (!aux[i]) continue;
    const auto& seg = path.segments[i];
    const Eigen::Vector3d a = vec(states[i].position), b = vec(states[i + 1].position), x = vec(*aux[i]);
    const std::string where = "segment " + std::to_string(i) + " (circ)";
    if ((b - a).norm() < kAuxDistinct)
      return where + ": its two ends are the same point; a full circle is not one circ segment";
    if (seg.via)
    {
      if ((x - a).norm() < kAuxDistinct || (x - b).norm() < kAuxDistinct)
        return where + ": the via point coincides with an end of the arc";
      if (sinBetween(x - a, b - a) < kCollinear)
        return where + ": the via point is collinear with the arc's ends: no circle passes through them";
    }
    else
    {
      const double r1 = (a - x).norm(), r2 = (b - x).norm();
      if (r1 < kAuxDistinct) return where + ": the centre coincides with the arc's start";
      if (std::abs(r1 - r2) > kRadiusMatch)
        return where + ": the centre is not equidistant from the arc's ends (" + std::to_string(r1) +
               " m vs " + std::to_string(r2) + " m)";
      if (sinBetween(a - x, b - x) < kCollinear)
        return where + ": the ends and the centre are collinear (a half circle, or none): the arc's "
               "plane is undefined; use a via point";
    }
  }
  for (std::size_t i = 0; i < path.segments.size(); ++i)
  {
    const double r = path.segments[i].blend_radius_m;
    if (r <= 0.0 || i + 2 >= states.size()) continue;
    const double in = (vec(states[i + 1].position) - vec(states[i].position)).norm();
    const double out = (vec(states[i + 2].position) - vec(states[i + 1].position)).norm();
    if (r > 0.5 * std::min(in, out))
      return "segment " + std::to_string(i) + ": blend_radius_m " + std::to_string(r) +
             " exceeds half of an adjacent segment (" + std::to_string(std::min(in, out)) +
             " m): the blend would swallow a state";
  }
  return "";
}

PathLibrary parsePathLibrary(const std::string& text, const std::string& base_dir,
                             const PathRuntimeFacts& runtime)
{
  YAML::Node root;
  try { root = YAML::Load(text); }
  catch (const std::exception& e) { throw PathError("", std::string("not YAML: ") + e.what()); }
  std::map<std::string, PathSpec> paths;
  build(root, base_dir, runtime, paths);
  return PathLibrary(std::move(paths));
}

PathLibrary loadPathLibrary(const std::string& file, const PathRuntimeFacts& runtime)
{
  YAML::Node root;
  try { root = YAML::LoadFile(file); }
  catch (const std::exception& e) { throw PathError("", "cannot read " + file + ": " + e.what()); }
  std::map<std::string, PathSpec> paths;
  build(root, std::filesystem::path(file).parent_path().string(), runtime, paths);
  return PathLibrary(std::move(paths));
}

const PathSpec* PathLibrary::find(const std::string& name) const
{
  const auto it = paths_.find(name);
  return it == paths_.end() ? nullptr : &it->second;
}

std::vector<std::string> PathLibrary::names() const
{
  std::vector<std::string> out;
  for (const auto& kv : paths_) out.push_back(kv.first);
  return out;
}

}  // namespace trainit
