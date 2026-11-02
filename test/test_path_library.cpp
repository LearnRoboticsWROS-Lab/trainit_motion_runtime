// test_path_library (vNext M3.1, ADR-0024): the canonical path model's loader, pure (no ROS graph).
// Every path that loads says what it requires of its provider; every one that cannot be executed as
// written is refused, naming the path and the problem -- nothing is coerced.
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "trainit_motion_runtime/motion_providers.hpp"
#include "trainit_motion_runtime/path_library.hpp"

using namespace trainit;

namespace
{
const PathRuntimeFacts kPilz{ProcessPathBackend::PILZ_SEQUENCE, "tcp"};
const PathRuntimeFacts kCartesian{ProcessPathBackend::CARTESIAN_WAYPOINTS, "tcp"};
const std::string kData = TRAINIT_M31_DATA_DIR;

std::string pose(double x, double y, double z)
{
  return "{pose: {position: [" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) +
         "], orientation: [0, 0, 0, 1]}}";
}

// One path named `p`, the given body (YAML flow lines of the path's map).
std::string one(const std::string& body)
{
  return "schema_version: 1\npaths:\n  p:\n" + body;
}

// W1 -LIN-> W2 -CIRC(via)-> W3 -LIN-> W4, the M3.1 example.
std::string lcl(const std::string& continuity, double b0, double b1, double b2,
                const std::string& extra = "")
{
  return one("    continuity: " + continuity + "\n" + extra +
             "    states: [" + pose(0.5, 0.0, 0.3) + ", " + pose(0.5, 0.1, 0.3) + ", " + pose(0.5, 0.2, 0.2) +
             ", " + pose(0.4, 0.2, 0.2) + "]\n"
             "    segments:\n"
             "      - {type: lin, blend_radius_m: " + std::to_string(b0) + "}\n"
             "      - {type: circ, via: {position: [0.5, 0.17, 0.27]}, blend_radius_m: " + std::to_string(b1) + "}\n"
             "      - {type: lin, blend_radius_m: " + std::to_string(b2) + "}\n");
}

PathLibrary parse(const std::string& text, const PathRuntimeFacts& rt = kPilz)
{
  return parsePathLibrary(text, kData, rt);
}

// The path `p` of a one-path text, copied out of its (temporary) library.
std::optional<PathSpec> only(const std::string& text, const PathRuntimeFacts& rt = kPilz)
{
  const PathLibrary lib = parse(text, rt);
  const PathSpec* p = lib.find("p");
  return p ? std::optional<PathSpec>(*p) : std::nullopt;
}

// The refusal message, or "" when the text loads.
std::string refusal(const std::string& text, const PathRuntimeFacts& rt = kPilz)
{
  try { parse(text, rt); return ""; }
  catch (const PathError& e) { return e.what(); }
}

#define EXPECT_REFUSED(text, needle)                                                   \
  do {                                                                                \
    const std::string why = refusal(text);                                            \
    EXPECT_NE(why.find(needle), std::string::npos) << "expected a refusal naming '"   \
      << (needle) << "', got: '" << why << "'";                                       \
  } while (0)

using Tokens = std::vector<std::string>;
}  // namespace

// --- what loads -----------------------------------------------------------------------------------

TEST(PathLibrary, ALinCircLinBlendedPathLoadsAndRequiresWhatItUses)
{
  const PathLibrary lib = parse(lcl("blend", 0.02, 0.02, 0.0));
  const PathSpec* p = lib.find("p");
  ASSERT_NE(p, nullptr);
  EXPECT_EQ(p->provider, kProviderPilzSequence);   // provider_defaults.path
  EXPECT_EQ(p->continuity, PathContinuity::BLEND);
  ASSERT_EQ(p->states.size(), 4u);
  ASSERT_EQ(p->segments.size(), 3u);
  EXPECT_EQ(p->segments[1].type, PathSegmentType::CIRC);
  ASSERT_TRUE(p->segments[1].via.has_value());
  EXPECT_DOUBLE_EQ(p->segments[0].blend_radius_m, 0.02);
  EXPECT_EQ(p->required, (Tokens{kCapMultiSegmentPath, kCapCartesianLinear, kCapCartesianCircular,
                                 kCapBlending, kCapVelocityScaling, kCapAccelerationScaling}));
}

TEST(PathLibrary, AStoppedPathRequiresNoBlending)
{
  const std::optional<PathSpec> p = only(lcl("stop", 0, 0, 0));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->continuity, PathContinuity::STOP);
  EXPECT_EQ(p->required, (Tokens{kCapMultiSegmentPath, kCapCartesianLinear, kCapCartesianCircular,
                                 kCapVelocityScaling, kCapAccelerationScaling}));
}

TEST(PathLibrary, ALinOnlyPathAndACircOnlyPathLoad)
{
  const std::optional<PathSpec> lin = only(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) +
                                  "]\n    segments: [{type: lin}]\n"));
  ASSERT_TRUE(lin.has_value());
  EXPECT_EQ(lin->required, (Tokens{kCapMultiSegmentPath, kCapCartesianLinear, kCapVelocityScaling,
                                   kCapAccelerationScaling}));
  const std::optional<PathSpec> circ = only(one("    states: [" + pose(0.5, 0.1, 0.3) + ", " + pose(0.5, 0.2, 0.2) +
                                   "]\n    segments: [{type: circ, center: {position: [0.5, 0.1, 0.2]}}]\n"));
  ASSERT_TRUE(circ.has_value());
  ASSERT_TRUE(circ->segments[0].center.has_value());
  EXPECT_EQ(circ->required, (Tokens{kCapMultiSegmentPath, kCapCartesianCircular, kCapVelocityScaling,
                                    kCapAccelerationScaling}));
}

TEST(PathLibrary, AGoalToleranceIsARequirement)
{
  const std::optional<PathSpec> p = only(lcl("stop", 0, 0, 0, "    goal_tolerance: {position_m: 0.001, orientation_rad: 0.01}\n"));
  ASSERT_TRUE(p.has_value());
  ASSERT_TRUE(p->goal_tolerance.has_value());
  EXPECT_EQ(p->required.back(), kCapGoalTolerance);
}

TEST(PathLibrary, WaypointStatesLoadAndAreResolvedOnlyAtTickTime)
{
  const std::optional<PathSpec> p = only(one("    states: [{waypoint: a}, {waypoint: b}, {waypoint: c}]\n"
                                "    segments: [{type: lin}, {type: circ, via: {waypoint: v}}]\n"));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->states[0].waypoint, "a");
  EXPECT_FALSE(p->states[0].pose.has_value());
  EXPECT_EQ(p->segments[1].via->waypoint, "v");
}

TEST(PathLibrary, TheCommittedLibraryLoadsWithRelativeDenseSources)
{
  const PathLibrary lib = loadPathLibrary(kData + "/paths.yaml", kPilz);
  EXPECT_EQ(lib.names(), (std::vector<std::string>{"by_waypoints", "circ_center", "dense_stop", "dense_yaml_stop",
                                                   "elsewhere", "in_tool_frame", "lcl_blend", "lcl_stop"}));
  const PathSpec* dense = lib.find("dense_stop");
  ASSERT_NE(dense, nullptr);
  EXPECT_TRUE(dense->dense);
  EXPECT_EQ(dense->samples.waypoints.size(), 4u);
  EXPECT_EQ(dense->provider, kProviderPilzSequence);
  EXPECT_EQ(dense->required, (Tokens{kCapMultiSegmentPath, kCapCartesianLinear, kCapVelocityScaling,
                                     kCapAccelerationScaling}));
  EXPECT_EQ(lib.find("dense_yaml_stop")->samples.waypoints.size(), 3u);
}

TEST(PathLibrary, AContinuousDensePathNeedsTheCartesianBackend)
{
  const std::string text = one("    continuity: blend\n    source: {file: bead.csv, orientation: [0, 0, 0, 1]}\n");
  const std::optional<PathSpec> p = only(text, kCartesian);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->provider, kProviderCartesianWaypoints);   // provider_defaults.dense_path
  EXPECT_EQ(p->required, (Tokens{kCapContinuousPath, kCapVelocityScaling, kCapAccelerationScaling}));
  const std::string why = refusal(text, kPilz);
  EXPECT_NE(why.find("process_path_backend is 'pilz_sequence'"), std::string::npos) << why;
}

TEST(PathLibrary, GeneratorsAreDenseSources)
{
  const std::string o = "orientation: [0, 0, 0, 1]";
  EXPECT_EQ(parse(one("    source: {" + o + ", generator: {type: polyline, points: [[0.5, 0, 0.3], [0.5, 0.1, 0.3], [0.4, 0.1, 0.3]]}}\n"))
              .find("p")->samples.waypoints.size(), 3u);
  EXPECT_EQ(parse(one("    source: {" + o + ", generator: {type: circle, center: [0.5, 0, 0.3], radius: 0.05, segments: 12}}\n"))
              .find("p")->samples.waypoints.size(), 13u);
  EXPECT_GT(parse(one("    source: {" + o + ", generator: {type: raster, center: [0.5, 0, 0.3], size_x: 0.1, size_y: 0.1, rows: 3}}\n"))
              .find("p")->samples.waypoints.size(), 2u);
  EXPECT_EQ(parse(one("    source: {" + o + ", generator: {type: rectangle, center: [0.5, 0, 0.3], size_x: 0.1, size_y: 0.05}}\n"))
              .find("p")->samples.waypoints.size(), 5u);
}

// --- what is refused, by name -----------------------------------------------------------------------

TEST(PathLibraryRefusal, TooFewStatesAndAWrongSegmentCount)
{
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + "]\n    segments: []\n"), "at least 2 states");
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) +
                     "]\n    segments: [{type: lin}, {type: lin}]\n"), "2 states need 1 segments, got 2");
  EXPECT_REFUSED(one("    continuity: stop\n"), "either states + segments, or a dense source");
}

TEST(PathLibraryRefusal, CircGeometryMustBeDefined)
{
  const std::string two = "    states: [" + pose(0.5, 0.1, 0.3) + ", " + pose(0.5, 0.2, 0.2) + "]\n";
  EXPECT_REFUSED(one(two + "    segments: [{type: circ}]\n"), "exactly one of via or center");
  EXPECT_REFUSED(one(two + "    segments: [{type: circ, via: {position: [0.5, 0.17, 0.27]}, center: {position: [0.5, 0.1, 0.2]}}]\n"),
                 "exactly one of via or center");
  EXPECT_REFUSED(one(two + "    segments: [{type: lin, via: {position: [0.5, 0.17, 0.27]}}]\n"), "a lin segment has no via");
  // collinear via, a via on an end, a centre not equidistant, a half circle about its centre
  EXPECT_REFUSED(one(two + "    segments: [{type: circ, via: {position: [0.5, 0.15, 0.25]}}]\n"), "collinear");
  EXPECT_REFUSED(one(two + "    segments: [{type: circ, via: {position: [0.5, 0.2, 0.2]}}]\n"), "coincides with an end");
  EXPECT_REFUSED(one(two + "    segments: [{type: circ, center: {position: [0.5, 0.1, 0.1]}}]\n"), "not equidistant");
  EXPECT_REFUSED(one(two + "    segments: [{type: circ, center: {position: [0.5, 0.15, 0.25]}}]\n"), "collinear");
}

TEST(PathLibraryRefusal, BlendPlacement)
{
  EXPECT_REFUSED(lcl("blend", 0.02, 0.02, 0.01), "the last segment cannot blend");
  EXPECT_REFUSED(lcl("stop", 0.02, 0.0, 0.0), "a blend_radius_m on a 'continuity: stop' path");
  EXPECT_REFUSED(lcl("blend", 0.0, 0.0, 0.0), "'continuity: blend' with no blend_radius_m > 0");
  EXPECT_REFUSED(lcl("blend", -0.01, 0.02, 0.0), "blend_radius_m must be >= 0");
  EXPECT_REFUSED(lcl("blend", 0.06, 0.0, 0.0), "exceeds half of an adjacent segment");
}

TEST(PathLibraryRefusal, APtpSegmentInsideAPathIsReserved)
{
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) + "]\n    segments: [{type: ptp}]\n"),
                 "UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL");
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) + "]\n    segments: [{type: spline}]\n"),
                 "unknown segment type 'spline'");
}

TEST(PathLibraryRefusal, ProviderCapabilities)
{
  // a segment path on the continuous-path provider: it is not a multi-segment provider
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    provider: cartesian_waypoints\n"),
                 "provider 'cartesian_waypoints' cannot honour this path: it does not provide multi_segment_path");
  // a continuous dense path on the Pilz sequence: it is not continuous
  EXPECT_REFUSED(one("    continuity: blend\n    provider: pilz_sequence\n    source: {file: bead.csv, orientation: [0, 0, 0, 1]}\n"),
                 "does not provide continuous_path");
  // a goal tolerance the continuous provider does not honour
  {
    const std::string why = refusal(one("    continuity: blend\n    goal_tolerance: {position_m: 0.001, orientation_rad: 0.01}\n"
                                        "    source: {file: bead.csv, orientation: [0, 0, 0, 1]}\n"), kCartesian);
    EXPECT_NE(why.find("does not provide goal_tolerance"), std::string::npos) << why;
  }
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    provider: cumotion\n"), "unknown provider 'cumotion'");
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    provider: pilz_lin\n"), "does not provide multi_segment_path");
}

TEST(PathLibraryRefusal, ATcpOtherThanTheTipIsUnsupportedInM31)
{
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    tcp: suction_link\n"), "UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL");
  EXPECT_EQ(refusal(lcl("stop", 0, 0, 0, "    tcp: tcp\n")), "");
}

TEST(PathLibraryRefusal, ValuesAndKeys)
{
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    velocity_scaling: 1.5\n"), "velocity_scaling must be in (0, 1]");
  EXPECT_REFUSED(lcl("stop", 0, 0, 0, "    jerk: 3\n"), "unknown key 'jerk' in the path");
  EXPECT_REFUSED(one("    continuity: smooth\n    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) +
                     "]\n    segments: [{type: lin}]\n"), "unknown continuity 'smooth'");
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) +
                     "]\n    segments: [{type: lin, duration_s: 2}]\n"), "unknown key 'duration_s' in segments[0]");
  EXPECT_REFUSED(one("    states: [{pose: {position: [0.5, 0, 0.3], orientation: [0, 0, 0, 2]}}, " + pose(0.5, 0.1, 0.3) +
                     "]\n    segments: [{type: lin}]\n"), "not a unit quaternion");
  EXPECT_REFUSED(one("    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0, 0.3) + "]\n    segments: [{type: lin}]\n"),
                 "the same pose");
  EXPECT_REFUSED("schema_version: 2\npaths: {}\n", "schema_version must be 1");
  EXPECT_REFUSED("schema_version: 1\npaths: {}\ntasks: {}\n", "unknown key 'tasks' in the file");
}

TEST(PathLibraryRefusal, DenseSourcesCarryOnlyWhatThePathModelHonours)
{
  EXPECT_REFUSED(one("    source: {file: bead_speeds.csv, orientation: [0, 0, 0, 1]}\n"), "6 columns");
  EXPECT_REFUSED(one("    source: {file: bead_speeds.yaml}\n"), "unknown key 'velocity'");
  EXPECT_REFUSED(one("    source: {file: bead.csv}\n"), "source.orientation is required");
  EXPECT_REFUSED(one("    source: {file: bead.yaml, orientation: [0, 0, 0, 1]}\n"), "a YAML path file carries its own");
  EXPECT_REFUSED(one("    source: {file: missing.csv, orientation: [0, 0, 0, 1]}\n"), "cannot open");
  EXPECT_REFUSED(one("    source: {orientation: [0, 0, 0, 1], generator: {type: circle, center: [0.5, 0, 0.3], radius: 0.05, rows: 3}}\n"),
                 "'rows' is not a parameter of a circle generator");
  EXPECT_REFUSED(one("    source: {orientation: [0, 0, 0, 1], generator: {type: spline, points: []}}\n"),
                 "unknown generator type 'spline'");
}

TEST(PathGeometry, TheCheckerSeesResolvedPoses)
{
  const std::optional<PathSpec> p = only(one("    continuity: blend\n    states: [{waypoint: a}, {waypoint: b}, {waypoint: c}]\n"
                                "    segments: [{type: lin, blend_radius_m: 0.03}, {type: lin}]\n"));
  ASSERT_TRUE(p.has_value());
  auto at = [](double x, double y, double z) {
    geometry_msgs::msg::Pose q; q.position.x = x; q.position.y = y; q.position.z = z; q.orientation.w = 1; return q;
  };
  EXPECT_EQ(checkPathGeometry(*p, {at(0, 0, 0), at(0.1, 0, 0), at(0.2, 0, 0)}, {std::nullopt, std::nullopt}), "");
  EXPECT_NE(checkPathGeometry(*p, {at(0, 0, 0), at(0.04, 0, 0), at(0.2, 0, 0)}, {std::nullopt, std::nullopt})
              .find("exceeds half"), std::string::npos);
  // a pure reorientation is a segment, not a duplicate
  geometry_msgs::msg::Pose turned = at(0.1, 0, 0);
  turned.orientation.z = 0.7071068; turned.orientation.w = 0.7071068;
  const std::optional<PathSpec> stop = only(one("    states: [{waypoint: a}, {waypoint: b}]\n    segments: [{type: lin}]\n"));
  EXPECT_EQ(checkPathGeometry(*stop, {at(0.1, 0, 0), turned}, {std::nullopt}), "");
}

// --- the review of M3.1: nothing is dropped or coerced -----------------------------------------------

TEST(PathLibraryRefusal, ARepeatedPathOrKeyIsRefused)
{
  const std::string path = "    states: [" + pose(0.5, 0, 0.3) + ", " + pose(0.5, 0.1, 0.3) + "]\n    segments: [{type: lin}]\n";
  EXPECT_REFUSED("schema_version: 1\npaths:\n  p:\n" + path + "  p:\n" + path, "defined twice");
  EXPECT_REFUSED(one("    profile: approach\n    profile: process\n" + path), "key 'profile' is given twice");
}

TEST(PathLibraryRefusal, AValueOfTheWrongTypeNamesThePath)
{
  EXPECT_REFUSED(one("    source: {orientation: [0, 0, 0, 1], generator: {type: polyline, closed: maybe, "
                     "points: [[0.5, 0, 0.3], [0.5, 0.1, 0.3]]}}\n"), "path 'p'");
}

TEST(PathLibrary, ADenseFileForTheTipLinkLoadsWithoutAPathTcp)
{
  const auto p = only(one("    source: {file: bead_tip.yaml}\n"));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->samples.tcp_link, "");
  const std::string why = refusal(one("    source: {file: bead_tip.yaml}\n"),
                                  PathRuntimeFacts{ProcessPathBackend::PILZ_SEQUENCE, "suction_link"});
  EXPECT_NE(why.find("is not the runtime's tip link 'suction_link'"), std::string::npos) << why;
}

// --- vNext M3.5: process events, anchored at states ----------------------------------------------

namespace
{
// a stopped 4-state lin path with the given `events:` lines
std::string evented(const std::string& events, const std::string& continuity = "stop", double b0 = 0.0)
{
  return one("    continuity: " + continuity + "\n" +
             "    states: [" + pose(0.5, 0.0, 0.3) + ", " + pose(0.5, 0.1, 0.3) + ", " + pose(0.5, 0.2, 0.3) +
             ", " + pose(0.5, 0.3, 0.3) + "]\n"
             "    segments:\n"
             "      - {type: lin, blend_radius_m: " + std::to_string(b0) + "}\n"
             "      - {type: lin}\n"
             "      - {type: lin}\n"
             "    events:\n" + events);
}
}  // namespace

TEST(PathLibraryEvents, EventsLoadWithTheirStatesSortedAndUnique)
{
  const auto p = only(evented("      - {at: {state: 2, when: after_arrival}, kind: capture}\n"
                              "      - {at: {state: 1, when: after_arrival}, kind: capture}\n"
                              "      - {at: {state: 1, when: before_departure}, kind: process_on}\n"
                              "      - {at: {state: 0, when: before_departure}, kind: dwell}\n"
                              "      - {at: {state: 3, when: after_arrival}, kind: set_value}\n"
                              "      - {at: {state: 3, when: after_arrival}, kind: external}\n"
                              "      - {at: {state: 3, when: after_arrival}, kind: process_off}\n"));
  ASSERT_TRUE(p);
  ASSERT_EQ(p->events.size(), 7u);
  EXPECT_EQ(p->events[0].state, 2u);                      // file order is kept
  EXPECT_EQ(p->events[2].when, PathEventWhen::BEFORE_DEPARTURE);
  EXPECT_EQ(p->event_states, (std::vector<std::size_t>{0, 1, 2, 3}));
  EXPECT_TRUE(p->isEventState(1));
  // an event changes nothing of what the path requires of its provider
  std::string text = evented("");
  text = text.substr(0, text.find("    events:"));
  const auto plain = only(text);
  ASSERT_TRUE(plain);
  EXPECT_TRUE(plain->events.empty());
  EXPECT_EQ(p->required, plain->required);
}

TEST(PathLibraryEvents, APathWithoutEventsHasNoEventState)
{
  const auto p = only(lcl("blend", 0.02, 0.02, 0.0));
  ASSERT_TRUE(p);
  EXPECT_TRUE(p->events.empty());
  EXPECT_TRUE(p->event_states.empty());
}

TEST(PathLibraryEvents, AnEventTheModelCannotSynchroniseIsRefused)
{
  // the anchor is a state; anything inside a segment, asynchronous or unknown is not a key of the model
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: after_arrival, fraction: 0.5}, kind: capture}\n"),
                 "unknown key 'fraction' in events[0].at");
  EXPECT_REFUSED(evented("      - {at: {segment: 1, when: end}, kind: capture}\n"), "unknown key 'segment'");
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: after_arrival}, kind: capture, sync: async}\n"),
                 "unknown key 'sync' in events[0]");
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: during}, kind: capture}\n"),
                 "events[0].at.when 'during' (accepted: after_arrival | before_departure)");
  EXPECT_REFUSED(evented("      - {at: {state: 4, when: after_arrival}, kind: capture}\n"),
                 "events[0].at.state 4 is not a state of the path (0..3)");
  EXPECT_REFUSED(evented("      - {at: {state: 1.5, when: after_arrival}, kind: capture}\n"),
                 "events[0].at.state must be an index of states");
  EXPECT_REFUSED(evented("      - {at: {state: -1, when: after_arrival}, kind: capture}\n"),
                 "is not a state of the path");
  EXPECT_REFUSED(evented("      - {at: {state: 1}, kind: capture}\n"), "events[0].at needs state");
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: after_arrival}}\n"), "events[0] needs at and kind");
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: after_arrival}, kind: glue_on}\n"),
                 "unknown kind 'glue_on'");
  EXPECT_REFUSED(evented("      - {at: {state: 3, when: before_departure}, kind: process_off}\n"),
                 "before_departure at the last state");
  EXPECT_REFUSED(evented("  oops\n"), "events must be a list");
}

TEST(PathLibraryEvents, TheRobotStopsAtAnEventStateSoNothingBlendsIntoIt)
{
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: after_arrival}, kind: capture}\n", "blend", 0.02),
                 "the robot stops at state 1 for this event, but segments[0] blends into the next one");
  // before departure too: the robot is at rest there
  EXPECT_REFUSED(evented("      - {at: {state: 1, when: before_departure}, kind: process_on}\n", "blend", 0.02),
                 "segments[0] blends");
  // an event elsewhere on the same blended path is fine
  const auto p = only(evented("      - {at: {state: 2, when: after_arrival}, kind: capture}\n", "blend", 0.02));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->event_states, (std::vector<std::size_t>{2}));
}

TEST(PathLibraryEvents, ADensePathHasNoEventAnchors)
{
  EXPECT_REFUSED(one("    continuity: stop\n    source: {file: bead.csv, orientation: [0, 0, 0, 1]}\n"
                     "    events:\n      - {at: {state: 0, when: before_departure}, kind: process_on}\n"),
                 "events on a dense path: UNSUPPORTED_BY_CURRENT_CANONICAL_MODEL");
}
