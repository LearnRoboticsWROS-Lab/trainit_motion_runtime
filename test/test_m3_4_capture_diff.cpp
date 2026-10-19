// test_m3_4_capture_diff (vNext M3.4, ADR-0015 amendment M3.4): the BEFORE -> AFTER difference of
// every capture family M3.4 moves, EXACTLY. Pure: no ROS graph; it reads the committed files.
//
// Each family is split into its "=== scenario" blocks. The header and every block not listed below
// must be byte-identical BEFORE and AFTER; every listed block must differ, and its AFTER must
// contain the facts that classify the change (and its BEFORE the facts it replaced). The BEFORE
// files are pinned by sha256 (they are never re-frozen); so are the reviewed AFTER files.
//
//   new family (test_m3_4_execution, test_m3_4_runner_stop): test/data/m3_4/{before,after}/
//   legacy families (frozen on TMR 1.8.0, test/data/m2_5/) whose M3.4-approved scenarios moved:
//     their AFTER is test/data/m3_4/legacy_after/, which the M2.5 binaries compare with
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "m2_5/capture.hpp"   // m25::sha256

namespace
{
const std::string kM25 = TRAINIT_M25_DATA_DIR;
const std::string kM34 = TRAINIT_M34_DATA_DIR;

std::string slurp(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in.is_open()) << "missing " << path;
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

struct Split
{
  std::string header;
  std::vector<std::string> order;
  std::map<std::string, std::string> blocks;
};

Split split(const std::string& text)
{
  Split out;
  const std::string tag = "=== scenario ";
  std::size_t at = text.find(tag);
  out.header = text.substr(0, at);
  while (at != std::string::npos)
  {
    std::size_t next = text.find("\n" + tag, at);
    next = next == std::string::npos ? text.size() : next + 1;
    const std::string block = text.substr(at, next - at);
    const std::string id = block.substr(tag.size(), block.find('\n') - tag.size());
    out.order.push_back(id);
    out.blocks[id] = block;
    at = next < text.size() ? next : std::string::npos;
  }
  return out;
}

// a changed scenario: what its BEFORE showed, what its AFTER must show (substrings), and why
struct Change
{
  std::string id;
  std::vector<std::string> before_has;
  std::vector<std::string> after_has;
  std::string classification;
  std::vector<std::string> after_lacks{};
};

void checkFamily(const std::string& before_path, const std::string& after_path, const std::string& before_sha,
                 const std::string& after_sha, const std::vector<Change>& changes)
{
  const std::string before = slurp(before_path), after = slurp(after_path);
  EXPECT_EQ(m25::sha256(before), before_sha) << before_path << ": a BEFORE record is never re-frozen";
  EXPECT_EQ(m25::sha256(after), after_sha) << after_path << ": the reviewed AFTER changed";
  const Split b = split(before), a = split(after);
  EXPECT_EQ(b.header, a.header) << after_path;
  EXPECT_EQ(b.order, a.order) << after_path << ": the scenarios and their order are the BEFORE's";
  std::set<std::string> changed;
  for (const auto& c : changes)
  {
    changed.insert(c.id);
    ASSERT_TRUE(b.blocks.count(c.id) && a.blocks.count(c.id)) << c.id;
    EXPECT_NE(b.blocks.at(c.id), a.blocks.at(c.id)) << c.id << " should differ: " << c.classification;
    for (const auto& s : c.before_has)
      EXPECT_NE(b.blocks.at(c.id).find(s), std::string::npos) << c.id << " BEFORE lacks: " << s;
    for (const auto& s : c.after_has)
      EXPECT_NE(a.blocks.at(c.id).find(s), std::string::npos) << c.id << " AFTER lacks: " << s;
    for (const auto& s : c.after_lacks)
      EXPECT_EQ(a.blocks.at(c.id).find(s), std::string::npos) << c.id << " AFTER still has: " << s;
  }
  for (const auto& id : b.order)
  {
    if (changed.count(id) || !a.blocks.count(id)) continue;
    EXPECT_EQ(b.blocks.at(id), a.blocks.at(id)) << after_path << ": scenario " << id
                                                << " is not an approved M3.4 change and must be byte-identical";
  }
}

// the classifications (ADR-0015 amendment M3.4)
const char* kFailurePath = "FAILURE-PATH BEHAVIOUR CHANGE";
const char* kDataGated = "DATA-GATED BEHAVIOUR CHANGE";
const char* kBugFix = "BUG FIX";

// sha256 of every file compared here: the BEFOREs (never re-frozen) and the reviewed AFTERs
const std::map<std::string, std::string> kSha = {
#include "data/m3_4/sha256.inc"
};
std::string sha(const std::string& key)
{
  const auto it = kSha.find(key);
  return it == kSha.end() ? "(no pinned sha256 for " + key + ")" : it->second;
}

void checkLegacy(const std::string& before_dir, const std::string& family, const std::vector<Change>& changes)
{
  checkFamily(before_dir + "/" + family + ".txt", kM34 + "/legacy_after/" + family + ".txt", sha(family + "_before"),
              sha(family + "_after"), changes);
}
}  // namespace

TEST(M34CaptureDiff, the_new_family_differs_exactly_where_m3_4_changed_it)
{
  const std::string pos_tol = "- 0.0020000000000000000";
  const std::string ori_tol = "absolute_x_axis_tolerance: 0.020000000000000000";
  checkFamily(kM34 + "/before/m3_4_execution.txt", kM34 + "/after/m3_4_execution.txt",
              sha("m3_4_execution_before"), sha("m3_4_execution_after"),
              {{"TOL-01", {"- 0.00010000000000000000"}, {pos_tol, ori_tol}, kDataGated},
               {"TOL-02", {"- 0.00010000000000000000"}, {pos_tol, ori_tol}, kDataGated},
               {"TOL-03", {"- 0.00010000000000000000"}, {pos_tol}, kDataGated},
               {"TOL-04", {}, {pos_tol}, kDataGated},
               {"TOL-07", {"target=joint"}, {"UNSUPPORTED_TOLERANCE during request", "counts: none"}, kFailurePath},
               {"TOL-08", {"bt: SUCCESS"}, {"UNSUPPORTED_TOLERANCE during request", "counts: none"}, kFailurePath},
               {"PLN-01", {"pipeline=ompl"}, {"UNKNOWN_PLANNER during request", "counts: none"}, kFailurePath},
               {"PLN-02", {"planner=LIN"}, {"UNKNOWN_PLANNER during request", "counts: none"}, kFailurePath},
               {"PLN-03", {"pipeline=ompl"}, {"bt: FAILURE", "counts: none"}, kFailurePath},
               {"CIRC-02", {"MOTION_FAULT: EXCEPTION"}, {"path_constraints=center", "bt: SUCCESS"}, kBugFix},
               {"CIRC-03", {"bad optional access"}, {"path_constraints=center", "bt: SUCCESS"}, kBugFix},
               {"CIRC-04", {"MOTION_FAULT: EXCEPTION"}, {"INVALID_POSE during request"}, kFailurePath},
               {"CIRC-05", {"bt: SUCCESS"}, {"INVALID_POSE during request", "counts: none"}, kFailurePath},
               {"CIRC-06", {"MOTION_FAULT: EXCEPTION"}, {"CIRCLE_GEOMETRY during request"}, kFailurePath},
               {"CIRC-07", {"bt: SUCCESS"}, {"CIRCLE_GEOMETRY during request", "counts: none"}, kFailurePath},
               {"TIP-01", {"frames=tcp@base_link"}, {"frames=suction_link@base_link", "link_name: \"suction_link\""},
                kDataGated},
               {"RST-01", {"frames=tcp@world"}, {"frames=tcp@base_link"}, kBugFix},
               {"RST-02", {"after: frame=world eef=l3"}, {"run 1 success=1 | after: frame=base_link eef=tcp",
                "run 2 (fraction 0.5) success=0 | after: frame=base_link eef=tcp"}, kBugFix},
               {"ST-03", {"bt: SUCCESS", "cur=Pose(0,0,0 | 0,0,0,1)"}, {"bt: FAILURE", "cur=(none)"}, kFailurePath},
               {"ST-04", {"bt: SUCCESS"}, {"bt: FAILURE"}, kFailurePath},
               {"ST-05", {"bt: SUCCESS"}, {"bt: FAILURE"}, kFailurePath},
               {"ST-06", {"NOT_AT_PATH_START"}, {"STATE_UNAVAILABLE during request"}, kFailurePath},
               {"TO-01", {"counts: /sequence_move_group=1"}, {"MoveGroupSequence.cancel -> ACCEPT",
                "RESULT_TIMEOUT during plan_and_execute"}, kFailurePath},
               {"ST-08", {"NOT_AT_PATH_START"}, {"bt: SUCCESS", "items=3"}, kFailurePath},
               {"ST-07", {"/execute_trajectory=1", "bt: SUCCESS"}, {"bt: FAILURE", "counts: none"}, kFailurePath},
               {"CIRC-08", {"bt: SUCCESS"}, {"bt: FAILURE", "counts: none"}, kFailurePath},
               {"CIRC-09", {"result: success=1"}, {"success=0 cause=CIRCLE_GEOMETRY", "counts: none"}, kFailurePath},
               {"ALS-01", {"vel=0.5 acc=0.5 time=5 attempts=10"}, {"vel=0.20000000000000001 acc=0.20000000000000001 time=3"},
                kBugFix},
               {"ALS-02", {"vel=0.5 acc=0.5 time=5 attempts=10"}, {"vel=0.20000000000000001 acc=0.20000000000000001 time=3"},
                kBugFix}});
}

TEST(M34CaptureDiff, the_runner_sends_the_stop_on_sigint)
{
  checkFamily(kM34 + "/before/m3_4_runner_stop.txt", kM34 + "/after/m3_4_runner_stop.txt",
              sha("m3_4_runner_stop_before"), sha("m3_4_runner_stop_after"),
              {{"STOP-RUNNER-01", {"topics heard (sorted):\n(none)"},
                {"/trajectory_execution_event String: stop", "killed by signal 9"}, kFailurePath}});
}

// --- the legacy families (BEFORE: TMR 1.8.0, test/data/m2_5/; the namespace pair: M2.7, m2_7_ns/) ---------

TEST(M34CaptureDiff, the_legacy_families_moved_only_in_the_approved_scenarios)
{
  const std::string center = "path_constraints=center";
  const std::vector<std::string> refused = {"bt: FAILURE", "counts: none"};
  checkLegacy(kM25, "planned_waypoint",
              {{"WP-LIN-04", {"vel=0.5 acc=0.5 time=5 attempts=10"},
                {"vel=0.20000000000000001 acc=0.20000000000000001 time=3"}, kBugFix},
               {"WP-CIRC-04", {"bt: FAILURE"}, {center, "bt: SUCCESS"}, kBugFix},
               {"WP-FREE-08", {"pipeline=ompl", "bt: SUCCESS"}, refused, kFailurePath}});
  checkLegacy(kM25, "planned_lin_circ",
              {{"CIRC-02", {"frames=tcp@base_link"}, {"frames=l3@base_link"}, kDataGated},
               {"CIRC-03", {"bad optional access"}, {center, "bt: SUCCESS"}, kBugFix}});
  checkLegacy(kM25, "planner_identity",
              {{"T3-02 free tcp planner='bogus'", {"bt: SUCCESS"}, refused, kFailurePath},
               {"T3-02 ptp joint planner='bogus'", {"bt: SUCCESS"}, refused, kFailurePath}});
  checkLegacy(kM25, "sequences", {{"T5-06", {"frames=tcp@world"}, {"frames=tcp@base_link"}, kBugFix,
                                   {"frames=tcp@world"}}});
  checkLegacy(kM25, "task_runner", {{"TASK-02", {"bad optional access"}, {center}, kBugFix,
                                     {"bad optional access"}}});
  checkLegacy(kM25, "task_runner_2",
              {{"TASK-03a", {"frames=tcp@world"}, {"frames=tcp@base_link"}, kBugFix, {"frames=tcp@world"}},
               {"TASK-03b", {"frames=tcp@world"}, {"frames=tcp@base_link"}, kBugFix, {"frames=tcp@world"}}});
  checkLegacy(kM25, "state_reads", {{"READ-04", {"cur=Pose(0,0,0 | 0,0,0,1)"}, {"bt: FAILURE"}, kFailurePath,
                                     {"Pose(0,0,0 | 0,0,0,1)"}}});
  checkLegacy(kM25, "servo_track_2", {{"SERVO-14", {"start_servo Trigger.call"}, {"twists: none", "counts: none"},
                                       kFailurePath, {"Trigger.call"}}});
  for (const char* ns : {"namespace", "namespace_remap"})
    checkLegacy(TRAINIT_M27_NS_DATA_DIR, ns, {{"NS-01c", {"cur=Pose(0,0,0 | 0,0,0,1)"}, {"bt: FAILURE"},
                                               kFailurePath, {"cur=Pose(0,0,0 | 0,0,0,1)"}}});
}
