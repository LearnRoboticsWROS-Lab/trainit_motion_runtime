// vNext M2.7 (owner Q8.2): the canonical bin-picking loop, ticked as XML with the REAL MoveWaypoint
// against the M2.5 fake move_group and the M2.6 fake end effector (test/m2_7/mf_bin_picking_cell.hpp).
// What a MOTION failure does to the canonical cycle:
//
//   a target the robot cannot reach (class A: no IK, no plan) must keep the existing strategy -- the
//   next candidate, then the stall branch -- while a failure that is not the target's (class B: the
//   motion infrastructure, a controller, the robot's start state) must never read as "try the next
//   candidate" or "bin empty" (owner Q8.2); a failure while an object is held is HOLD (Q8.1); a
//   detector that cannot be read is a FAULT, never an empty bin (Q9).
//
// BEFORE_the_m2_6_tree (C0 on production adf94b0; C1 on every later one) ticks the M2.6 tree
// (test/data/m2_6_contract/bin_picking_canonical.xml, TSA's generation for its pinned canonical
// bin-picking project, sha256 25fb4e25..., never edited) and compares the capture with
// test/data/m2_7_mf/bin_picking_motion_before.txt, frozen on production adf94b0 (TMR 2.0.0) BEFORE
// vNext M2.7 touched a production line: a tree that does not remap MoveWaypoint's `fault` behaves
// exactly as before, byte for byte. AFTER_the_m2_7_tree (C2) ticks the M2.7 tree
// (test/data/m2_7_mf/bin_picking_canonical.xml: the same tree with fault="{eef_fault}" on the five
// MoveWaypoints inside the cycle's absorbing Fallback, sha256 93c37df9...) and compares the capture with
// test/data/m2_7_mf/bin_picking_motion_after.txt, frozen on the production of the commit that added it.
// the_BEFORE_to_AFTER_diff_is_exactly_the_intended_lines reads the two committed captures (no ROS):
// the difference is exactly the intended one (test/data/m2_7_mf/README.md). Domain 97 (the M2.6
// in-process lock), localhost only, graph-guarded.
//
// Scripting is by goal ordinal: one MoveGroup goal per planned move (four when a plan fails: TMR
// retries every plan failure, plan_retries + 1), one ExecuteTrajectory goal per executed move. The
// moves, in order: #1 ready, #2 pre_pick, then per cycle pick, post_pick, pre_place, place,
// post_place (cycle 1 = #3..#7, cycle 2 = #8..#12 when nothing fails).
#include <algorithm>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "m2_7/mf_bin_picking_cell.hpp"

namespace
{
using m25::ErrorCodes;
using m25::Reply;
using m27mf::Cell;
using m27mf::Scenario;

void times(std::deque<Reply>& q, int n, const Reply& r)
{
  for (int i = 0; i < n; ++i) q.push_back(r);
}

// S0-S12 (WP_MF section 7.3) and S6c (INTEGRATION G-3), in the capture's order.
std::vector<Scenario> scenarios()
{
  std::vector<Scenario> v;
  v.push_back({"S0", "nominal: two cycles, each candidate 0 picked, held, placed and released", nullptr});
  v.push_back({"S1", "every candidate unreachable (class A): NO_IK_SOLUTION on every plan of every pick",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 2, Reply::succeed());
                 times(mg.mg_script, 24, Reply::abort(ErrorCodes::NO_IK_SOLUTION));
               }});
  {
    // dropped at the first RUNNING after cycle 1's seventh execute (the post_place dwell), once
    auto dropped = std::make_shared<bool>(false);
    Scenario s{"S2", "move_group's MoveGroup action server gone mid-run (class B, plan): dropped during "
                     "cycle 1's post_place dwell, cycle 2's moves never reach MoveIt", nullptr};
    s.hook = "drop move_action after cycle 1";
    s.between_ticks = [dropped](Cell& c) {
      if (*dropped || c.motionLog().count("/execute_trajectory") < 7) return;
      *dropped = true;
      c.moveGroup().dropMoveAction();
    };
    s.after = [](Cell& c) { c.moveGroup().start(m25::FakeMoveGroup::MOVE_ACTION); };
    v.push_back(s);
  }
  v.push_back({"S3", "CONTROL_FAILED executing candidate 0's approach (pick), nothing held (class B, execute)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.ex_script, 2, Reply::succeed());
                 mg.ex_script.push_back(Reply::abort(ErrorCodes::CONTROL_FAILED));
               }});
  v.push_back({"S4", "CONTROL_FAILED executing pre_place while the object is held (Q8.1 HOLD)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.ex_script, 4, Reply::succeed());
                 mg.ex_script.push_back(Reply::abort(ErrorCodes::CONTROL_FAILED));
               }});
  v.push_back({"S5", "PLANNING_FAILED on every plan of place while the object is held (class A while holding)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 5, Reply::succeed());
                 times(mg.mg_script, 4, Reply::abort(ErrorCodes::PLANNING_FAILED));
               }});
  {
    Scenario s{"S6", "a silent detector (Q9): no fresh detection within the timeout", nullptr};
    s.setup = [](Cell& c) { c.detector().set(true, true); };
    v.push_back(s);
  }
  {
    Scenario s{"S6b", "a fresh detection with no match (class bottle, the tree wants object): the empty bin",
               nullptr};
    s.setup = [](Cell& c) { c.detector().set(false, false); };
    v.push_back(s);
  }
  {
    Scenario s{"S6c", "the detector publishes in a frame TF cannot transform (Q9, the TF lookup failure): "
                      "never an empty bin", nullptr};
    s.setup = [](Cell& c) { c.detector().set(false, true, "m27mf_unknown_optical"); };
    v.push_back(s);
  }
  v.push_back({"S7", "candidate 0 unreachable (NO_IK_SOLUTION on its four plans), candidate 1 reachable",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 2, Reply::succeed());
                 times(mg.mg_script, 4, Reply::abort(ErrorCodes::NO_IK_SOLUTION));
               }});
  v.push_back({"S8", "CONTROL_FAILED executing post_place after the confirmed release, nothing held (class B)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.ex_script, 6, Reply::succeed());
                 mg.ex_script.push_back(Reply::abort(ErrorCodes::CONTROL_FAILED));
               }});
  v.push_back({"S9", "PLANNING_FAILED on every plan of post_place after the release (class A, not a candidate move)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 6, Reply::succeed());
                 times(mg.mg_script, 4, Reply::abort(ErrorCodes::PLANNING_FAILED));
               }});
  v.push_back({"S10", "PLANNING_FAILED on every plan of post_pick after candidate 0's grasp (class A, inside the retry)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 3, Reply::succeed());
                 times(mg.mg_script, 4, Reply::abort(ErrorCodes::PLANNING_FAILED));
               }});
  v.push_back({"S11", "CONTROL_FAILED executing post_pick after candidate 0's grasp (class B, inside the retry)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.ex_script, 3, Reply::succeed());
                 mg.ex_script.push_back(Reply::abort(ErrorCodes::CONTROL_FAILED));
               }});
  v.push_back({"S12", "START_STATE_IN_COLLISION on every plan of every pick (class B: the robot's own state)",
               [](m25::FakeMoveGroup& mg) {
                 times(mg.mg_script, 2, Reply::succeed());
                 times(mg.mg_script, 24, Reply::abort(ErrorCodes::START_STATE_IN_COLLISION));
               }});
  return v;
}

const m27mf::TreeFile kM26Tree{"m2_6_contract/bin_picking_canonical.xml",
                               m26::after::contractFile("bin_picking_canonical.xml")};

const m27mf::TreeFile kM27Tree{"m2_7_mf/bin_picking_canonical.xml",
                               std::string(TRAINIT_M27_DATA_DIR) + "/bin_picking_canonical.xml"};

constexpr const char* kM26TreeSha = "25fb4e2533423d150c040dee2a19bb9d65905d2a9ff247c0c8e7cf6456716b5b";
constexpr const char* kM27TreeSha = "93c37df9acb6c497c1d1af1fad107f646f3d3735165034373b855171557b59a5";
// C0, frozen on adf94b0 by TMR-1 (b11a374): the BEFORE the diff test starts from, never re-frozen
constexpr const char* kC0Sha = "8df96d5fa37f857787fb170a390b1f05455025e4293dab4b186cfa79adf50308";

// --- a committed capture, parsed --------------------------------------------------------------------
struct Block
{
  std::string id;
  std::string text;                       // the whole block, its trailing blank line included
  std::vector<std::string> lines;         // without the trailing blank line
  std::vector<std::string> events;        // one entry per event: its 2-3 lines, joined by '\n'
  std::map<std::string, std::string> summary;   // key -> value of the lines after the events
};

struct Capture
{
  std::vector<std::string> header;        // the three header lines
  std::vector<Block> blocks;
};

std::vector<std::string> splitLines(const std::string& text)
{
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) out.push_back(line);
  return out;
}

Capture parse(const std::string& text)
{
  Capture c;
  const auto lines = splitLines(text);
  std::size_t i = 0;
  for (; i < lines.size() && i < 3; ++i) c.header.push_back(lines[i]);
  for (; i < lines.size(); ++i)
  {
    const std::string& l = lines[i];
    if (l.rfind("=== scenario ", 0) == 0)
    {
      c.blocks.push_back({});
      c.blocks.back().id = l.substr(13);
    }
    if (c.blocks.empty()) continue;
    Block& b = c.blocks.back();
    b.text += l + "\n";
    if (l.empty()) continue;
    b.lines.push_back(l);
  }
  for (Block& b : c.blocks)
  {
    bool in_events = false;
    for (const auto& l : b.lines)
    {
      if (l == "events:") { in_events = true; continue; }
      if (in_events && !l.empty() && l[0] == '[') { b.events.push_back(l); continue; }
      if (in_events && l.rfind("  ", 0) == 0 && !b.events.empty()) { b.events.back() += "\n" + l; continue; }
      in_events = false;
      const auto colon = l.find(": ");
      if (colon != std::string::npos && l.rfind("=== ", 0) != 0) b.summary[l.substr(0, colon)] = l.substr(colon + 2);
    }
  }
  return c;
}

const Block* blockOf(const Capture& c, const std::string& id)
{
  for (const auto& b : c.blocks)
    if (b.id == id) return &b;
  return nullptr;
}

std::vector<std::string> words(const std::string& s)
{
  std::vector<std::string> out;
  std::istringstream in(s);
  std::string w;
  while (in >> w) out.push_back(w);
  return out;
}

// One line of a capture block's summary, BEFORE (as C0 records it) and AFTER (the intended outcome).
struct Change
{
  std::string key;
  std::string before;
  std::string after;
};

// The scenarios whose block may change, the change of each summary line, and where AFTER's MoveIt
// traffic stops: AFTER's events are BEFORE's first events up to and including the cut event, the n-th
// event whose first line contains `cut` (S2: the whole list -- nothing reaches MoveIt after the drop,
// BEFORE or AFTER). The BEFORE values are C0's (AMENDMENTS A-9), never WP_MF's [inf] table.
struct Expected
{
  std::vector<Change> changes;
  std::string cut;      // "" = AFTER's events are BEFORE's, all of them
  int cut_nth{1};
};

const std::map<std::string, Expected>& changedScenarios()
{
  static const std::map<std::string, Expected> m = {   // test data: built once
    {"S2", {{{"moves", "ready pre_pick pick post_pick pre_place place post_place pick pick pick",
                       "ready pre_pick pick post_pick pre_place place post_place pick"},
             {"bt", "SUCCESS", "FAILURE"},
             {"stall", "recovery=1 bin_empty=0 bin_stalled=1", "recovery=0 bin_empty=0 bin_stalled=0"},
             {"vacuum", "engage release", "engage release"},
             {"scene_calls", "2", "2"},
             {"eef_fault", "\"\"", "\"MOTION_FAULT: FAILURE during plan\""},
             {"latched", "(none)", "(none)"},
             {"runner_exit", "0", "1"}},
            "", 1}},
    {"S3", {{{"moves", "ready pre_pick pick pick post_pick pre_place place post_place pick post_pick pre_place place post_place",
                       "ready pre_pick pick"},
             {"bt", "SUCCESS", "FAILURE"},
             {"stall", "recovery=0 bin_empty=0 bin_stalled=0", "recovery=0 bin_empty=0 bin_stalled=0"},
             {"vacuum", "engage release engage release", "(none)"},
             {"scene_calls", "4", "0"},
             {"eef_fault", "\"\"", "\"MOTION_FAULT: CONTROL_FAILED during execute\""},
             {"latched", "(none)", "(none)"},
             {"runner_exit", "0", "1"}},
            "ExecuteTrajectory.goal -> ABORT(-4)", 1}},
    {"S8", {{{"moves", "ready pre_pick pick post_pick pre_place place post_place pick post_pick pre_place place post_place",
                       "ready pre_pick pick post_pick pre_place place post_place"},
             {"bt", "SUCCESS", "FAILURE"},
             {"stall", "recovery=1 bin_empty=0 bin_stalled=1", "recovery=0 bin_empty=0 bin_stalled=0"},
             {"vacuum", "engage release engage release", "engage release"},
             {"scene_calls", "4", "2"},
             {"eef_fault", "\"\"", "\"MOTION_FAULT: CONTROL_FAILED during execute\""},
             {"latched", "(none)", "(none)"},
             {"runner_exit", "0", "1"}},
            "ExecuteTrajectory.goal -> ABORT(-4)", 1}},
    {"S11", {{{"moves", "ready pre_pick pick post_pick pick post_pick pre_place place post_place pick post_pick pre_place place post_place",
                        "ready pre_pick pick post_pick"},
              {"bt", "SUCCESS", "FAILURE"},
              {"stall", "recovery=0 bin_empty=0 bin_stalled=0", "recovery=0 bin_empty=0 bin_stalled=0"},
              {"vacuum", "engage engage release engage release", "engage"},
              {"scene_calls", "5", "1"},
              {"eef_fault", "\"\"", "\"MOTION_FAULT: CONTROL_FAILED during execute\""},
              {"latched", "(none)", "(none)"},
              {"runner_exit", "0", "1"}},
             "ExecuteTrajectory.goal -> ABORT(-4)", 1}},
    {"S12", {{{"moves", "ready pre_pick pick pick pick pick pick pick", "ready pre_pick pick"},
              {"bt", "SUCCESS", "FAILURE"},
              {"stall", "recovery=2 bin_empty=0 bin_stalled=2", "recovery=0 bin_empty=0 bin_stalled=0"},
              {"vacuum", "(none)", "(none)"},
              {"scene_calls", "0", "0"},
              {"eef_fault", "\"\"", "\"MOTION_FAULT: START_STATE_IN_COLLISION during plan\""},
              {"latched", "(none)", "(none)"},
              {"runner_exit", "0", "1"}},
             "-> ABORT(-10)", 4}},
  };
  return m;
}
const std::vector<std::string> kScenarioIds = {"S0", "S1", "S2", "S3", "S4", "S5", "S6", "S6b", "S6c",
                                               "S7", "S8", "S9", "S10", "S11", "S12"};
const std::set<std::string> kUnchanged = {"S0", "S1", "S5", "S6", "S6b", "S6c", "S7", "S9", "S10"};
const std::vector<std::string> kSummaryKeys = {"moves", "bt", "stall", "vacuum", "scene_calls",
                                               "eef_fault", "latched", "runner_exit"};

// The M2.6 -> M2.7 tree: exactly these five lines gain fault="{eef_fault}" (1-based line numbers).
const std::map<int, std::string> kTreeLines = {
  {38, "                <MoveWaypoint waypoint=\"pick\"/>"},
  {41, "                <MoveWaypoint waypoint=\"post_pick\"/>"},
  {50, "            <MoveWaypoint waypoint=\"pre_place\"/>"},
  {51, "            <MoveWaypoint waypoint=\"place\"/>"},
  {56, "            <MoveWaypoint waypoint=\"post_place\"/>"}};
}  // namespace

// C0 (BEFORE; on a later production, C1): the M2.6 tree. Frozen ONLY from production adf94b0.
TEST(M27BinPickingMotion, BEFORE_the_m2_6_tree)
{
  ASSERT_EQ(m25::sha256(m26::after::readFile(kM26Tree.path)), kM26TreeSha)
    << kM26Tree.path << " is not the M2.6 tree TSA pinned (test/data/m2_6_contract/README.md)";
  m27mf::check(m27mf::capture(scenarios(), kM26Tree), "bin_picking_motion_before");
}

// C2 (AFTER): the M2.7 tree. Frozen ONLY from the production of the commit that added it (TMR-7).
TEST(M27BinPickingMotion, AFTER_the_m2_7_tree)
{
  ASSERT_EQ(m25::sha256(m26::after::readFile(kM27Tree.path)), kM27TreeSha)
    << kM27Tree.path << " is not the M2.7 tree TSA generates (test/data/m2_7_mf/README.md)";
  m27mf::check(m27mf::capture(scenarios(), kM27Tree), "bin_picking_motion_after", m27mf::kAfterFrozenOn);
}

// The M2.7 tree is the M2.6 tree with fault="{eef_fault}" on exactly the five MoveWaypoints inside the
// cycle's absorbing Fallback (the candidate moves pick, post_pick inside the retry; pre_place, place,
// post_place after it); the prologue's ready and pre_pick are not wired. Its bytes are what TSA
// generates for its pinned canonical bin-picking project (sha256 93c37df9..., pinned on both sides).
TEST(M27BinPickingMotion, the_m2_7_tree_is_the_pinned_byte_copy_of_what_TSA_generates)
{
  const std::string m26 = m26::after::readFile(kM26Tree.path);
  const std::string m27 = m26::after::readFile(kM27Tree.path);
  ASSERT_EQ(m25::sha256(m26), kM26TreeSha);
  ASSERT_EQ(m25::sha256(m27), kM27TreeSha);
  EXPECT_EQ(m27.size(), 4880u);
  const auto before = splitLines(m26), after = splitLines(m27);
  ASSERT_EQ(before.size(), after.size());
  std::map<int, std::string> changed;
  for (std::size_t i = 0; i < before.size(); ++i)
    if (before[i] != after[i])
    {
      changed[static_cast<int>(i + 1)] = before[i];
      const std::string wired = before[i].substr(0, before[i].size() - 2) + " fault=\"{eef_fault}\"/>";
      EXPECT_EQ(after[i], wired) << "line " << i + 1;
    }
  EXPECT_EQ(changed, kTreeLines);
  for (const char* prologue : {"      <MoveWaypoint waypoint=\"ready\"/>", "      <MoveWaypoint waypoint=\"pre_pick\"/>"})
    EXPECT_NE(m27.find(std::string(prologue) + "\n"), std::string::npos) << prologue << " (not wired)";
}

// AFTER - BEFORE == the intended lines (WP_MF 7.5, AMENDMENTS A-9). Pure: the two committed captures.
//   - the header: lines 1-2 equal, line 3 the two trees;
//   - the same scenarios in the same order;
//   - UNCHANGED (S0 S1 S5 S6 S6b S6c S7 S9 S10): byte-identical blocks -- class A keeps its strategy,
//     a held object is HOLD as before, a detector fault is a FAULT as before;
//   - S4 (CONTROL_FAILED on pre_place while holding): identical but its eef_fault line, which now says why;
//   - CHANGED (S2 S3 S8 S11 S12, class B): FAULT, and the run stops: AFTER's MoveIt traffic is BEFORE's
//     up to the class-B failure (no request BEFORE did not send), its moves a prefix of BEFORE's, and
//     the summary lines move from C0's values to the intended ones.
TEST(M27BinPickingMotion, the_BEFORE_to_AFTER_diff_is_exactly_the_intended_lines)
{
  const std::string before_text = m26::after::readFile(std::string(TRAINIT_M27_DATA_DIR) + "/bin_picking_motion_before.txt");
  const std::string after_text = m26::after::readFile(std::string(TRAINIT_M27_DATA_DIR) + "/bin_picking_motion_after.txt");
  ASSERT_EQ(m25::sha256(before_text), kC0Sha) << "the committed BEFORE (C0) is not TMR-1's";
  ASSERT_FALSE(after_text.empty()) << "no committed AFTER capture";
  const Capture before = parse(before_text), after = parse(after_text);

  ASSERT_EQ(before.header.size(), 3u);
  ASSERT_EQ(after.header.size(), 3u);
  EXPECT_EQ(before.header[0], "m2_7 capture bin_picking_motion");
  EXPECT_EQ(after.header[0], before.header[0]);
  EXPECT_EQ(after.header[1], before.header[1]) << "another environment is not a pass";
  EXPECT_EQ(before.header[2], std::string("tree: m2_6_contract/bin_picking_canonical.xml sha256=") + kM26TreeSha);
  EXPECT_EQ(after.header[2], std::string("tree: m2_7_mf/bin_picking_canonical.xml sha256=") + kM27TreeSha);

  std::vector<std::string> ids_before, ids_after;
  for (const auto& b : before.blocks) ids_before.push_back(b.id);
  for (const auto& b : after.blocks) ids_after.push_back(b.id);
  EXPECT_EQ(ids_before, kScenarioIds);
  ASSERT_EQ(ids_after, kScenarioIds);

  std::ostringstream summary;   // the per-scenario diff, written to the build tree for the checkpoint
  summary << "BEFORE " << kC0Sha << " -> AFTER " << m25::sha256(after_text) << "\n";
  for (const std::string& id : kScenarioIds)
  {
    const Block& b = *blockOf(before, id);
    const Block& a = *blockOf(after, id);
    EXPECT_EQ(b.text.find("\ntopics ("), std::string::npos) << id << ": BEFORE has asynchronous topics";
    EXPECT_EQ(a.text.find("\ntopics ("), std::string::npos) << id << ": AFTER has asynchronous topics";
    EXPECT_EQ(a.summary.at("pins"), b.summary.at("pins")) << id;
    EXPECT_EQ(a.summary.at("script"), b.summary.at("script")) << id;
    if (kUnchanged.count(id))
    {
      EXPECT_EQ(a.text, b.text) << id << " must be byte-identical";
      summary << id << ": unchanged (" << b.events.size() << " events)\n";
      continue;
    }
    if (id == "S4")
    {
      ASSERT_EQ(a.lines.size(), b.lines.size()) << id;
      std::vector<std::pair<std::string, std::string>> diff;
      for (std::size_t i = 0; i < b.lines.size(); ++i)
        if (a.lines[i] != b.lines[i]) diff.emplace_back(b.lines[i], a.lines[i]);
      EXPECT_EQ(diff, (std::vector<std::pair<std::string, std::string>>{
                        {"eef_fault: \"\"", "eef_fault: \"MOTION_FAULT: CONTROL_FAILED during execute\""}}))
        << id << ": only the eef_fault line may change";
      summary << id << ": only eef_fault: \"\" -> \"MOTION_FAULT: CONTROL_FAILED during execute\" ("
              << b.events.size() << " events, identical)\n";
      continue;
    }
    const auto it = changedScenarios().find(id);
    ASSERT_NE(it, changedScenarios().end()) << id << " is in no set";
    const Expected& want = it->second;
    // 1. MoveIt traffic: AFTER's events are BEFORE's, cut after the class-B failure
    std::size_t cut = b.events.size();
    if (!want.cut.empty())
    {
      int seen = 0;
      cut = 0;
      for (std::size_t i = 0; i < b.events.size(); ++i)
        if (b.events[i].substr(0, b.events[i].find('\n')).find(want.cut) != std::string::npos && ++seen == want.cut_nth)
        {
          cut = i + 1;
          break;
        }
      ASSERT_GT(cut, 0u) << id << ": BEFORE has no event '" << want.cut << "' #" << want.cut_nth;
      EXPECT_LT(cut, b.events.size()) << id << ": BEFORE went on after that event";
    }
    ASSERT_EQ(a.events.size(), cut) << id << ": AFTER's events must stop right after the class-B failure";
    for (std::size_t i = 0; i < cut; ++i)
      EXPECT_EQ(a.events[i], b.events[i]) << id << ": event " << i + 1 << " (text and sha256)";
    // 2. the summary lines: C0's value BEFORE, the intended one AFTER
    std::set<std::string> keys;
    for (const Change& c : want.changes)
    {
      keys.insert(c.key);
      EXPECT_EQ(b.summary.at(c.key), c.before) << id << " BEFORE " << c.key << " (C0)";
      EXPECT_EQ(a.summary.at(c.key), c.after) << id << " AFTER " << c.key;
    }
    EXPECT_EQ(keys, std::set<std::string>(kSummaryKeys.begin(), kSummaryKeys.end())) << id;
    // 3. the moves: a prefix of BEFORE's
    const auto mb = words(b.summary.at("moves")), ma = words(a.summary.at("moves"));
    ASSERT_LE(ma.size(), mb.size()) << id;
    EXPECT_TRUE(std::equal(ma.begin(), ma.end(), mb.begin())) << id << ": AFTER's moves are not a prefix of BEFORE's";
    // nothing else in the block: its lines are the header, the events and the summary lines
    std::size_t event_lines = 0;
    for (const auto& e : a.events) event_lines += splitLines(e).size();
    EXPECT_EQ(a.lines.size(), 4 + event_lines + kSummaryKeys.size())
      << id << ": the AFTER block holds its 4 header lines, its events and the 8 summary lines, nothing else";
    summary << id << ": FAULT, the run stops: events " << b.events.size() << " -> " << a.events.size()
            << " (BEFORE's first " << a.events.size() << ")";
    for (const Change& c : want.changes)
      if (c.before != c.after) summary << "; " << c.key << " " << c.before << " -> " << c.after;
    summary << "\n";
  }
  std::ofstream(std::string(TRAINIT_M27_ACTUAL_DIR) + "/bin_picking_motion_before_to_after.diff",
                std::ios::binary | std::ios::trunc) << summary.str();
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
