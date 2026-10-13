// vNext M2.7 (owner decision D2): the M2.5 namespace families' BEFORE -> AFTER diff is RESOURCE
// NAMES / NAMESPACE RESOLUTION only. No ROS graph; the committed files are read.
//
// BEFORE: test/data/m2_5/namespace*.txt, recorded on TMR 1.8.0 (9cfb601), kept byte-identical:
//         their sha256 are pinned here, so a re-frozen BEFORE record fails this test.
// AFTER:  test/data/m2_7_ns/namespace*.txt, recorded on the M2.7 production sources (the
//         MoveGroupInterface and PlanningSceneInterface built in the node's namespace).
//
// An AFTER line equals its BEFORE line, or is the BEFORE line rewritten by exactly ONE rule:
//   R1  an event line "[k] ACTION|SERVICE <one of MoveGroupInterface's 8 root names> ..." -> the
//       same line with the name under /r3 (the endpoint moved, the request did not change);
//   R2  a "counts:" line naming one of them: every such key under /r3, the keys in std::map order;
//   R3  an NS-03 stage line "after <stage>, within 10 s: constructed=no" -> "constructed=yes"
//       (the constructor no longer waits for a ROOT endpoint).
// Only MoveGroupInterface's eight names may move: a PlanningSceneInterface root name never counts
// (INTEGRATION C-12). Same line count; EXACTLY the recorded lines change, each by its recorded rule
// (WP_NS 6.1: 9 + 9 + 3 + 2 lines), so an AFTER equal to BEFORE fails too; and no AFTER event or
// counts line names a root MoveGroupInterface endpoint any more.
#include <cstddef>
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
const std::set<std::string> kRootMgi = {
  "/move_action", "/execute_trajectory", "/query_planner_interface", "/get_planner_params",
  "/set_planner_params", "/compute_cartesian_path", "/trajectory_execution_event",
  "/attached_collision_object"};
const std::string kNs = "/r3";
const std::regex kEvent(R"(^(\[\d+\] (?:ACTION|SERVICE) )(\S+)( .*)$)");
const std::regex kStage(R"(^(after .+, within 10 s: constructed=)no$)");

// The BEFORE record (TMR 1.8.0), byte for byte: it is never re-frozen (test/data/m2_5/README.md).
const std::map<std::string, std::string> kBeforeSha256 = {
  {"namespace", "ec88f398a6470b909058c8df5b8d7ffa56b99da915fb3071d2634cfa58656bce"},
  {"namespace_remap", "512204a2f1e6319c46ad5075fb746eced4ef85bdaebd1d439f272555e5d7b7ed"},
  {"namespace_constructor", "9d41c3b403a4d6e5068364ce6f8376a9bfcf3b9da64951f433837f219852d157"},
  {"namespace_remap_constructor", "5071790526fbb276a9bc0944774879707a3b87a62cc25e43feca231ddc41fa96"}};

// WP_NS 6.1: the lines that change (1-based) and the one rule that explains each. Every other
// line is byte-identical.
using Changed = std::map<std::size_t, std::string>;
const Changed kMotionLines = {{12, "R1"}, {147, "R1"}, {156, "R2"}, {159, "R1"}, {320, "R1"},
                              {329, "R2"}, {533, "R1"}, {600, "R1"}, {609, "R2"}};
const std::map<std::string, Changed> kChanged = {
  {"namespace", kMotionLines},
  {"namespace_remap", kMotionLines},
  {"namespace_constructor", {{7, "R3"}, {8, "R3"}, {9, "R3"}}},
  {"namespace_remap_constructor", {{7, "R3"}, {8, "R3"}}}};

std::string slurp(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in.is_open()) << path;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<std::string> lines(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s)
  {
    if (c == '\n') { out.push_back(cur); cur.clear(); }
    else cur += c;
  }
  out.push_back(cur);
  return out;
}

std::string r1(const std::string& b)
{
  std::smatch m;
  if (std::regex_match(b, m, kEvent) && kRootMgi.count(m[2])) return std::string(m[1]) + kNs + std::string(m[2]) + std::string(m[3]);
  return {};
}

std::string r2(const std::string& b)
{
  const std::string head = "counts: ";
  if (b.rfind(head, 0) != 0 || b == "counts: none") return {};
  std::map<std::string, std::string> moved;
  bool any = false;
  std::istringstream in(b.substr(head.size()));
  std::string kv;
  while (in >> kv)
  {
    const auto eq = kv.rfind('=');
    std::string k = kv.substr(0, eq);
    if (kRootMgi.count(k)) { k = kNs + k; any = true; }
    moved[k] = kv.substr(eq + 1);
  }
  if (!any) return {};
  std::string out = head;
  bool first = true;
  for (const auto& [k, v] : moved) { out += (first ? "" : " ") + k + "=" + v; first = false; }
  return out;
}

std::string r3(const std::string& b)
{
  std::smatch m;
  if (std::regex_match(b, m, kStage)) return std::string(m[1]) + "yes";
  return {};
}

// A "counts:" line that still names a root MoveGroupInterface endpoint.
bool countsNameRootMgi(const std::string& a)
{
  const std::string head = "counts: ";
  if (a.rfind(head, 0) != 0) return false;
  std::istringstream in(a.substr(head.size()));
  std::string kv;
  while (in >> kv)
    if (kRootMgi.count(kv.substr(0, kv.rfind('=')))) return true;
  return false;
}

struct Result
{
  Changed changed;                       // line (1-based) -> the rule that explains it
  std::vector<std::string> unexplained;
};

Result classify(const std::string& before_text, const std::string& after_text)
{
  const auto before = lines(before_text);
  const auto after = lines(after_text);
  Result r;
  if (before.size() != after.size())
  {
    r.unexplained.push_back("line count " + std::to_string(before.size()) + " -> " + std::to_string(after.size()));
    return r;
  }
  for (std::size_t i = 0; i < before.size(); ++i)
  {
    const std::string& b = before[i];
    const std::string& a = after[i];
    if (a == b) continue;
    if (!r1(b).empty() && r1(b) == a) { r.changed[i + 1] = "R1"; continue; }
    if (!r2(b).empty() && r2(b) == a) { r.changed[i + 1] = "R2"; continue; }
    if (!r3(b).empty() && r3(b) == a) { r.changed[i + 1] = "R3"; continue; }
    r.unexplained.push_back("line " + std::to_string(i + 1) + ": '" + b + "' -> '" + a + "'");
  }
  for (std::size_t i = 0; i < after.size(); ++i)
  {
    std::smatch m;
    if ((std::regex_match(after[i], m, kEvent) && kRootMgi.count(m[2])) || countsNameRootMgi(after[i]))
      r.unexplained.push_back("line " + std::to_string(i + 1) + ": AFTER still names a root MoveIt endpoint");
  }
  return r;
}

std::string beforeFile(const std::string& family) { return std::string(TRAINIT_M25_DATA_DIR) + "/" + family + ".txt"; }
std::string afterFile(const std::string& family) { return std::string(TRAINIT_M27_NS_DATA_DIR) + "/" + family + ".txt"; }
}  // namespace

TEST(M27NamespaceDiff, the_after_series_differs_from_before_only_by_namespace_resolution)
{
  for (const auto& [family, want] : kChanged)
  {
    const std::string before = slurp(beforeFile(family));
    const std::string after = slurp(afterFile(family));
    ASSERT_FALSE(before.empty()) << beforeFile(family);
    ASSERT_FALSE(after.empty()) << afterFile(family);
    EXPECT_EQ(m25::sha256(before), kBeforeSha256.at(family))
      << family << ": the BEFORE record (TMR 1.8.0) changed -- it is kept byte for byte and never re-frozen";
    EXPECT_NE(after, before) << family << ": the AFTER series equals BEFORE -- the namespace fix is not in it";
    const Result r = classify(before, after);
    EXPECT_TRUE(r.unexplained.empty()) << family << ": a difference that is not namespace resolution:\n  "
                                       << ::testing::PrintToString(r.unexplained);
    EXPECT_EQ(r.changed, want) << family << ": the AFTER series moved a different set of lines";
  }
}

// The rules themselves: a name outside the eight, a changed request, a reordered count are NOT
// namespace resolution; nor is an unchanged series.
TEST(M27NamespaceDiff, the_rules_accept_only_a_moved_name)
{
  EXPECT_EQ(r1("[1] ACTION /move_action MoveGroup.goal #1 -> SUCCEED(1)"), "[1] ACTION /r3/move_action MoveGroup.goal #1 -> SUCCEED(1)");
  EXPECT_EQ(r1("[1] SERVICE /compute_cartesian_path GetCartesianPath.request #1 -> SUCCEED(1)"),
            "[1] SERVICE /r3/compute_cartesian_path GetCartesianPath.request #1 -> SUCCEED(1)");
  EXPECT_EQ(r1("[1] ACTION /r3/sequence_move_group MoveGroupSequence.goal #1 -> SUCCEED(1)"), "");
  EXPECT_EQ(r1("[2] SERVICE /get_planning_scene GetPlanningScene.request components=1023"), "")
    << "the scene services are not MoveGroupInterface's: a PSI root name never counts as moved";
  EXPECT_NE(r1("[1] ACTION /move_action MoveGroup.goal #1 -> SUCCEED(1)"), "[1] ACTION /r3/move_action MoveGroup.goal #2 -> SUCCEED(1)");
  EXPECT_EQ(r2("counts: /execute_trajectory=1 /move_action=1 /r3/get_planning_scene=1"),
            "counts: /r3/execute_trajectory=1 /r3/get_planning_scene=1 /r3/move_action=1");
  EXPECT_EQ(r2("counts: /r3/servo_n1/pause_servo=1"), "");
  EXPECT_EQ(r2("counts: /apply_planning_scene=1 /get_planning_scene=1"), "")
    << "a PSI root name in a counts line never counts as moved";
  EXPECT_EQ(r2("counts: none"), "");
  EXPECT_EQ(r3("after root /move_action, within 10 s: constructed=no"), "after root /move_action, within 10 s: constructed=yes");
  EXPECT_EQ(r3("after root /move_action, within 10 s: constructed=yes"), "");

  // classify(): an unchanged series moves no line (so the first test fails on AFTER == BEFORE), and
  // a moved PSI name, a changed ordinal or a root MGI name left in AFTER are unexplained.
  const std::string before = "x\n[1] ACTION /move_action MoveGroup.goal #1 -> SUCCEED(1)\n"
                             "[2] SERVICE /get_planning_scene GetPlanningScene.request components=1023\n";
  EXPECT_TRUE(classify(before, before).changed.empty());
  EXPECT_FALSE(classify(before, before).unexplained.empty()) << "BEFORE itself names a root MGI endpoint";
  const Result ok = classify(before, "x\n[1] ACTION /r3/move_action MoveGroup.goal #1 -> SUCCEED(1)\n"
                                     "[2] SERVICE /get_planning_scene GetPlanningScene.request components=1023\n");
  EXPECT_TRUE(ok.unexplained.empty()) << ::testing::PrintToString(ok.unexplained);
  EXPECT_EQ(ok.changed, (Changed{{2, "R1"}}));
  EXPECT_EQ(classify(before, "x\n[1] ACTION /r3/move_action MoveGroup.goal #1 -> SUCCEED(1)\n"
                             "[2] SERVICE /r3/get_planning_scene GetPlanningScene.request components=1023\n")
              .unexplained.size(), 1u) << "a moved PSI name is not namespace resolution of MoveGroupInterface";
  EXPECT_EQ(classify(before, "x\n[1] ACTION /r3/move_action MoveGroup.goal #2 -> SUCCEED(1)\n"
                             "[2] SERVICE /get_planning_scene GetPlanningScene.request components=1023\n")
              .unexplained.size(), 1u) << "a moved name with a changed ordinal is not namespace resolution";
  EXPECT_EQ(classify(before, before + "counts: /r3/move_action=1\n").unexplained,
            (std::vector<std::string>{"line count 4 -> 5"})) << "an added line is not namespace resolution";
}
