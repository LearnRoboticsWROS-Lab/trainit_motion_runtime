// vNext M2.5: the direct-MoveIt accesses of TMR's runtime CONSUMERS, counted in their source.
//
// The MotionRuntime seam (ADR-0015 amendment M2.5) says BT nodes, TaskRunner and the two runners
// reach MoveIt only through MotionRuntime. This test reads those four translation units, strips
// comments (string- and raw-string-aware) and counts:
//   - MoveIt C++ names in code: the MGI/PSI/PSM types, the raw calls, the namespaces;
//   - `#include <moveit/...>` lines;
//   - MoveIt endpoint names inside string literals (a reach-around over ROS), except two
//     allow-listed literals of ServoTrack's arbitration port (class D), each required exactly once.
// The EXPECTED table is the checkpoint's "direct MoveIt accesses before/after", machine-checked.
// It is structural: it changes in the commit that changes the code it counts, never to make a
// behaviour capture pass. No ROS graph is used.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
// The runtime's CONSUMERS: scanned, and built with -Werror=deprecated-declarations.
const std::vector<std::string> kUnits = {
  "src/bt_nodes.cpp", "src/task_runner.cpp", "src/trainit_run_bt_node.cpp",
  "src/trainit_run_task_node.cpp"};

// Sources with no MoveIt access at all: scanned too (every count must be 0).
// vNext M2.6 (ADR-0022): the end-effector contract's parser, its two pure tables (the actuator
// outcome and the VerifyGrasp judge) and the registry of standard EEF clients (SetBool,
// GripperCommand, FollowJointTrajectory: actuation, the seam's class C, with no MoveIt API).
// vNext M2.7 (owner Q8.2): the motion-failure classification, a pure table over MotionResult.
const std::vector<std::string> kPlain = {
  "src/bt_profiles.cpp", "src/process_path_generators.cpp", "src/process_path_loaders.cpp",
  "src/task_loader.cpp", "src/end_effector_contract.cpp", "src/actuator_outcome.cpp",
  "src/verify_grasp.cpp", "src/end_effector_registry.cpp", "src/motion_failure.cpp",
  // vNext M3.1 (ADR-0024, ADR-0025): the provider table and the path library, pure data/geometry
  "src/motion_providers.cpp", "src/path_library.cpp",
  // vNext M3.6 (ADR-0026): the grasp-candidate contract (validation, projection, align_approach)
  "src/grasp_candidates.cpp"};

// Sources that legitimately name MoveIt (ADR-0015 amendment M2.5-4), not scanned.
const std::map<std::string, std::string> kExempt = {
  {"src/moveit_motion_runtime.cpp", "B: the door's MoveIt implementation"},
  {"src/pilz_sequence_backend.cpp", "B: a process-path backend (sequence_move_group)"},
  {"src/cartesian_waypoint_backend.cpp", "B: a process-path backend (computeCartesianPath)"},
  {"src/trajectory_validator.cpp", "B: validates against the robot model and scene"},
  {"src/planning_scene_manager.cpp", "B (monitor) / C (scene writes): the runtime's scene"},
  {"src/manual_motion_strategy_selector.cpp", "B: resolves a mode to a pipeline/planner"},
  {"src/motion_types.cpp", "B: the motion vocabulary"},
  {"src/scene_manager_node.cpp", "C: the scene-loading node (planning-scene writes)"},
  {"src/gripper_controller.cpp", "C: end-effector actuation (GripperCommand)"},
  {"src/process_controller.cpp", "C: cell I/O (ADR-0013)"},
};

const std::vector<std::string> kCodeNames = {
  "moveGroup", "MoveGroupInterface", "PlanningSceneInterface", "PlanningSceneMonitor",
  "snapshotForValidation", "getCurrentPose", "getCurrentState", "computeCartesianPath",
  "setPlanningPipelineId", "setPlannerId", "planning_interface", "planning_scene_monitor"};

const std::vector<std::string> kEndpoints = {
  "move_action", "execute_trajectory", "sequence_move_group", "compute_cartesian_path",
  "plan_kinematic_path", "get_planning_scene", "apply_planning_scene", "query_planner_interface",
  "compute_ik", "compute_fk", "check_state_validity", "trajectory_execution_event",
  "attached_collision_object"};

// ServoTrack's arbitration port default and its detail text (class D, ADR-0015 Decision 5).
const std::vector<std::string> kAllowedLiterals = {
  "execute_trajectory/_action/status", "no execute_trajectory status published"};

// AS-IS on TMR 1.8.0: the three class-A reads, all in bt_nodes.cpp (moveGroup 3, getCurrentPose 3).
// M2.5 (4/7): the currentPose() helper reads MotionRuntime::currentTipPose() (2 + 2).
// M2.5 (5/7): ServoTrack's frame and feedback reads too: none left.
const std::map<std::string, int> kExpected = {};

struct Split
{
  std::string code;                  // comments and literals removed (preprocessor lines kept)
  std::vector<std::string> literals;  // the contents of every string literal
};

Split split(const std::string& src)
{
  Split out;
  std::size_t i = 0;
  const std::size_t n = src.size();
  while (i < n)
  {
    const char c = src[i];
    if (c == '/' && i + 1 < n && src[i + 1] == '/')
    {
      while (i < n && src[i] != '\n') ++i;
      continue;
    }
    if (c == '/' && i + 1 < n && src[i + 1] == '*')
    {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) ++i;
      i += 2;
      out.code += ' ';
      continue;
    }
    if (c == 'R' && i + 1 < n && src[i + 1] == '"')
    {
      const std::size_t open = src.find('(', i + 2);
      const std::string delim = ")" + src.substr(i + 2, open - (i + 2)) + "\"";
      const std::size_t close = src.find(delim, open);
      out.literals.push_back(src.substr(open + 1, close - open - 1));
      i = close + delim.size();
      out.code += "\"\"";
      continue;
    }
    if (c == '"' || c == '\'')
    {
      std::string lit;
      ++i;
      while (i < n && src[i] != c)
      {
        if (src[i] == '\\' && i + 1 < n) { lit += src[i + 1]; i += 2; continue; }
        lit += src[i++];
      }
      ++i;
      if (c == '"') out.literals.push_back(lit);
      out.code += "\"\"";
      continue;
    }
    out.code += c;
    ++i;
  }
  return out;
}

int countWord(const std::string& text, const std::string& word)
{
  const std::regex re("\\b" + word + "\\b");
  return static_cast<int>(std::distance(std::sregex_iterator(text.begin(), text.end(), re),
                                        std::sregex_iterator()));
}

std::string read(const std::string& rel)
{
  std::ifstream in(std::string(TRAINIT_SOURCE_DIR) + "/" + rel);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}  // namespace

// Default-deny: a source file nobody classified fails here, so a new BT node file (or a split of
// bt_nodes.cpp) cannot escape the scan and the -Werror barrier by not being listed.
TEST(M25SeamScan, every_source_file_is_classified)
{
  std::vector<std::string> unclassified;
  for (const auto& entry : std::filesystem::directory_iterator(std::string(TRAINIT_SOURCE_DIR) + "/src"))
  {
    if (entry.path().extension() != ".cpp") continue;
    const std::string rel = "src/" + entry.path().filename().string();
    const bool known = std::count(kUnits.begin(), kUnits.end(), rel) ||
                       std::count(kPlain.begin(), kPlain.end(), rel) || kExempt.count(rel);
    if (!known) unclassified.push_back(rel);
  }
  EXPECT_TRUE(unclassified.empty())
    << "classify these as a runtime consumer (kUnits), MoveIt-free (kPlain) or a B/C exception "
       "(kExempt, with its reason): "
    << ::testing::PrintToString(unclassified);
}

// The two lists the barrier keeps are ONE list: the units CMakeLists.txt builds with
// -Werror=deprecated-declarations are exactly the scanned consumers.
TEST(M25SeamScan, the_compiler_barrier_covers_exactly_the_scanned_consumers)
{
  const std::string cmake = read("CMakeLists.txt");
  const auto end = cmake.find("\"-Werror=deprecated-declarations\"");
  ASSERT_NE(end, std::string::npos) << "the barrier's set_source_files_properties is gone";
  const auto begin = cmake.rfind("set_source_files_properties(", end);
  ASSERT_NE(begin, std::string::npos);
  std::vector<std::string> barrier;
  const std::regex unit("src/[A-Za-z0-9_]+\\.cpp");
  const std::string block = cmake.substr(begin, end - begin);
  for (auto it = std::sregex_iterator(block.begin(), block.end(), unit); it != std::sregex_iterator(); ++it)
    barrier.push_back(it->str());
  std::vector<std::string> want = kUnits;
  std::sort(barrier.begin(), barrier.end());
  std::sort(want.begin(), want.end());
  EXPECT_EQ(barrier, want);
}

TEST(M25SeamScan, the_runtime_consumers_reach_moveit_only_through_motion_runtime)
{
  std::map<std::string, int> found;
  std::map<std::string, int> allowed;
  std::vector<std::string> scanned = kUnits;
  scanned.insert(scanned.end(), kPlain.begin(), kPlain.end());
  for (const auto& unit : scanned)
  {
    const std::string src = read(unit);
    ASSERT_FALSE(src.empty()) << unit;
    const Split s = split(src);
    for (const auto& name : kCodeNames)
      if (const int k = countWord(s.code, name)) found[unit + ":" + name] = k;
    if (const int k = countWord(s.code, "include <moveit")) found[unit + ":#include <moveit/"] = k;
    for (const auto& lit : s.literals)
    {
      bool is_allowed = false;
      for (const auto& a : kAllowedLiterals)
        if (lit == a) { ++allowed[unit + ":" + a]; is_allowed = true; }
      if (is_allowed) continue;
      for (const auto& e : kEndpoints)
        if (lit.find(e) != std::string::npos) ++found[unit + ":\"" + e + "\""];
    }
  }
  std::ostringstream table;
  for (const auto& [k, v] : found) table << "  " << k << " = " << v << "\n";
  EXPECT_EQ(found, kExpected) << "direct MoveIt accesses in the runtime consumers:\n" << table.str();
  for (const auto& a : kAllowedLiterals)
    EXPECT_EQ(allowed["src/bt_nodes.cpp:" + a], 1) << "allow-listed literal '" << a << "'";
}

// vNext M2.7 (owner decision 11.2): BtContext and TaskRunner depend on the MotionRuntime door.
// What they still took from the concrete runtime at 2.0.0 (namedPose, tipLink, baseFrame, scene(),
// node()) is behind the door or handed over by the composition roots: the BT nodes and BtContext
// name the concrete runtime nowhere, TaskRunner only in its compatibility constructor (declared
// once, defined once, delegating), and the two runners hand over the runtime's scene. In code:
// comments and literals do not count.
TEST(M25SeamScan, the_consumers_depend_on_the_door_not_on_the_concrete_runtime)
{
  struct Pin
  {
    const char* file;
    const char* pattern;
    int count;
    const char* why;
  };
  const std::vector<Pin> pins = {
    {"src/bt_nodes.cpp", "\\bMoveItMotionRuntime\\b", 0, "the BT nodes name only the door"},
    {"src/bt_nodes.cpp", "\\bscene\\(\\)", 0, "the scene nodes write BtContext::scene"},
    {"include/trainit_motion_runtime/bt_context.hpp", "\\bMoveItMotionRuntime\\b", 0,
     "BtContext::runtime is a MotionRuntime*"},
    {"include/trainit_motion_runtime/task_runner.hpp", "\\bMoveItMotionRuntime\\b", 1,
     "only the compatibility constructor's declaration"},
    {"src/task_runner.cpp", "\\bMoveItMotionRuntime\\b", 1, "only the compatibility constructor's definition"},
    {"src/task_runner.cpp", "\\bscene\\(\\)", 1, "only in the compatibility constructor"},
    {"src/task_runner.cpp", "\\bnode\\(\\)", 1, "only in the compatibility constructor (its logger)"},
    {"src/trainit_run_bt_node.cpp", "ctx\\.scene = &runtime->scene\\(\\);", 1,
     "the BT runner hands the runtime's scene to the context"},
    {"src/trainit_run_task_node.cpp",
     "TaskRunner runner\\(\\*runtime, runtime->scene\\(\\), node->get_logger\\(\\),", 1,
     "the task runner is built from the door, the scene and the logger"},
  };
  for (const auto& p : pins)
  {
    const std::string src = read(p.file);
    ASSERT_FALSE(src.empty()) << p.file;
    const std::string code = split(src).code;
    const std::regex re(p.pattern);
    const int n = static_cast<int>(std::distance(std::sregex_iterator(code.begin(), code.end(), re),
                                                 std::sregex_iterator()));
    EXPECT_EQ(n, p.count) << p.file << " /" << p.pattern << "/: " << p.why;
  }
}

// The scanner itself: a comment, a literal and a raw string are not code; code is.
TEST(M25SeamScan, the_scanner_tells_code_from_comments_and_literals)
{
  const Split s = split("a.moveGroup(); // moveGroup\n/* moveGroup */ f(\"move_action\");\n"
                        "auto r = R\"x(get_planning_scene)x\"; char c = '\"'; b.moveGroup();");
  EXPECT_EQ(countWord(s.code, "moveGroup"), 2);
  ASSERT_EQ(s.literals.size(), 2u);
  EXPECT_EQ(s.literals[0], "move_action");
  EXPECT_EQ(s.literals[1], "get_planning_scene");
}
