// vNext M2.6 AFTER (ADR-0022), M6: the REAL trainit_run_bt as a child process (domain 88), the way
// a launch starts it (a params file, an optional `-r __ns:=`), against the M2.5 fake move_group
// (the runner builds a MoveItMotionRuntime first) and the AFTER fakes for the end effector:
//
//   - exit 2: a malformed contract (REFUSED <key>, before any MoveIt client exists), a literal
//     eef/site/target port the contract does not know, a non-integer timeout_ms literal, SetGripper
//     in a canonical tree, two writers on a device's state topic;
//   - exit 3: a canonical tree that ends SUCCESS while an end-effector FAULT was latched;
//   - exit 0 / 1 as before for a canonical run that succeeds / fails; exit 1 for a canonical tree
//     that throws (halted, never std::terminate), while a legacy one is unchanged (SIGABRT);
//   - the pinned lines: "end-effector contract: LEGACY (ADR-0022)", "end-effector contract:
//     canonical, devices [<ids>] (ADR-0022)" (ONE per run), and the legacy GripperController's
//     EEF-D3 WARN for a non-SUCCEEDED result.
// Harness keys every params file carries, as in E7: the inline M2.5 test arm, groot_monitor false.
#include <sys/stat.h>

#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"
#include "m2_6/eef_after_contract.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "m2_6/eef_fakes.hpp"

namespace
{
using m26::GripperReply;
using m26::milliseconds;

std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/eef_runner_child"; }

// The process-wide MoveIt fakes the runner's MotionRuntime waits for: the root set, and in every
// namespace a child runs in (/r1, /r2) the move_group actions and the scene services. A namespaced
// child's MoveGroupInterface resolves move_action and execute_trajectory in its own namespace
// since the vNext M2.7 namespace fix (ADR-0023 Decision 3). Without those actions the C11-C13
// children would wait for a move_group nobody serves and never get past the MotionRuntime: C11
// and C12 would never tick their tree, C13 would never reach its start-up two-writers check.
struct MoveItFakes
{
  MoveItFakes()
  {
    log.setBodies(false);
    root = std::make_unique<m25::FakeMoveGroup>(log, "/");
    for (const char* ns : {"/r1", "/r2"})
      scenes.push_back(std::make_unique<m25::FakeMoveGroup>(
        log, ns, m25::FakeMoveGroup::MOVE_ACTION | m25::FakeMoveGroup::EXECUTE | m25::FakeMoveGroup::SCENE_SERVICES));
  }
  m26::Log log;
  std::unique_ptr<m25::FakeMoveGroup> root;
  std::vector<std::unique_ptr<m25::FakeMoveGroup>> scenes;
};

MoveItFakes& moveit()
{
  static std::unique_ptr<MoveItFakes> f;   // test harness: one per process
  if (!f)
  {
    f = std::make_unique<MoveItFakes>();
    m26::teardowns().push_back([] { f.reset(); });
  }
  return *f;
}

struct Run
{
  std::string exit;       // "exit code N"
  std::string finished;   // the 'BT finished:' status ("" = absent)
  std::vector<std::string> lines;
  // how many lines END with `text` (the message after the "[logger]: " prefix)
  int count(const std::string& text) const
  {
    int n = 0;
    for (const auto& l : lines)
      if (l.size() >= text.size() && l.compare(l.size() - text.size(), text.size(), text) == 0) ++n;
    return n;
  }
  bool contains(const std::string& text) const
  {
    for (const auto& l : lines)
      if (l.find(text) != std::string::npos) return true;
    return false;
  }
  std::string dump() const
  {
    std::string s;
    for (const auto& l : lines) s += l + "\n";
    return s;
  }
};

// Runs trainit_run_bt with `keys` (YAML lines under ros__parameters) and the tree `tree_xml` (the
// inside of a BehaviorTree), in `ns`.
Run runChild(const std::string& id, const std::string& ns, const std::string& keys, const std::string& tree_xml)
{
  ::mkdir(actualDir().c_str(), 0755);
  moveit();
  const std::string stem = actualDir() + "/" + m26::tagOf(id);
  m26::writeFile(stem + ".xml", "<?xml version=\"1.0\"?>\n<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
                                "  <BehaviorTree ID=\"MainTree\">\n    " + tree_xml + "\n  </BehaviorTree>\n</root>\n");
  std::ostringstream y;
  y << "# vNext M2.6 AFTER " << id << ": trainit_run_bt params, written by test_m2_6_eef_runner_child\n"
    << "/**:\n  ros__parameters:\n"
    << "    bt_tree_file: \"" << stem << ".xml\"\n"
    << "    bt_tree_id: \"MainTree\"\n"
    << "    planner_mode: \"ompl\"\n"
    << "    process_path_backend: \"pilz_sequence\"\n"
    << "    group: \"arm\"\n"
    << "    base_frame: \"base_link\"\n"
    << "    tip_link: \"tcp\"\n"
    << "    process_controller: \"mock\"\n"
    << "    enforce_validation: true\n"
    << "    groot_monitor: false\n"
    << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
    << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6)
    << m26::indentBlock(keys, 4);
  m26::writeFile(stem + ".yaml", y.str());
  std::vector<std::string> args{"--ros-args", "--params-file", stem + ".yaml"};
  if (ns != "/")
  {
    args.push_back("-r");
    args.push_back("__ns:=" + ns);
  }
  Run r;
  {
    m26::ChildProcess child(TRAINIT_RUN_BT_EXE, args, m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), stem + ".log");
    m26::watchdog().arm(id, std::chrono::seconds(90));
    child.wait(milliseconds(80000));
    child.stop();
    m26::watchdog().disarm();
    r.exit = child.exitText();
    r.finished = child.btFinished();
  }
  std::ifstream in(stem + ".log");
  std::string line;
  while (std::getline(in, line))
  {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    r.lines.push_back(line);
  }
  return r;
}

// A canonical suction device `main` (and optionally a second device), as TSA emits the block.
std::string suctionBlock(const std::string& extra_device = "", const std::string& extra_main = "")
{
  std::string ids = extra_device.empty() ? "[\"main\"]" : "[\"main\", \"" + extra_device + "\"]";
  auto device = [](const std::string& id, const std::string& extra) {
    return "  " + id + ":\n    actuator_ids: [\"vacuum\"]\n    site_ids: [\"main\"]\n    scene_signal: \"adapter\"\n"
           "    actuators:\n      vacuum: {command_interface: \"set_bool\"}\n"
           "    sites:\n      main: {grip: \"vacuum\", engage: \"engage\", release: \"release\"}\n"
           "    verification:\n      require_grasp_confirmation: true\n      require_release_confirmation: true\n"
           "      timeout_ms: 1500\n" + extra;
  };
  std::string s = "gripper_action: \"\"\neef:\n  ids: " + ids + "\n" + device("main", extra_main);
  if (!extra_device.empty()) s += device(extra_device, "");
  return s;
}

const trainit::eef::DeviceSpec& mainSpec()
{
  static const trainit::eef::Contract c = m26::after::contractFromYaml(
    "/**:\n  ros__parameters:\n" + m26::indentBlock(suctionBlock(), 4), "m26_runner_child");   // test data
  return *c.device("main");
}

const char* kLegacyLine = "end-effector contract: LEGACY (ADR-0022)";
}  // namespace

// --- LEGACY: one INFO line, and the EEF-D3 WARN for a non-SUCCEEDED result ---------------------------
TEST(M26EefRunnerChild, legacy_runs_as_before_with_its_line_and_the_warn)
{
  m26::Log log;
  m26::ScriptedGripper shim(log, "/", "/suctioncup_controller/gripper_command",
                            {GripperReply::shimAbort(), GripperReply::succeed()});
  const auto r = runChild("C01 legacy", "/", "gripper_action: \"/suctioncup_controller/gripper_command\"\n",
                          "<Sequence><CloseGripper/><OpenGripper/></Sequence>");
  EXPECT_EQ(r.exit, "exit code 0") << r.dump();
  EXPECT_EQ(r.finished, "SUCCESS");
  EXPECT_EQ(r.count(kLegacyLine), 1) << "ONE line per run naming the contract";
  EXPECT_FALSE(r.contains("end-effector contract: canonical"));
  EXPECT_EQ(r.count("gripper command position=1.00: result ABORTED is counted as done (LEGACY contract, ADR-0022)"), 1)
    << r.dump();
  EXPECT_FALSE(r.contains("position=0.00: result")) << "no WARN for a SUCCEEDED result";
  EXPECT_EQ(r.count("gripper command position=1.00 done"), 1) << "the legacy INFO line follows the WARN, unchanged";
}

// --- exit 2: a malformed contract, refused before any MoveIt client exists ------------------------------
TEST(M26EefRunnerChild, a_malformed_contract_exits_2_naming_the_key)
{
  struct Case
  {
    std::string id, keys, line;
  };
  // command_echo is never accepted evidence (EEF-D8)
  std::string echo = suctionBlock();
  const std::string at = "      timeout_ms: 1500\n";
  echo.replace(echo.find(at), at.size(), at + "      accepted_evidence: [\"sensor\", \"command_echo\"]\n");
  const std::vector<Case> cases{
    {"C02 unknown key", suctionBlock("", "    colour: \"red\"\n"),
     "end-effector contract REFUSED eef.main.colour: unknown key (ADR-0022)"},
    {"C03 legacy key next to the block", suctionBlock() + "gripper_cmd_topic: \"/gripper_cmd\"\n",
     "end-effector contract REFUSED gripper_cmd_topic: a legacy key next to an eef block (ADR-0022)"},
    {"C04 command_echo accepted", echo,
     "end-effector contract REFUSED eef.main.verification.accepted_evidence: unique values of "
     "('sensor', 'sim_ground_truth', 'actuation_inference') (ADR-0022)"},
    {"C05 an axis without goal_tolerance",
     "gripper_action: \"\"\neef:\n  ids: [\"tool\"]\n  tool:\n    actuator_ids: [\"slide\", \"clamp\"]\n    site_ids: [\"main\"]\n"
     "    scene_signal: \"none\"\n    actuators:\n      slide:\n        command_interface: \"follow_joint_trajectory\"\n"
     "        action: \"slide_controller/follow_joint_trajectory\"\n        joints: [\"slide_joint\"]\n"
     "        targets: {home: [0.0], out: [0.05]}\n        duration_s: 1.0\n      clamp: {command_interface: \"set_bool\"}\n"
     "    sites:\n      main: {grip: \"clamp\", engage: \"engage\", release: \"release\"}\n"
     "    verification:\n      require_grasp_confirmation: false\n      require_release_confirmation: false\n",
     "end-effector contract REFUSED eef.tool.actuators.slide.goal_tolerance: required (ADR-0022)"},
  };
  for (const auto& c : cases)
  {
    const auto r = runChild(c.id, "/", c.keys, "<CloseGripper/>");
    EXPECT_EQ(r.exit, "exit code 2") << c.id << "\n" << r.dump();
    EXPECT_EQ(r.count(c.line), 1) << c.id << "\n" << r.dump();
    EXPECT_EQ(r.finished, "") << c.id << ": no tick";
    EXPECT_FALSE(r.contains("end-effector contract: canonical")) << c.id;
    EXPECT_FALSE(r.contains(kLegacyLine)) << c.id << ": a refused contract is neither";
  }
}

// --- exit 2: the tree's literal ports, and SetGripper ----------------------------------------------
TEST(M26EefRunnerChild, a_tree_the_contract_does_not_know_exits_2)
{
  struct Case
  {
    std::string id, tree, problem;
  };
  const std::vector<Case> cases{
    {"C06 unknown site", "<CloseGripper eef=\"main\" site=\"nope\"/>", "site=\"nope\": no such site of main"},
    {"C07 unknown device", "<VerifyGrasp eef=\"other\" expect=\"held\"/>", "eef=\"other\": no such device in the contract [main]"},
    {"C08 SetGripper", "<SetGripper position=\"0.5\"/>", "SetGripper is not part of the canonical end-effector contract"},
    {"C09 a bad expect", "<VerifyGrasp expect=\"gripped\"/>", "expect=\"gripped\": held | released"},
    // an int port's literal BT.CPP cannot convert is refused when the tree is built ("failed to load BT")
    {"C14 a non-integer timeout_ms", "<VerifyGrasp expect=\"held\" timeout_ms=\"fast\"/>",
     "The port with name \"timeout_ms\" and value \"fast\" can not be converted to int"},
  };
  for (const auto& c : cases)
  {
    const auto r = runChild(c.id, "/", suctionBlock(), c.tree);
    EXPECT_EQ(r.exit, "exit code 2") << c.id << "\n" << r.dump();
    EXPECT_TRUE(r.contains(c.problem)) << c.id << "\n" << r.dump();
    EXPECT_EQ(r.count("end-effector contract: canonical, devices [main] (ADR-0022)"), 1) << c.id;
    EXPECT_EQ(r.finished, "") << c.id << ": refused before the first tick";
  }
}

// --- exit 3: a tree that absorbs a FAULT -------------------------------------------------------------
TEST(M26EefRunnerChild, a_tree_that_absorbs_a_fault_exits_3)
{
  // no adapter: CloseGripper ends SERVER_MISSING after 5 s, a hand-written Fallback absorbs it
  const auto r = runChild("C10 absorbed fault", "/", suctionBlock("aux"),
                          "<Fallback><CloseGripper eef=\"main\"/><AlwaysSuccess/></Fallback>");
  EXPECT_EQ(r.exit, "exit code 3") << r.dump();
  EXPECT_EQ(r.finished, "SUCCESS");
  EXPECT_EQ(r.count("end-effector contract: canonical, devices [main, aux] (ADR-0022)"), 1) << r.dump();
  EXPECT_FALSE(r.contains(kLegacyLine));
  EXPECT_TRUE(r.contains("end-effector FAULT latched: SERVER_MISSING /eef/main/actuators/vacuum"));
  EXPECT_TRUE(r.contains("the tree ended SUCCESS with an end-effector FAULT latched: exit code 3 (ADR-0022)"));
}

// --- a canonical run in a cell namespace: exit 0, and exit 1 for a VERDICT -----------------------------
TEST(M26EefRunnerChild, a_canonical_run_in_r1_succeeds_and_a_verdict_is_exit_1)
{
  m26::Log log;
  m26::after::FakeEefAdapter adapter(log, "/r1", mainSpec());
  const auto ok = runChild("C11 canonical in r1", "/r1", suctionBlock(),
                           "<Sequence><CloseGripper/><VerifyGrasp expect=\"held\"/><OpenGripper/>"
                           "<VerifyGrasp expect=\"released\"/></Sequence>");
  EXPECT_EQ(ok.exit, "exit code 0") << ok.dump();
  EXPECT_EQ(ok.finished, "SUCCESS");
  EXPECT_EQ(ok.count("end-effector contract: canonical, devices [main] (ADR-0022)"), 1);
  const auto calls = adapter.setBool("vacuum").calls();
  ASSERT_EQ(calls.size(), 2u);
  EXPECT_TRUE(calls[0].data);
  EXPECT_FALSE(calls[1].data);
  // NO_OBJECT is a VERDICT: the tree fails, exit 1 -- not 3
  adapter.setSite("main", m26::after::SiteScript{m26::after::State::GRASP_NOT_HELD});
  const auto verdict = runChild("C12 a verdict in r1", "/r1", suctionBlock(),
                                "<Sequence><CloseGripper/><VerifyGrasp expect=\"held\"/></Sequence>");
  EXPECT_EQ(verdict.exit, "exit code 1") << verdict.dump();
  EXPECT_EQ(verdict.finished, "FAILURE");
  EXPECT_FALSE(verdict.contains("FAULT latched"));
}

// --- a tree that THROWS (a Script reading a key nobody wrote throws at tick time) -----------------------
// Canonical: the runner catches it, halts the tree (HOLD) and the run fails, exit 1 -- never
// std::terminate (ADR-0022 TMR-2). LEGACY: unchanged (ADR-0022 D9), the exception still ends the
// process as it does on the BEFORE sources, by SIGABRT and with no "BT finished:" line.
TEST(M26EefRunnerChild, a_canonical_tree_that_throws_fails_with_exit_1_legacy_is_unchanged)
{
  const std::string tree = "<Sequence><Script code=\"x := never_written + 1\"/></Sequence>";
  const auto canonical = runChild("C15 a canonical tree throws", "/", suctionBlock(), tree);
  EXPECT_EQ(canonical.exit, "exit code 1") << canonical.dump();
  EXPECT_EQ(canonical.finished, "FAILURE");
  EXPECT_TRUE(canonical.contains("the tree threw: ")) << canonical.dump();
  EXPECT_TRUE(canonical.contains("Variable not found: never_written")) << canonical.dump();
  EXPECT_TRUE(canonical.contains("it is halted and the run fails (ADR-0022)")) << canonical.dump();
  const auto legacy = runChild("C16 a legacy tree throws", "/", "gripper_action: \"\"\n", tree);
  EXPECT_EQ(legacy.exit, "killed by signal 6") << legacy.dump();
  EXPECT_EQ(legacy.finished, "") << "no BT finished line, as before";
  EXPECT_EQ(legacy.count(kLegacyLine), 1);
  EXPECT_FALSE(legacy.contains("the tree threw: ")) << "the canonical catch never runs for a legacy project";
}

// --- exit 2: two writers on a device's state topic, checked at start-up ---------------------------------
TEST(M26EefRunnerChild, two_writers_on_a_state_topic_exit_2)
{
  m26::Log log;
  m26::after::FakeEefAdapter first(log, "/r2", mainSpec());
  m26::after::AdapterOptions o;
  o.serve = false;
  o.node_name = "m26_second_writer";
  m26::after::FakeEefAdapter second(log, "/r2", mainSpec(), o);
  const auto r = runChild("C13 two writers", "/r2", suctionBlock(), "<VerifyGrasp expect=\"released\"/>");
  EXPECT_EQ(r.exit, "exit code 2") << r.dump();
  EXPECT_TRUE(r.contains("/r2/eef/main: 2 publishers on '/r2/eef/main/state'; ONE adapter writes a device's state"))
    << r.dump();
  EXPECT_EQ(r.finished, "");
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
