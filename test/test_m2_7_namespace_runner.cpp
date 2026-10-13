// vNext M2.7 (WP_NS, owner section 12, tests T7 and T9): the REAL trainit_run_bt as child processes
// -- ONE PROCESS PER ROBOT, because MoveIt caches the robot model and the current-state monitor
// process-wide -- against the M2.5 fake move_group served in EVERY namespace under test (the root,
// /r1, /r2, /r3), each fake with its own log, so a request is attributed to the namespace that
// received it. Each child runs one planned move (MoveToNamed "ready").
//
//   T7  root, /r1, /r2, /r3 one at a time, then /r1 + /r2 at once: every MoveIt request a runner
//       sends lands in ITS namespace (move_action, get_planning_scene, execute_trajectory), every
//       other fake sees nothing, each child exits 0 with 'BT finished: SUCCESS', no 'MoveIt
//       start-up' line (its move_group is up), and the eight MoveGroupInterface endpoints of the
//       child's node (from the graph) are all under its namespace. The root case is the legacy
//       check at runner level (root names, root fake).
//   T9  a runner in /r1 while move_group serves only the root, /r2 and /r3: the runners' MoveIt
//       start-up wait (owner decision D1) names /r1/... (WARN at 5 s, FATAL at the bound), exit 2,
//       and no fake received a request. A remap that sends one of /r1's MoveIt or scene services to
//       /r2 (another robot's live move_group) is refused at once, before any MoveIt client exists
//       (ADR-0023 Decision 5, the cross-namespace remap guard NS-G): exit 2, a FATAL naming
//       "service /r1/<id> -> /r2/<id>", every fake untouched. A remap of a robot topic, of a
//       PlanningSceneMonitor subscription or of one of an action's five internal entities is refused
//       the same way (Humble applies no remap of an action NAME, but one of each entity, which would
//       send /r1's MoveGroup goals to /r2's move_group): "topic /r1/<id> -> /r2/<id>", "service
//       /r1/<action>/_action/<entity> -> /r2/...". A remap that stays inside /r1 is not one. The
//       guard's whole list (every robot-scoped name the runner's MoveIt and scene clients resolve)
//       is pinned in-process, with the root and an action-name remap left alone.
//   G0  (the release preparation, D-GUARD) trainit_run_bt checks more, once it has read its
//       end-effector contract: ServoTrack's DEFAULT servo node (servo_node/delta_twist_cmds, status,
//       start_servo, pause_servo, unpause_servo) and every name the contract gives its devices
//       (eef/<id>/state, the TMR scene signal eef/<id>/engaged, a SetBool service, an action's five
//       entities). A LEGACY child whose tree runs a ServoTrack, its twists or its start_servo remapped
//       onto /r2, and a canonical child with an action's send_goal, a SetBool service, the scene
//       signal, an action's status or a state topic remapped onto /r2, are refused at start-up with
//       the same FATAL and exit 2: no request reaches either namespace's fakes. A remap that stays
//       inside /r1 is not refused. trainit_run_bt's whole list (51 names for the two-device test
//       contract) is pinned in-process beside trainit_run_task's 32, and two graph censuses bind it
//       to what the registry and ServoTrack create.
#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "m2_5/fake_move_group.hpp"
#include "m2_5/test_arm.hpp"
#include "m2_6/child_process.hpp"
#include "m2_6/eef_fakes.hpp"
#include "m2_6/eef_after_fakes.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"

namespace
{
using m26::milliseconds;

const std::vector<std::string> kSpaces{"/", "/r1", "/r2", "/r3"};

std::string actualDir() { return std::string(TRAINIT_M26_ACTUAL_DIR) + "/m2_7_namespace_runner"; }

// The fakes of every namespace, with joint states, one log each (one set per process).
struct World
{
  World()
  {
    for (const auto& ns : kSpaces)
    {
      logs[ns] = std::make_unique<m25::Log>();
      logs[ns]->setBodies(false);
      fakes[ns] = std::make_unique<m25::FakeMoveGroup>(*logs[ns], ns);
      js[ns] = std::make_unique<m25::FakeJointStates>(ns, std::vector<double>{0.0, 0.0, 0.0});
    }
    probe = std::make_shared<rclcpp::Node>("m27_probe", "/", rclcpp::NodeOptions().use_global_arguments(false));
  }
  void reset()
  {
    for (auto& [ns, f] : fakes) if (f) f->resetScenario();
    for (auto& [ns, l] : logs) l->reset();
  }
  std::map<std::string, std::unique_ptr<m25::Log>> logs;
  std::map<std::string, std::unique_ptr<m25::FakeMoveGroup>> fakes;
  std::map<std::string, std::unique_ptr<m25::FakeJointStates>> js;
  rclcpp::Node::SharedPtr probe;
};

World& world()
{
  static std::unique_ptr<World> w;   // test harness: one per process
  if (!w)
  {
    w = std::make_unique<World>();
    m26::teardowns().push_back([] { w.reset(); });
  }
  return *w;
}

// One namespace's move_group withdrawn for a scope (its joint states and its log stay).
struct Withdrawn
{
  explicit Withdrawn(std::string ns) : ns_(std::move(ns)) { world().fakes[ns_].reset(); }
  ~Withdrawn() { world().fakes[ns_] = std::make_unique<m25::FakeMoveGroup>(*world().logs[ns_], ns_); }
  Withdrawn(const Withdrawn&) = delete;
  Withdrawn& operator=(const Withdrawn&) = delete;
  std::string ns_;
};

struct Child
{
  std::string ns;
  std::vector<std::string> extra_args{};   // e.g. a remap
  std::string extra_keys{};               // YAML lines under ros__parameters
  std::string tree{"<MoveToNamed target=\"ready\"/>"};   // the MainTree's one child
};

struct Ran
{
  std::string exit, finished;
  std::vector<std::string> lines;
  std::set<std::string> mgi;   // MoveGroupInterface endpoints of the child's node, from the graph
  int count(const std::string& text) const
  {
    int n = 0;
    for (const auto& l : lines) if (l.find(text) != std::string::npos) ++n;
    return n;
  }
  std::string dump() const { std::string s; for (const auto& l : lines) s += l + "\n"; return s; }
};

// "action /r1/move_action" for the action's internal names, "service ..." / "publisher ..." else.
std::set<std::string> mgiEndpoints(const rclcpp::Node::SharedPtr& probe, const std::string& ns)
{
  static const std::vector<std::string> kServices{"query_planner_interface", "get_planner_params",
                                                  "set_planner_params", "compute_cartesian_path"};
  static const std::vector<std::string> kPublishers{"trajectory_execution_event", "attached_collision_object"};
  std::set<std::string> out;
  try
  {
    auto g = probe->get_node_graph_interface();
    for (const auto& [name, types] : g->get_client_names_and_types_by_node("trainit_run_bt", ns))
    {
      for (const char* a : {"move_action", "execute_trajectory"})
      {
        const std::string tail = std::string("/") + a + "/_action/send_goal";
        if (name.size() >= tail.size() && name.compare(name.size() - tail.size(), tail.size(), tail) == 0)
          out.insert("action " + name.substr(0, name.size() - std::string("/_action/send_goal").size()));
      }
      for (const auto& s : kServices)
        if (name.size() > s.size() && name.compare(name.size() - s.size(), s.size(), s) == 0) out.insert("service " + name);
    }
    for (const auto& [name, types] : g->get_publisher_names_and_types_by_node("trainit_run_bt", ns))
      for (const auto& p : kPublishers)
        if (name.size() > p.size() && name.compare(name.size() - p.size(), p.size(), p) == 0) out.insert("publisher " + name);
  }
  catch (const std::exception&) {}   // the node is gone
  return out;
}

// `poll`, when set, runs on every graph poll while the children run (a census of their names).
std::vector<Ran> runChildren(const std::string& id, const std::vector<Child>& children,
                             const std::function<void()>& poll = {})
{
  ::mkdir(actualDir().c_str(), 0755);
  World& w = world();
  w.reset();
  std::vector<std::unique_ptr<m26::ChildProcess>> kids;
  std::vector<Ran> out(children.size());
  for (std::size_t i = 0; i < children.size(); ++i)
  {
    const Child& c = children[i];
    const std::string stem = actualDir() + "/" + m26::tagOf(id) + m26::tagOf(c.ns == "/" ? "_root" : c.ns);
    m26::writeFile(stem + ".xml", "<?xml version=\"1.0\"?>\n<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
                                  "  <BehaviorTree ID=\"MainTree\">\n    " + c.tree + "\n  </BehaviorTree>\n</root>\n");
    std::ostringstream y;
    y << "# vNext M2.7 " << id << ": trainit_run_bt params, written by test_m2_7_namespace_runner\n"
      << "/**:\n  ros__parameters:\n"
      << "    bt_tree_file: \"" << stem << ".xml\"\n    bt_tree_id: \"MainTree\"\n    planner_mode: \"ompl\"\n"
      << "    process_path_backend: \"pilz_sequence\"\n    group: \"arm\"\n    base_frame: \"base_link\"\n"
      << "    tip_link: \"tcp\"\n    process_controller: \"mock\"\n    enforce_validation: true\n    groot_monitor: false\n"
      << "    robot_description: |\n" << m26::indentBlock(m25::kUrdf, 6)
      << "    robot_description_semantic: |\n" << m26::indentBlock(m25::kSrdf, 6)
      << m26::indentBlock(c.extra_keys, 4);
    m26::writeFile(stem + ".yaml", y.str());
    std::vector<std::string> args{"--ros-args", "--params-file", stem + ".yaml"};
    if (c.ns != "/") { args.push_back("-r"); args.push_back("__ns:=" + c.ns); }
    args.insert(args.end(), c.extra_args.begin(), c.extra_args.end());
    kids.push_back(std::make_unique<m26::ChildProcess>(TRAINIT_RUN_BT_EXE, args,
                                                       m26::childEnvironment(TRAINIT_M26_DOMAIN, actualDir()), stem + ".log"));
  }
  m26::watchdog().arm(id, std::chrono::seconds(120));
  for (;;)   // poll the graph while they run: the children's MoveGroupInterface endpoints
  {
    bool any = false;
    for (std::size_t i = 0; i < kids.size(); ++i)
    {
      for (const auto& e : mgiEndpoints(w.probe, children[i].ns)) out[i].mgi.insert(e);
      if (kids[i]->running()) any = true;
    }
    if (poll) poll();
    if (!any) break;
    std::this_thread::sleep_for(milliseconds(100));
  }
  m26::watchdog().disarm();
  for (std::size_t i = 0; i < kids.size(); ++i)
  {
    out[i].exit = kids[i]->exitText();
    out[i].finished = kids[i]->btFinished();
    std::ifstream in(kids[i]->output());
    std::string line;
    while (std::getline(in, line)) out[i].lines.push_back(line);
  }
  return out;
}

// What a runtime in `ns` must use: the eight names under it.
std::set<std::string> expectedMgi(const std::string& ns)
{
  const std::string p = ns == "/" ? "" : ns;
  return {"action " + p + "/execute_trajectory", "action " + p + "/move_action",
          "publisher " + p + "/attached_collision_object", "publisher " + p + "/trajectory_execution_event",
          "service " + p + "/compute_cartesian_path", "service " + p + "/get_planner_params",
          "service " + p + "/query_planner_interface", "service " + p + "/set_planner_params"};
}

// The ordered request summaries ONE planned move sends, all in `ns`.
std::string expectedEvents(const std::string& ns)
{
  const std::string p = ns == "/" ? "" : ns;
  return "[1] SERVICE " + p + "/get_planning_scene GetPlanningScene.request components=1023\n"   // the monitor's start-up request
         "[2] ACTION " + p + "/move_action MoveGroup.goal #1 -> SUCCEED(1)\n"
         "[3] SERVICE " + p + "/get_planning_scene GetPlanningScene.request components=1023\n"   // the validation snapshot
         "[4] ACTION " + p + "/execute_trajectory ExecuteTrajectory.goal -> SUCCEED(1)\n";
}

std::string summaries(const m25::Log& log)
{
  std::string s;
  std::istringstream in(log.events());
  std::string line;
  while (std::getline(in, line)) if (line.rfind("[", 0) == 0) s += line + "\n";
  return s;
}

// What the guard reports for a node in `ns` whose OWN arguments carry `rules` (no global arguments,
// no rosout, no parameter services: the probe adds nothing to the graph but its name).
std::vector<std::string> guardOf(const std::string& ns, const std::vector<std::string>& rules)
{
  std::vector<std::string> args{"--ros-args"};
  for (const auto& r : rules) { args.push_back("-r"); args.push_back(r); }
  auto node = std::make_shared<rclcpp::Node>(
    "m27_guard_probe", ns,
    rclcpp::NodeOptions().arguments(args).use_global_arguments(false).enable_rosout(false)
                         .start_parameter_services(false).start_parameter_event_publisher(false));
  return trainit::MoveItMotionRuntime::crossNamespaceRemaps(node);
}

// The same through trainit_run_bt's overload: the 32, ServoTrack's default servo node, `contract`'s names.
std::vector<std::string> guardOfBt(const std::string& ns, const std::vector<std::string>& rules,
                                   const trainit::eef::Contract& contract)
{
  std::vector<std::string> args{"--ros-args"};
  for (const auto& r : rules) { args.push_back("-r"); args.push_back(r); }
  auto node = std::make_shared<rclcpp::Node>(
    "m27_guard_probe", ns,
    rclcpp::NodeOptions().arguments(args).use_global_arguments(false).enable_rosout(false)
                         .start_parameter_services(false).start_parameter_event_publisher(false));
  return trainit::MoveItMotionRuntime::crossNamespaceRemaps(node, contract);
}

using Names = std::vector<std::pair<std::string, std::string>>;   // {kind, relative id}

// An action's five entities, in the guard's order.
void addActionNames(Names& all, const std::string& action)
{
  for (const auto& [kind, entity] : Names{{"service", "send_goal"}, {"service", "cancel_goal"},
                                          {"service", "get_result"}, {"topic", "feedback"}, {"topic", "status"}})
    all.push_back({kind, action + "/_action/" + entity});
}

// The 32 names both runners check, in the guard's order.
Names baseNames()
{
  Names all{
    {"service", "compute_cartesian_path"}, {"service", "query_planner_interface"}, {"service", "get_planner_params"},
    {"service", "set_planner_params"}, {"service", "get_planning_scene"}, {"service", "apply_planning_scene"},
    {"service", "scene_manager_node/attached_collision_check"}, {"service", "scene_manager_node/reset_scene"},
    {"topic", "planning_scene"}, {"topic", "joint_states"}, {"topic", "trajectory_execution_event"},
    {"topic", "attached_collision_object"}, {"topic", "collision_object"}, {"topic", "planning_scene_world"},
    {"topic", "monitored_planning_scene"}, {"service", "save_map"}, {"service", "load_map"}};
  for (const char* action : {"move_action", "execute_trajectory", "sequence_move_group"}) addActionNames(all, action);
  return all;
}

// ServoTrack's default servo node, as trainit_run_bt's guard checks it after the 32.
Names servoNames()
{
  return {{"topic", "servo_node/delta_twist_cmds"}, {"topic", "servo_node/status"},
          {"service", "servo_node/start_servo"}, {"service", "servo_node/pause_servo"},
          {"service", "servo_node/unpause_servo"}};
}

// The test contract's end-effector names, in the guard's order: per device its state (and its scene
// signal when TMR publishes it), then per actuator its SetBool service or its action's five entities.
Names eefNames()
{
  Names all{{"topic", "eef/jaw/state"}, {"topic", "eef/jaw/engaged"}};
  addActionNames(all, "jaw_controller/gripper_cmd");
  all.push_back({"topic", "eef/tool/state"});
  all.push_back({"service", "eef/tool/actuators/vacuum/set_engaged"});
  addActionNames(all, "slide_controller/follow_joint_trajectory");
  return all;
}

// "<kind> /r1/<id>" for each.
std::set<std::string> inR1(const Names& names)
{
  std::set<std::string> out;
  for (const auto& [kind, rel] : names) out.insert(kind + " /r1/" + rel);
  return out;
}

// The two-device contract of the G0 cases (D-GUARD 4.1). jaw: one gripper_command actuator gripping its
// one site, TMR publishes its scene signal. tool: a set_bool actuator gripping its one site and a
// follow_joint_trajectory slide, no scene signal. kTestEefYaml is the same block as params-file text.
trainit::eef::Contract testContract()
{
  using PV = rclcpp::ParameterValue;
  using Strs = std::vector<std::string>;
  using S = std::string;
  const trainit::eef::ParameterMap p{
    {"eef.ids", PV(Strs{"jaw", "tool"})},
    {"eef.jaw.actuator_ids", PV(Strs{"gripper"})},
    {"eef.jaw.site_ids", PV(Strs{"main"})},
    {"eef.jaw.scene_signal", PV(S("tmr"))},
    {"eef.jaw.actuators.gripper.command_interface", PV(S("gripper_command"))},
    {"eef.jaw.actuators.gripper.action", PV(S("jaw_controller/gripper_cmd"))},
    {"eef.jaw.actuators.gripper.joints", PV(Strs{"finger_joint"})},
    {"eef.jaw.actuators.gripper.targets.open", PV(0.0)},
    {"eef.jaw.actuators.gripper.targets.closed", PV(0.7)},
    {"eef.jaw.actuators.gripper.result_policy", PV(S("succeeded"))},
    {"eef.jaw.sites.main.grip", PV(S("gripper"))},
    {"eef.jaw.sites.main.engage", PV(S("closed"))},
    {"eef.jaw.sites.main.release", PV(S("open"))},
    {"eef.jaw.verification.require_grasp_confirmation", PV(false)},
    {"eef.jaw.verification.require_release_confirmation", PV(false)},
    {"eef.tool.actuator_ids", PV(Strs{"vacuum", "slide"})},
    {"eef.tool.site_ids", PV(Strs{"cup"})},
    {"eef.tool.scene_signal", PV(S("none"))},
    {"eef.tool.actuators.vacuum.command_interface", PV(S("set_bool"))},
    {"eef.tool.actuators.slide.command_interface", PV(S("follow_joint_trajectory"))},
    {"eef.tool.actuators.slide.action", PV(S("slide_controller/follow_joint_trajectory"))},
    {"eef.tool.actuators.slide.joints", PV(Strs{"slide_joint"})},
    {"eef.tool.actuators.slide.targets.home", PV(std::vector<double>{0.0})},
    {"eef.tool.actuators.slide.targets.out", PV(std::vector<double>{0.05})},
    {"eef.tool.actuators.slide.goal_tolerance", PV(0.001)},
    {"eef.tool.actuators.slide.duration_s", PV(0.2)},
    {"eef.tool.sites.cup.grip", PV(S("vacuum"))},
    {"eef.tool.sites.cup.engage", PV(S("engage"))},
    {"eef.tool.sites.cup.release", PV(S("release"))},
    {"eef.tool.verification.require_grasp_confirmation", PV(false)},
    {"eef.tool.verification.require_release_confirmation", PV(false)}};
  return trainit::eef::parseContract(p);
}

const std::string kTestEefYaml =
  "eef:\n"
  "  ids: [\"jaw\", \"tool\"]\n"
  "  jaw:\n"
  "    actuator_ids: [\"gripper\"]\n"
  "    site_ids: [\"main\"]\n"
  "    scene_signal: \"tmr\"\n"
  "    actuators:\n"
  "      gripper:\n"
  "        command_interface: \"gripper_command\"\n"
  "        action: \"jaw_controller/gripper_cmd\"\n"
  "        joints: [\"finger_joint\"]\n"
  "        targets: {open: 0.0, closed: 0.7}\n"
  "        result_policy: \"succeeded\"\n"
  "    sites:\n"
  "      main: {grip: \"gripper\", engage: \"closed\", release: \"open\"}\n"
  "    verification: {require_grasp_confirmation: false, require_release_confirmation: false}\n"
  "  tool:\n"
  "    actuator_ids: [\"vacuum\", \"slide\"]\n"
  "    site_ids: [\"cup\"]\n"
  "    scene_signal: \"none\"\n"
  "    actuators:\n"
  "      vacuum: {command_interface: \"set_bool\"}\n"
  "      slide:\n"
  "        command_interface: \"follow_joint_trajectory\"\n"
  "        action: \"slide_controller/follow_joint_trajectory\"\n"
  "        joints: [\"slide_joint\"]\n"
  "        targets: {home: [0.0], out: [0.05]}\n"
  "        goal_tolerance: 0.001\n"
  "        duration_s: 0.2\n"
  "    sites:\n"
  "      cup: {grip: \"vacuum\", engage: \"engage\", release: \"release\"}\n"
  "    verification: {require_grasp_confirmation: false, require_release_confirmation: false}\n";

const std::string kTestEefLine = "end-effector contract: canonical, devices [jaw, tool] (ADR-0022)";

// ServoTrack on the default servo node, and its waypoint: a cartesian target the fake arm never reaches.
const std::string kServoTrack = "<ServoTrack waypoint=\"t\" timeout_ms=\"300\"/>";
const std::string kServoTask = "task_parameters:\n  t:\n    position: \"0.4;0.0;0.4\"\n";

// The guard's FATAL for ONE name of /r1 remapped onto /r2.
std::string fatalOf(const std::string& kind, const std::string& rel)
{
  return "MoveIt start-up: a remap sends this robot's resources out of its namespace /r1: " + kind + " /r1/" +
         rel + " -> /r2/" + rel + " (ADR-0023: a cross-namespace remap) -- exit 2";
}

// The names `node` (in `ns`) has under `prefix`, from the graph: its publishers and subscriptions
// ("topic <name>") and its service clients ("service <name>"); what is there so far once it is gone.
std::set<std::string> namesOf(const rclcpp::Node::SharedPtr& probe, const std::string& node, const std::string& ns,
                              const std::string& prefix)
{
  std::set<std::string> out;
  try
  {
    auto g = probe->get_node_graph_interface();
    for (const auto& [name, types] : g->get_publisher_names_and_types_by_node(node, ns))
      if (name.rfind(prefix, 0) == 0) out.insert("topic " + name);
    for (const auto& [name, types] : g->get_subscriber_names_and_types_by_node(node, ns))
      if (name.rfind(prefix, 0) == 0) out.insert("topic " + name);
    for (const auto& [name, types] : g->get_client_names_and_types_by_node(node, ns))
      if (name.rfind(prefix, 0) == 0) out.insert("service " + name);
  }
  catch (const std::exception&) {}   // the node is gone
  return out;
}

// What trainit_run_bt's guard names (its "<kind> /r1/<id>" part) when each of `names` ("<kind> /r1/<id>")
// is remapped onto /r2 at once.
std::set<std::string> guardedOf(const std::set<std::string>& names, const trainit::eef::Contract& contract)
{
  std::vector<std::string> rules;
  for (const auto& n : names)
  {
    const std::string rel = n.substr(n.find(" /r1/") + std::string(" /r1/").size());
    rules.push_back(rel + ":=/r2/" + rel);
  }
  std::set<std::string> out;
  for (const auto& e : guardOfBt("/r1", rules, contract)) out.insert(e.substr(0, e.find(" -> ")));
  return out;
}

std::vector<std::string> endpointText(const std::vector<trainit::EefEndpoint>& endpoints)
{
  using Kind = trainit::EefEndpoint::Kind;
  std::vector<std::string> out;
  for (const auto& e : endpoints)
    out.push_back(std::string(e.kind == Kind::TOPIC ? "TOPIC " : e.kind == Kind::SERVICE ? "SERVICE " : "ACTION ") +
                  e.name);
  return out;
}
}  // namespace

// --- T7: one robot at a time, in every namespace ---------------------------------------------------
TEST(M27NamespaceRunner, each_namespace_alone_reaches_its_own_move_group)
{
  for (const auto& ns : kSpaces)
  {
    const auto r = runChildren("NSR " + ns, {{ns}});
    ASSERT_EQ(r.size(), 1u);
    EXPECT_EQ(r[0].exit, "exit code 0") << ns << "\n" << r[0].dump();
    EXPECT_EQ(r[0].finished, "SUCCESS") << ns;
    EXPECT_EQ(r[0].mgi, expectedMgi(ns)) << ns << ": the child's MoveGroupInterface endpoints";
    EXPECT_EQ(r[0].count("MoveIt start-up"), 0) << ns << ": no start-up line while MoveIt is up";
    for (const auto& other : kSpaces)
      EXPECT_EQ(summaries(*world().logs[other]), other == ns ? expectedEvents(ns) : std::string())
        << "runner in " << ns << ", fake in " << other;
  }
}

// --- T7: two robots at once ---------------------------------------------------------------------------
TEST(M27NamespaceRunner, r1_and_r2_at_once_never_share_a_move_group)
{
  const auto r = runChildren("NSR r1+r2", {{"/r1"}, {"/r2"}});
  ASSERT_EQ(r.size(), 2u);
  for (std::size_t i = 0; i < 2; ++i)
  {
    const std::string ns = i == 0 ? "/r1" : "/r2";
    EXPECT_EQ(r[i].exit, "exit code 0") << ns << "\n" << r[i].dump();
    EXPECT_EQ(r[i].finished, "SUCCESS") << ns;
    EXPECT_EQ(r[i].mgi, expectedMgi(ns)) << ns;
  }
  EXPECT_EQ(summaries(*world().logs["/r1"]), expectedEvents("/r1"));
  EXPECT_EQ(summaries(*world().logs["/r2"]), expectedEvents("/r2"));
  EXPECT_EQ(summaries(*world().logs["/"]), "") << "no namespaced runner reaches the root move_group";
  EXPECT_EQ(summaries(*world().logs["/r3"]), "");
}

// --- T9: a move_group in the wrong namespace fails loudly and finitely ---------------------------------
TEST(M27NamespaceRunner, a_runner_whose_move_group_is_elsewhere_names_what_it_waits_for_and_exits_2)
{
  // /r1 runner; move_group (fake) in /, /r2 and /r3 only
  Withdrawn r1_move_group("/r1");
  const auto r = runChildren("NSX wrong ns", {{"/r1", {}, "moveit_wait_timeout_s: 6\n"}});
  const std::string missing = "action /r1/move_action, action /r1/execute_trajectory, "
                              "service /r1/get_planning_scene, service /r1/apply_planning_scene";
  EXPECT_EQ(r[0].exit, "exit code 2") << r[0].dump();
  EXPECT_EQ(r[0].finished, "");
  EXPECT_EQ(r[0].count("MoveIt start-up: still waiting after 5 s for " + missing + " (namespace /r1, moveit_wait_timeout_s 6)"), 1)
    << r[0].dump();
  EXPECT_EQ(r[0].count("MoveIt start-up: not served after 6 s: " + missing + " (namespace /r1;"), 1) << r[0].dump();
  for (const auto& ns : {"/", "/r2", "/r3"}) EXPECT_EQ(summaries(*world().logs[ns]), "") << ns;

  // the scene services remapped onto /r2's live move_group: refused at once by the cross-namespace
  // remap guard (NS-G), before the wait -- WP_NS's prototype, which had no guard, waited and named
  // only the two actions as missing
  const auto x = runChildren("NSX scene remap", {{"/r1",
                                                  {"-r", "get_planning_scene:=/r2/get_planning_scene",
                                                   "-r", "apply_planning_scene:=/r2/apply_planning_scene"},
                                                  "moveit_wait_timeout_s: 6\n"}});
  EXPECT_EQ(x[0].exit, "exit code 2") << x[0].dump();
  EXPECT_EQ(x[0].finished, "");
  EXPECT_EQ(x[0].count("MoveIt start-up: a remap sends this robot's resources out of its namespace /r1: "
                       "service /r1/get_planning_scene -> /r2/get_planning_scene, "
                       "service /r1/apply_planning_scene -> /r2/apply_planning_scene "
                       "(ADR-0023: a cross-namespace remap) -- exit 2"), 1) << x[0].dump();
  EXPECT_EQ(x[0].count("still waiting"), 0) << "refused before the wait";
  for (const auto& ns : kSpaces) EXPECT_EQ(summaries(*world().logs[ns]), "") << ns;
}

// --- T9 / NS-G: a cross-namespace remap of a robot service fails loudly, at once ----------------------
TEST(M27NamespaceRunner, a_cross_namespace_remap_of_a_robot_service_fails_loudly)
{
  // child /r1, move_group served in /r1 AND /r2 (and / and /r3): one remap onto /r2 each
  const std::vector<std::pair<std::string, std::string>> cases{
    {"CR-02", "compute_cartesian_path"},   // a MoveGroupInterface service
    {"CR-03", "get_planning_scene"},   // the PlanningSceneInterface / monitor scene service
    {"CR-04", "scene_manager_node/attached_collision_check"}};   // a TMR scene client
  for (const auto& [id, rel] : cases)
  {
    const auto r = runChildren("NSG " + id, {{"/r1", {"-r", rel + ":=/r2/" + rel}}});
    EXPECT_EQ(r[0].exit, "exit code 2") << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].finished, "") << id;
    EXPECT_EQ(r[0].count("MoveIt start-up: a remap sends this robot's resources out of its namespace /r1: service /r1/" +
                         rel + " -> /r2/" + rel + " (ADR-0023: a cross-namespace remap) -- exit 2"), 1)
      << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].count("still waiting"), 0) << id << ": refused before the wait";
    EXPECT_EQ(r[0].count("Loading robot model"), 0) << id << ": refused before any MoveIt client exists";
    for (const auto& ns : kSpaces) EXPECT_EQ(summaries(*world().logs[ns]), "") << id << ", fake in " << ns;
  }
  // control: a remap that stays inside /r1 is not a cross-namespace remap
  const auto ok = runChildren("NSG control", {{"/r1", {"-r", "compute_cartesian_path:=/r1/m27_cartesian_alt"}}});
  EXPECT_EQ(ok[0].exit, "exit code 0") << ok[0].dump();
  EXPECT_EQ(ok[0].finished, "SUCCESS");
  EXPECT_EQ(ok[0].count("MoveIt start-up"), 0) << ok[0].dump();
  auto in_r1 = expectedMgi("/r1");
  in_r1.erase("service /r1/compute_cartesian_path");   // remapped to /r1/m27_cartesian_alt: still in /r1
  EXPECT_EQ(ok[0].mgi, in_r1) << "control: the child's other MoveGroupInterface endpoints";
  for (const auto& other : kSpaces)
    EXPECT_EQ(summaries(*world().logs[other]), other == "/r1" ? expectedEvents("/r1") : std::string())
      << "control, fake in " << other;
}

// --- T9 / NS-G: a cross-namespace remap of a robot topic or of an action's entity fails loudly --------
TEST(M27NamespaceRunner, a_cross_namespace_remap_of_a_robot_topic_or_action_entity_fails_loudly)
{
  // child /r1, move_group served in /r1 AND /r2 (and / and /r3): one remap onto /r2 each. CR-07 is
  // WP_TEST's "MoveIt action" row: Humble applies no remap of an action NAME but does apply one of an
  // internal entity, and without the guard /r1's MoveGroup goals reach /r2's move_group.
  const std::vector<std::tuple<std::string, std::string, std::string>> cases{
    {"CR-07", "service", "move_action/_action/send_goal"},   // an action's internal entity (MoveGroupInterface)
    {"CR-08", "topic", "joint_states"},                      // the robot's joint states (both state monitors)
    {"CR-09", "topic", "collision_object"}};                 // a PlanningSceneMonitor subscription
  for (const auto& [id, kind, rel] : cases)
  {
    const auto r = runChildren("NSG " + id, {{"/r1", {"-r", rel + ":=/r2/" + rel}}});
    EXPECT_EQ(r[0].exit, "exit code 2") << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].finished, "") << id;
    EXPECT_EQ(r[0].count("MoveIt start-up: a remap sends this robot's resources out of its namespace /r1: " + kind +
                         " /r1/" + rel + " -> /r2/" + rel + " (ADR-0023: a cross-namespace remap) -- exit 2"), 1)
      << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].count("still waiting"), 0) << id << ": refused before the wait";
    EXPECT_EQ(r[0].count("Loading robot model"), 0) << id << ": refused before any MoveIt client exists";
    EXPECT_TRUE(r[0].mgi.empty()) << id << ": no MoveGroupInterface endpoint ever appeared in the graph";
    for (const auto& ns : kSpaces)
    {
      EXPECT_EQ(summaries(*world().logs[ns]), "") << id << ", fake in " << ns;
      EXPECT_EQ(world().logs[ns]->topics(), "") << id << ", fake topics in " << ns;
    }
  }
}

// --- NS-G: the 32 names both runners check, in-process (no child) --------------------------------------
// Every robot-scoped name a runner's MoveIt and scene clients resolve, in the guard's order (ADR-0023
// Decision 5; MoveItMotionRuntime::crossNamespaceRemaps says where each comes from), each cross-remapped
// onto /r2 at once: the guard names every one. They are trainit_run_task's whole list; trainit_run_bt
// checks more after them (the next test). The same rules at the root, remaps that stay inside /r1
// and remaps of an action NAME (which Humble does not apply: the entities stay in /r1) are not refused.
// monitored_planning_scene is among them although this file's children never publish it: MoveIt starts
// that publisher from the publish_planning_scene parameter, which every generated runner gets and these
// children's params do not carry.
TEST(M27NamespaceRunner, the_guard_names_every_robot_scoped_name_and_only_under_a_namespace)
{
  const Names all = baseNames();
  std::vector<std::string> rules, expected;
  for (const auto& [kind, rel] : all)
  {
    rules.push_back(rel + ":=/r2/" + rel);
    expected.push_back(kind + " /r1/" + rel + " -> /r2/" + rel);
  }
  ASSERT_EQ(all.size(), 32u);
  EXPECT_EQ(guardOf("/r1", rules), expected);
  EXPECT_EQ(guardOf("/", rules), std::vector<std::string>{}) << "the root is not checked";
  EXPECT_EQ(guardOf("/r1", {"collision_object:=/r1/m27_alt_collision_object",
                            "move_action/_action/send_goal:=/r1/m27_alt/_action/send_goal"}),
            std::vector<std::string>{}) << "a remap that stays inside /r1 is not one";
  EXPECT_EQ(guardOf("/r1", {"move_action:=/r2/move_action", "execute_trajectory:=/r2/execute_trajectory",
                            "sequence_move_group:=/r2/sequence_move_group"}),
            std::vector<std::string>{}) << "an action NAME remap, which Humble does not apply";
}

// --- G0 (D-GUARD), T-G1: trainit_run_bt's whole list, in-process (no child) --------------------------------
// After the 32: ServoTrack's DEFAULT servo node, checked whether or not the tree runs a ServoTrack, then
// every name the end-effector contract gives its devices -- 51 names for the two-device test contract --
// each cross-remapped onto /r2 at once: the guard names every one, in order. At the root nothing is
// checked; a LEGACY contract adds no end-effector name; trainit_run_task's list (the 32) ignores the servo
// and end-effector names by design; remaps that stay inside /r1, action NAME remaps (which Humble does not
// apply) and a servo node a tree names in ServoTrack's port (deferred, KNOWN_DEBT) are not refused.
TEST(M27NamespaceRunner, the_bt_runner_guard_adds_servo_track_and_end_effector_names_and_only_under_a_namespace)
{
  const trainit::eef::Contract c = testContract();
  Names all = baseNames();
  for (const auto& n : servoNames()) all.push_back(n);
  for (const auto& n : eefNames()) all.push_back(n);
  ASSERT_EQ(all.size(), 51u);
  std::vector<std::string> rules, expected;
  for (const auto& [kind, rel] : all)
  {
    rules.push_back(rel + ":=/r2/" + rel);
    expected.push_back(kind + " /r1/" + rel + " -> /r2/" + rel);
  }
  EXPECT_EQ(guardOfBt("/r1", rules, c), expected) << "(a) every name, in the guard's order";
  EXPECT_EQ(guardOfBt("/", rules, c), std::vector<std::string>{}) << "(b) the root is not checked";
  EXPECT_EQ(guardOfBt("/r1", rules, trainit::eef::Contract{}),
            std::vector<std::string>(expected.begin(), expected.begin() + 37))
    << "(c) a LEGACY contract: the 32 and the servo names, no end-effector name";
  EXPECT_EQ(guardOf("/r1", rules), std::vector<std::string>(expected.begin(), expected.begin() + 32))
    << "(d) trainit_run_task's list: the 32 only";
  EXPECT_EQ(guardOfBt("/r1", {"servo_node/delta_twist_cmds:=/r1/m27_servo_alt/delta_twist_cmds",
                              "jaw_controller/gripper_cmd/_action/send_goal:=/r1/m27_jaw_alt/_action/send_goal",
                              "eef/tool/actuators/vacuum/set_engaged:=/r1/m27_vacuum_alt"}, c),
            std::vector<std::string>{}) << "(e) a remap that stays inside /r1 is not one";
  EXPECT_EQ(guardOfBt("/r1", {"jaw_controller/gripper_cmd:=/r2/jaw_controller/gripper_cmd",
                              "slide_controller/follow_joint_trajectory:=/r2/slide_controller/follow_joint_trajectory"},
                      c),
            std::vector<std::string>{}) << "(f) an action NAME remap, which Humble does not apply";
  EXPECT_EQ(guardOfBt("/r1", {"servo_n1/delta_twist_cmds:=/r2/servo_n1/delta_twist_cmds"}, c), std::vector<std::string>{})
    << "(g) a servo node a tree names is not checked (deferred)";
  EXPECT_EQ(endpointText(trainit::eefEndpoints(c)),
            (std::vector<std::string>{"TOPIC eef/jaw/state", "TOPIC eef/jaw/engaged", "ACTION jaw_controller/gripper_cmd",
                                      "TOPIC eef/tool/state", "SERVICE eef/tool/actuators/vacuum/set_engaged",
                                      "ACTION slide_controller/follow_joint_trajectory"}))
    << "(h) eefEndpoints(): per device its state, its TMR scene signal, then its actuators";
  EXPECT_EQ(endpointText(trainit::eefEndpoints(trainit::eef::Contract{})), std::vector<std::string>{}) << "(h) LEGACY";
}

// --- G0 (D-GUARD), T-G2: the end-effector census, in-process ------------------------------------------------
// The REAL registry built for the test contract on a bare node in /r1 (no rosout, no parameter services):
// the names its devices create under /r1, read back from the graph, are exactly the test contract's 14,
// and trainit_run_bt's guard names every one of them when each is remapped onto /r2. This binds
// EndEffectorDevice's constructor to eefEndpoints() and to the guard: a name one creates and the other
// does not list fails here.
TEST(M27NamespaceRunner, the_end_effector_names_the_guard_checks_are_the_names_the_registry_creates)
{
  const trainit::eef::Contract c = testContract();
  auto node = std::make_shared<rclcpp::Node>(
    "m27_eef_census", "/r1",
    rclcpp::NodeOptions().use_global_arguments(false).enable_rosout(false)
                         .start_parameter_services(false).start_parameter_event_publisher(false));
  trainit::EndEffectorRegistry reg(node, c);
  const std::set<std::string> expected = inR1(eefNames());
  std::set<std::string> census;
  m26::waitUntil([&] {
    census = namesOf(node, "m27_eef_census", "/r1", "/r1/");
    return census == expected;
  }, milliseconds(5000));
  EXPECT_EQ(census, expected) << "the registry's names under /r1, from the graph";
  EXPECT_EQ(guardedOf(census, c), census) << "trainit_run_bt's guard names every one";
}

// --- G0 (D-GUARD), T-G3: a cross-namespace remap of a ServoTrack name fails loudly, at once -----------------
// A LEGACY child in /r1 whose tree is one ServoTrack on the default servo node, a fake servo node in /r1
// AND /r2: its twist stream (CR-10) or its start_servo (CR-11) remapped onto /r2 is refused at start-up,
// before any client exists. Without the guard the run would start /r2's servo or stream twists to it (the
// M2.7 forensic sweep's B-SERVO: not refused).
TEST(M27NamespaceRunner, a_cross_namespace_remap_of_a_servo_track_name_fails_loudly)
{
  const std::vector<std::tuple<std::string, std::string, std::string>> cases{
    {"CR-10", "topic", "servo_node/delta_twist_cmds"},   // the twist publisher
    {"CR-11", "service", "servo_node/start_servo"}};     // a Trigger client
  for (const auto& [id, kind, rel] : cases)
  {
    m25::Log servo_log_r1, servo_log_r2;
    m25::FakeServo servo_r1(servo_log_r1, "/r1", "servo_node");
    m25::FakeServo servo_r2(servo_log_r2, "/r2", "servo_node");
    const auto r = runChildren("NSG " + id, {{"/r1", {"-r", rel + ":=/r2/" + rel}, kServoTask, kServoTrack}});
    EXPECT_EQ(r[0].exit, "exit code 2") << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].finished, "") << id;
    EXPECT_EQ(r[0].count(fatalOf(kind, rel)), 1) << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].count("still waiting"), 0) << id << ": refused before the wait";
    EXPECT_EQ(r[0].count("Loading robot model"), 0) << id << ": refused before any MoveIt client exists";
    EXPECT_TRUE(r[0].mgi.empty()) << id << ": no MoveGroupInterface endpoint ever appeared in the graph";
    for (const auto& ns : kSpaces)
    {
      EXPECT_EQ(summaries(*world().logs[ns]), "") << id << ", fake move_group in " << ns;
      EXPECT_EQ(world().logs[ns]->topics(), "") << id << ", fake topics in " << ns;
    }
    EXPECT_EQ(servo_log_r1.events(), "") << id << ": a Trigger call reached the servo node in /r1";
    EXPECT_EQ(servo_log_r2.events(), "") << id << ": a Trigger call reached the servo node in /r2";
    EXPECT_EQ(servo_r1.twists(), "twists: none\n") << id << ": twists reached the servo node in /r1";
    EXPECT_EQ(servo_r2.twists(), "twists: none\n") << id << ": twists reached the servo node in /r2";
  }
}

// --- G0 (D-GUARD), T-G4: a cross-namespace remap of an end-effector name fails loudly, at once --------------
// A canonical child in /r1 (the test contract); the jaw's GripperCommand controller, the tool's SetBool
// service and a latched probe on the jaw's scene signal in /r1 AND /r2. An action's send_goal (CR-12), a
// SetBool service (CR-13), the TMR scene signal (CR-14), an action's status (CR-15) or a device's state
// (CR-16) remapped onto /r2 is refused once the contract is read (its line comes first), before any client
// exists. Without the guard the goal or the SetBool request reaches /r2's device before any check (M2.6
// P6: "the command reached robot_b"), and the scene signal reaches /r2's scene manager.
TEST(M27NamespaceRunner, a_cross_namespace_remap_of_an_end_effector_name_fails_loudly)
{
  const std::vector<std::tuple<std::string, std::string, std::string, std::string>> cases{
    {"CR-12", "service", "jaw_controller/gripper_cmd/_action/send_goal", "<CloseGripper eef=\"jaw\"/>"},
    {"CR-13", "service", "eef/tool/actuators/vacuum/set_engaged", "<CloseGripper eef=\"tool\"/>"},
    {"CR-14", "topic", "eef/jaw/engaged", "<CloseGripper eef=\"jaw\"/>"},
    {"CR-15", "topic", "slide_controller/follow_joint_trajectory/_action/status", "<CloseGripper eef=\"jaw\"/>"},
    {"CR-16", "topic", "eef/tool/state", "<CloseGripper eef=\"tool\"/>"}};
  for (const auto& [id, kind, rel, tree] : cases)
  {
    std::map<std::string, std::unique_ptr<m25::Log>> logs;
    std::map<std::string, std::unique_ptr<m26::ScriptedGripper>> jaw;
    std::map<std::string, std::unique_ptr<m26::after::ScriptedSetBool>> vacuum;
    std::map<std::string, std::unique_ptr<m26::BoolRecorder>> engaged;
    for (const std::string ns : {"/r1", "/r2"})
    {
      logs[ns] = std::make_unique<m25::Log>();
      jaw[ns] = std::make_unique<m26::ScriptedGripper>(*logs[ns], ns, "jaw_controller/gripper_cmd");
      vacuum[ns] = std::make_unique<m26::after::ScriptedSetBool>(*logs[ns], ns, "eef/tool/actuators/vacuum/set_engaged",
                                                                 nullptr);
      engaged[ns] = std::make_unique<m26::BoolRecorder>("m27_engaged" + m26::tagOf(ns), ns + "/eef/jaw/engaged",
                                                        m26::BoolRecorder::Qos::LATCHED);
    }
    const auto r = runChildren("NSG " + id, {{"/r1", {"-r", rel + ":=/r2/" + rel}, kTestEefYaml, tree}});
    EXPECT_EQ(r[0].exit, "exit code 2") << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].finished, "") << id;
    EXPECT_EQ(r[0].count(fatalOf(kind, rel)), 1) << id << "\n" << r[0].dump();
    EXPECT_EQ(r[0].count(kTestEefLine), 1) << id << ": the contract is read first";
    EXPECT_EQ(r[0].count("still waiting"), 0) << id << ": refused before the wait";
    EXPECT_EQ(r[0].count("Loading robot model"), 0) << id << ": refused before any MoveIt client exists";
    EXPECT_TRUE(r[0].mgi.empty()) << id << ": no MoveGroupInterface endpoint ever appeared in the graph";
    for (const std::string ns : {"/r1", "/r2"})
    {
      EXPECT_EQ(jaw[ns]->goals().size(), 0u) << id << ": a GripperCommand goal reached " << ns;
      EXPECT_EQ(vacuum[ns]->calls().size(), 0u) << id << ": a SetBool request reached " << ns;
      EXPECT_EQ(engaged[ns]->count(), 0u) << id << ": a scene signal reached " << ns;
    }
    for (const auto& ns : kSpaces) EXPECT_EQ(summaries(*world().logs[ns]), "") << id << ", fake move_group in " << ns;
  }
}

// --- G0 (D-GUARD), T-G5: the control -- a servo or end-effector remap that stays inside /r1 is not one -------
// A canonical child in /r1 (the test contract), one planned move, three remaps that stay inside /r1 (the
// ServoTrack twists, an action's send_goal, a device's state): no FATAL, and the run succeeds against /r1's
// move_group only. The canonical start-up needs no adapter (checkStateWriters refuses two writers, not none).
TEST(M27NamespaceRunner, a_remap_of_a_servo_or_end_effector_name_that_stays_inside_the_namespace_is_not_refused)
{
  const auto ok = runChildren(
    "NSG eef control",
    {{"/r1",
      {"-r", "servo_node/delta_twist_cmds:=/r1/m27_servo_alt/delta_twist_cmds",
       "-r", "jaw_controller/gripper_cmd/_action/send_goal:=/r1/m27_jaw_alt/_action/send_goal",
       "-r", "eef/tool/state:=/r1/m27_tool_state_alt"},
      kTestEefYaml}});
  EXPECT_EQ(ok[0].exit, "exit code 0") << ok[0].dump();
  EXPECT_EQ(ok[0].finished, "SUCCESS");
  EXPECT_EQ(ok[0].count("MoveIt start-up"), 0) << ok[0].dump();
  EXPECT_EQ(ok[0].count(kTestEefLine), 1) << ok[0].dump();
  EXPECT_EQ(ok[0].mgi, expectedMgi("/r1")) << "control: the child's MoveGroupInterface endpoints";
  for (const auto& other : kSpaces)
    EXPECT_EQ(summaries(*world().logs[other]), other == "/r1" ? expectedEvents("/r1") : std::string())
      << "control, fake in " << other;
}

// --- G0 (D-GUARD), T-G6: the ServoTrack census, a child -----------------------------------------------------
// A LEGACY child in /r1 runs one ServoTrack on the default servo node against a fake servo node in /r1: it
// starts, unpauses, streams, times out and pauses (ForceSuccess; the Sleep keeps the late pause_servo client
// in the graph for the poll). The names its node creates under /r1/servo_node/ are exactly the five the
// guard checks, and trainit_run_bt's guard names every one of them when each is remapped onto /r2. This
// binds ServoTrack's servo_node default and its five suffixes (bt_nodes.cpp) to the guard's copy.
TEST(M27NamespaceRunner, servo_track_resolves_the_default_names_the_guard_checks)
{
  m25::Log servo_log;
  m25::FakeServo servo(servo_log, "/r1", "servo_node");
  std::set<std::string> census;
  const auto r = runChildren(
    "NSG servo census",
    {{"/r1", {}, kServoTask, "<Sequence><ForceSuccess>" + kServoTrack + "</ForceSuccess><Sleep msec=\"1500\"/></Sequence>"}},
    [&] { for (const auto& n : namesOf(world().probe, "trainit_run_bt", "/r1", "/r1/servo_node/")) census.insert(n); });
  EXPECT_EQ(r[0].exit, "exit code 0") << r[0].dump();
  EXPECT_EQ(r[0].finished, "SUCCESS");
  EXPECT_EQ(census, inR1(servoNames())) << "the child's names under /r1/servo_node/, from the graph";
  EXPECT_EQ(guardedOf(census, trainit::eef::Contract{}), census) << "trainit_run_bt's guard names every one";
  EXPECT_EQ(summaries(servo_log), "[1] SERVICE /r1/servo_node/start_servo Trigger.call\n"
                                  "[2] SERVICE /r1/servo_node/unpause_servo Trigger.call\n"
                                  "[3] SERVICE /r1/servo_node/pause_servo Trigger.call\n");
  EXPECT_NE(servo.twists(), "twists: none\n") << "the stream reached /r1's servo node";
}

int main(int argc, char** argv)
{
  return m26::runAll(argc, argv, TRAINIT_M26_DOMAIN);
}
