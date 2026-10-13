// test_planner_registry (vNext M2.1, ADR-0021): TMR's C++ planner mapping checked against
// TMR's planner registry, config/planner_profiles.yaml, entry by entry.
//
// registryProblems() is a pure check of ONE registry file against the code compiled here.
// The real registry must have no problem; and every committed MUTATION of it below must fail
// with the message that names what is missing -- so adding or removing a planner can never be
// half-done (the owner's M2.1 acceptance). Mutations go to temp files; no environment variable
// is read anywhere. The registry is read through a CONST node: a yaml-cpp lookup of an absent
// key through a non-const node inserts it, which would mutate the map being iterated.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"
#include "trainit_motion_runtime/moveit_motion_runtime.hpp"
#include "trainit_motion_runtime/motion_types.hpp"

using namespace trainit;
namespace fs = std::filesystem;

namespace
{

using Problems = std::vector<std::string>;

std::string join(const std::vector<std::string>& v)
{
  std::string out;
  for (const auto& s : v) out += (out.empty() ? "" : ", ") + s;
  return "[" + out + "]";
}

std::string str(const YAML::Node& n, const std::string& def = "")
{
  return (n && n.IsScalar()) ? n.as<std::string>() : def;
}

std::string profileText(const PlannerProfile& p)
{
  return p.pipeline_id + "/" + p.planner_id +
         (p.two_stage_optimize ? " +optimizer " + p.optimizer_pipeline_id : std::string());
}

// How MoveWaypoint treats an execution token: true = it plans the move (it gets past its
// guard and reaches the -- absent -- runtime, which throws), false = it refuses it.
bool moveWaypointPlans(const std::string& execution)
{
  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);
  factory.registerBehaviorTreeFromText(
    R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">)"
    R"(<MoveWaypoint waypoint="wp"/></BehaviorTree></root>)");
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("wp.type", "tcp");
  bb->set<std::string>("wp.position", "0.5;0.0;0.3");
  bb->set<std::string>("wp.orientation", "0;1;0;0");
  bb->set<std::string>("wp.motion", "free");
  bb->set<std::string>("wp.execution", execution);
  auto tree = factory.createTree("T", bb);
  try
  {
    tree.tickOnce();
    return false;   // FAILURE before the runtime: refused
  }
  catch (const std::exception&)
  {
    return true;    // past the guard: it would plan
  }
}

// Every way the C++ compiled here disagrees with the registry at `path`. Empty = it implements it.
Problems registryProblems(const std::string& path, const std::string& pipelines_dir)
{
  Problems out;
  YAML::Node loaded_file;
  try
  {
    loaded_file = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    return {"cannot read the registry " + path + ": " + e.what()};
  }
  const YAML::Node reg = loaded_file;   // every lookup below is const: none can insert a key
  if (str(reg["schema_version"]) != "1")
    return {"schema_version is '" + str(reg["schema_version"]) + "': this TMR maps schema 1"};

  // --- pipelines ------------------------------------------------------------------------
  std::vector<std::string> declared, loaded;
  std::set<std::string> parameter_files;
  for (const auto& kv : reg["pipelines"])
  {
    const std::string id = kv.first.as<std::string>();
    declared.push_back(id);
    const bool is_loaded = kv.second["loaded"] && kv.second["loaded"].as<bool>();
    const std::string file = str(kv.second["parameters"]);
    if (is_loaded) loaded.push_back(id);
    if (is_loaded && file.empty())
      out.push_back("pipeline '" + id + "' is loaded but has no parameters file");
    if (!file.empty())
    {
      parameter_files.insert(file);
      if (!fs::is_regular_file(fs::path(pipelines_dir) / file))
        out.push_back("pipeline '" + id + "': its parameters file " + file +
                      " is not in config/pipelines/");
    }
    if (str(kv.second["plugin_package"]).empty())
      out.push_back("pipeline '" + id + "' names no plugin_package");
  }
  const std::set<std::string> cpp_pipelines{PIPELINE_OMPL, PIPELINE_PILZ, PIPELINE_CHOMP,
                                            PIPELINE_STOMP};
  for (const auto& id : declared)
    if (!cpp_pipelines.count(id))
      out.push_back("registry pipeline '" + id + "' has no PIPELINE_* constant in TMR's C++: "
                    "the selector cannot request it");
  for (const auto& id : cpp_pipelines)
    if (std::find(declared.begin(), declared.end(), id) == declared.end())
      out.push_back("TMR's C++ names pipeline '" + id + "', which the registry does not declare");
  if (defaultAvailablePipelines() != loaded)
    out.push_back("defaultAvailablePipelines() is " + join(defaultAvailablePipelines()) +
                  " but the registry's loaded set is " + join(loaded));
  if (MoveItMotionRuntime::Config{}.available_pipelines != loaded)
    out.push_back("MoveItMotionRuntime::Config's default available_pipelines differ from the "
                  "registry's loaded set " + join(loaded));
  std::set<std::string> shipped;
  for (const auto& f : fs::directory_iterator(pipelines_dir))
    if (f.path().extension() == ".yaml") shipped.insert(f.path().filename().string());
  for (const auto& f : shipped)
    if (!parameter_files.count(f))
      out.push_back("config/pipelines/" + f + " is shipped but no registry pipeline declares it");
  const std::string default_pipeline = str(reg["default_pipeline"]);
  if (std::find(loaded.begin(), loaded.end(), default_pipeline) == loaded.end())
    out.push_back("default_pipeline '" + default_pipeline + "' is not loaded");

  // --- planner modes ----------------------------------------------------------------------
  std::vector<std::string> registry_modes, cpp_modes;
  for (const auto& kv : reg["planner_modes"]) registry_modes.push_back(kv.first.as<std::string>());
  for (const PlannerMode m : kPlannerModes) cpp_modes.push_back(toString(m));
  for (const auto& token : registry_modes)
    if (!plannerModeFromString(token))
      out.push_back("registry mode '" + token + "' is not a PlannerMode: add it to PlannerMode, "
                    "kPlannerModes, toString() and the selector's switch");
  for (const auto& token : cpp_modes)
    if (std::find(registry_modes.begin(), registry_modes.end(), token) == registry_modes.end())
      out.push_back("PlannerMode '" + token + "' is not in the registry: declare it there or "
                    "remove it from the C++");
  if (registry_modes != cpp_modes && std::is_permutation(registry_modes.begin(),
      registry_modes.end(), cpp_modes.begin(), cpp_modes.end()))
    out.push_back("kPlannerModes lists the modes in another order than the registry: " +
                  join(cpp_modes) + " vs " + join(registry_modes));

  auto deliverable = [&](const YAML::Node& spec) {
    auto is_loaded = [&](const std::string& id) {
      return std::find(loaded.begin(), loaded.end(), id) != loaded.end();
    };
    const std::string optimizer = str(spec["optimizer_pipeline"]);
    return is_loaded(str(spec["pipeline"])) && (optimizer.empty() || is_loaded(optimizer));
  };

  for (const auto& kv : reg["planner_modes"])
  {
    const std::string token = kv.first.as<std::string>();
    const auto mode = plannerModeFromString(token);
    if (!mode) continue;   // reported above
    const YAML::Node spec = kv.second;
    const std::string stability = str(spec["stability"]);
    if (stability != "stable" && stability != "experimental")
      out.push_back("mode '" + token + "': stability '" + stability +
                    "' is not stable | experimental");

    // what the row says the selector returns, with every declared pipeline available
    PlannerProfile want{str(spec["pipeline"]), str(spec["planner"]), false, ""};
    if (!str(spec["optimizer_pipeline"]).empty())
    {
      want.two_stage_optimize = true;
      want.optimizer_pipeline_id = str(spec["optimizer_pipeline"]);
    }
    ManualMotionStrategySelector every(*mode, ProcessPathBackend::PILZ_SEQUENCE, declared);
    for (const MotionIntent intent : {MotionIntent::PTP, MotionIntent::COLLISION_FREE})
    {
      const PlannerProfile got = every.selectPlanner(intent, "registry");
      if (every.mode() != *mode || got.pipeline_id != want.pipeline_id ||
          got.planner_id != want.planner_id || got.two_stage_optimize != want.two_stage_optimize ||
          got.optimizer_pipeline_id != want.optimizer_pipeline_id)
        out.push_back("mode '" + token + "' resolves to " + profileText(got) + " in C++; the "
                      "registry says " + profileText(want));
    }

    // what the loaded set does to it
    const std::string degrades_to = str(spec["degrades_to"]);
    ManualMotionStrategySelector deployed(*mode, ProcessPathBackend::PILZ_SEQUENCE, loaded);
    if (deliverable(spec))
    {
      if (deployed.mode() != *mode)
        out.push_back("mode '" + token + "' is deliverable but C++ degrades it to '" +
                      toString(deployed.mode()) + "'");
    }
    else if (degrades_to.empty())
    {
      out.push_back("mode '" + token + "' is not deliverable and declares no degrades_to");
    }
    else
    {
      if (toString(deployed.mode()) != degrades_to)
        out.push_back("mode '" + token + "' degrades to '" + toString(deployed.mode()) +
                      "' in C++; the registry says '" + degrades_to + "'");
      const bool declared_mode = std::find(registry_modes.begin(), registry_modes.end(),
                                           degrades_to) != registry_modes.end();
      if (!declared_mode || !deliverable(reg["planner_modes"][degrades_to]))
        out.push_back("mode '" + token + "' degrades to '" + degrades_to +
                      "', which is not a deliverable mode");
    }
    const std::string optimizer = str(spec["optimizer_pipeline"]);
    if (!optimizer.empty())
    {
      std::vector<std::string> without = loaded;
      without.erase(std::remove(without.begin(), without.end(), optimizer), without.end());
      ManualMotionStrategySelector missing(*mode, ProcessPathBackend::PILZ_SEQUENCE, without);
      if (toString(missing.mode()) != degrades_to)
        out.push_back("with its optimizer pipeline missing, mode '" + token + "' degrades to '" +
                      toString(missing.mode()) + "' in C++; the registry says '" + degrades_to + "'");
    }
  }
  const std::string default_mode = str(reg["default_planner_mode"]);
  if (default_mode != toString(kDefaultPlannerMode))
    out.push_back("default_planner_mode '" + default_mode + "' but TMR's kDefaultPlannerMode is '" +
                  toString(kDefaultPlannerMode) + "'");
  if (PlanningOptions{}.mode != kDefaultPlannerMode ||
      MoveItMotionRuntime::Config{}.mode != kDefaultPlannerMode)
    out.push_back("PlanningOptions / MoveItMotionRuntime::Config do not default to "
                  "kDefaultPlannerMode");

  // --- Cartesian shapes: mode-independent ----------------------------------------------------
  const std::vector<std::pair<std::string, MotionIntent>> shapes = {
    {"linear", MotionIntent::LINEAR}, {"circular", MotionIntent::CIRCULAR},
    {"process_path", MotionIntent::CARTESIAN_PROCESS_PATH}};
  for (const auto& [name, intent] : shapes)
  {
    const YAML::Node spec = reg["cartesian"][name];
    if (!spec)
    {
      out.push_back("the registry declares no cartesian '" + name + "'");
      continue;
    }
    for (const PlannerMode m : kPlannerModes)
    {
      ManualMotionStrategySelector s(m, ProcessPathBackend::PILZ_SEQUENCE, declared);
      const PlannerProfile got = s.selectPlanner(intent, "registry");
      if (got.pipeline_id != str(spec["pipeline"]) || got.planner_id != str(spec["planner"]))
        out.push_back("cartesian '" + name + "' resolves to " + profileText(got) + " in mode '" +
                      toString(m) + "'; the registry says " + str(spec["pipeline"]) + "/" +
                      str(spec["planner"]));
    }
  }
  for (const auto& kv : reg["cartesian"])
  {
    const std::string name = kv.first.as<std::string>();
    if (std::none_of(shapes.begin(), shapes.end(), [&](const auto& s) { return s.first == name; }))
      out.push_back("cartesian '" + name + "' is not a shape TMR maps (linear | circular | "
                    "process_path)");
  }
  // the Pilz sequence backend (the path circ moves and process paths take) uses these constants
  if (str(reg["cartesian"]["linear"]["planner"]) != PILZ_LIN ||
      str(reg["cartesian"]["circular"]["planner"]) != PILZ_CIRC)
    out.push_back("PILZ_LIN / PILZ_CIRC differ from the registry's cartesian planners");

  // --- execution: which BT node runs a move, and whether it is planned ------------------------
  // Exactly the ADR-0015 modes TMR implements: MoveWaypoint plans `plan`, ServoTrack streams
  // `servo`. A learned policy is TMR's RunPolicy node calling the Pro policy runtime (ADR-0005):
  // never an execution entry of this Community registry (ADR-0021 Decision 6).
  const std::vector<std::pair<std::string, std::string>> implemented = {
    {"plan", "MoveWaypoint"}, {"servo", "ServoTrack"}};
  for (const auto& [token, node] : implemented)
  {
    if (!reg["execution"][token])
      out.push_back("the registry declares no execution '" + token + "', which TMR implements "
                    "(" + node + ")");
    else if (str(reg["execution"][token]["node"]) != node)
      out.push_back("execution '" + token + "' is run by " + node + " in TMR; the registry says '" +
                    str(reg["execution"][token]["node"]) + "'");
  }
  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);
  for (const auto& kv : reg["execution"])
  {
    const std::string token = kv.first.as<std::string>();
    const std::string node = str(kv.second["node"]);
    const bool planned = kv.second["planned"] && kv.second["planned"].as<bool>();
    if (std::none_of(implemented.begin(), implemented.end(),
                     [&](const auto& m) { return m.first == token; }))
      out.push_back("execution '" + token + "' is not a mode TMR implements (plan | servo); a "
                    "learned policy is RunPolicy, never a Community registry entry (ADR-0021 D6)");
    if (!factory.builders().count(node))
      out.push_back("execution '" + token + "': node '" + node + "' is not a registered BT node");
    if (moveWaypointPlans(token) != planned)
      out.push_back("execution '" + token + "' is " + (planned ? "planned" : "not planned") +
                    " in the registry, but MoveWaypoint " +
                    (planned ? "refuses it" : "plans it"));
    if (planned && node != "MoveWaypoint")
      out.push_back("execution '" + token + "' is planned, so MoveWaypoint runs it, not '" + node + "'");
  }
  return out;
}

// A registry, mutated, in a private temp file; the file is removed with the object.
class Mutated
{
public:
  explicit Mutated(const std::function<void(YAML::Node&)>& edit)
  {
    YAML::Node reg = YAML::Clone(YAML::LoadFile(TRAINIT_PLANNER_REGISTRY));
    edit(reg);
    std::string tmpl = (fs::temp_directory_path() / "trainit_planner_registry_XXXXXX").string();
    const int fd = mkstemp(tmpl.data());
    if (fd < 0) throw std::runtime_error("mkstemp failed for " + tmpl);
    close(fd);
    path_ = tmpl;
    std::ofstream out(path_, std::ios::trunc);
    out << YAML::Dump(reg) << '\n';
    out.close();
    if (!out) throw std::runtime_error("could not write the mutated registry " + path_);
  }
  Mutated(const Mutated&) = delete;
  Mutated& operator=(const Mutated&) = delete;
  ~Mutated() { std::remove(path_.c_str()); }
  Problems problems() const { return registryProblems(path_, TRAINIT_PIPELINES_DIR); }

private:
  std::string path_;
};

bool mentions(const Problems& problems, const std::string& needle)
{
  return std::any_of(problems.begin(), problems.end(),
                     [&](const std::string& p) { return p.find(needle) != std::string::npos; });
}

std::string all(const Problems& problems)
{
  std::string out;
  for (const auto& p : problems) out += "\n  - " + p;
  return out.empty() ? " (none)" : out;
}

}  // namespace

TEST(PlannerRegistry, TheCodeImplementsTheRegistry)
{
  const Problems p = registryProblems(TRAINIT_PLANNER_REGISTRY, TRAINIT_PIPELINES_DIR);
  EXPECT_TRUE(p.empty()) << "TMR's C++ disagrees with config/planner_profiles.yaml:" << all(p);
}

TEST(PlannerRegistry, ThePlannerModeListIsTheTokenTable)
{
  EXPECT_EQ(plannerModeList(), "pilz | ompl | ompl_chomp | stomp");
  for (const PlannerMode m : kPlannerModes)
    EXPECT_EQ(plannerModeFromString(toString(m)).value(), m);
}

// --- the owner's acceptance: adding or removing an entry fails, and says what is missing ------

TEST(PlannerRegistryMutation, AModeAddedOnlyToTheRegistryIsNotMapped)
{
  const auto p = Mutated([](YAML::Node& r) {
    r["planner_modes"]["rrtstar"]["pipeline"] = "ompl";
    r["planner_modes"]["rrtstar"]["planner"] = "RRTstarkConfigDefault";
    r["planner_modes"]["rrtstar"]["stability"] = "stable";
  }).problems();
  EXPECT_TRUE(mentions(p, "registry mode 'rrtstar' is not a PlannerMode")) << all(p);
}

TEST(PlannerRegistryMutation, AModeRemovedOnlyFromTheRegistryIsStillMapped)
{
  const auto p = Mutated([](YAML::Node& r) { r["planner_modes"].remove("stomp"); }).problems();
  EXPECT_TRUE(mentions(p, "PlannerMode 'stomp' is not in the registry")) << all(p);
}

TEST(PlannerRegistryMutation, APipelineLoadedOnlyInTheRegistryIsNotTheDefaultSet)
{
  const auto p = Mutated([](YAML::Node& r) {
    r["pipelines"]["stomp"]["loaded"] = true;
    r["pipelines"]["stomp"]["parameters"] = "stomp_planning.yaml";
  }).problems();
  EXPECT_TRUE(mentions(p, "defaultAvailablePipelines()")) << all(p);
  EXPECT_TRUE(mentions(p, "stomp_planning.yaml is not in config/pipelines/")) << all(p);
}

TEST(PlannerRegistryMutation, ACandidateProviderCannotSlipInAsAPipeline)
{
  // cuMotion is candidate metadata (ADR-0019), never an entry: if one is written in anyway,
  // the C++ has no constant for it, the default set disagrees and no parameters file exists.
  const auto p = Mutated([](YAML::Node& r) {
    r["pipelines"]["cumotion"]["plugin_package"] = "isaac_ros_cumotion_moveit";
    r["pipelines"]["cumotion"]["parameters"] = "cumotion_planning.yaml";
    r["pipelines"]["cumotion"]["loaded"] = true;
  }).problems();
  EXPECT_TRUE(mentions(p, "registry pipeline 'cumotion' has no PIPELINE_* constant")) << all(p);
  EXPECT_TRUE(mentions(p, "defaultAvailablePipelines()")) << all(p);
}

TEST(PlannerRegistryMutation, AProfileTheSelectorDoesNotReturnIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) {
    r["planner_modes"]["ompl"]["planner"] = "RRTstarkConfigDefault";
  }).problems();
  EXPECT_TRUE(mentions(p, "mode 'ompl' resolves to ompl/RRTConnectkConfigDefault in C++")) << all(p);
}

TEST(PlannerRegistryMutation, ADegradeTheSelectorDoesNotDoIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) {
    r["planner_modes"]["ompl_chomp"]["degrades_to"] = "pilz";
  }).problems();
  EXPECT_TRUE(mentions(p, "mode 'ompl_chomp' degrades to 'ompl' in C++")) << all(p);
}

TEST(PlannerRegistryMutation, ADefaultModeOnlyTheRegistryChangedIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) { r["default_planner_mode"] = "ompl"; }).problems();
  EXPECT_TRUE(mentions(p, "default_planner_mode 'ompl' but TMR's kDefaultPlannerMode is 'pilz'"))
    << all(p);
}

TEST(PlannerRegistryMutation, AnExecutionModeNoNodeRunsIsCaught)
{
  const auto planned = Mutated([](YAML::Node& r) {
    r["execution"]["policy"]["planned"] = true;
    r["execution"]["policy"]["node"] = "MoveWaypoint";
  }).problems();
  EXPECT_TRUE(mentions(planned, "execution 'policy' is planned in the registry, but MoveWaypoint "
                                "refuses it")) << all(planned);
  const auto ghost = Mutated([](YAML::Node& r) {
    r["execution"]["ghost"]["planned"] = false;
    r["execution"]["ghost"]["node"] = "NoSuchNode";
  }).problems();
  EXPECT_TRUE(mentions(ghost, "node 'NoSuchNode' is not a registered BT node")) << all(ghost);
}

TEST(PlannerRegistryMutation, AnotherSchemaIsRefused)
{
  const auto p = Mutated([](YAML::Node& r) { r["schema_version"] = 2; }).problems();
  EXPECT_TRUE(mentions(p, "schema_version is '2': this TMR maps schema 1")) << all(p);
}

TEST(PlannerRegistryMutation, ADegradeToAModeTheRegistryDoesNotHaveIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) { r["planner_modes"]["stomp"]["degrades_to"] = "nope"; })
                   .problems();
  EXPECT_TRUE(mentions(p, "mode 'stomp' degrades to 'nope', which is not a deliverable mode")) << all(p);
}

TEST(PlannerRegistryMutation, AnExecutionModeRemovedFromTheRegistryIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) { r["execution"].remove("servo"); }).problems();
  EXPECT_TRUE(mentions(p, "the registry declares no execution 'servo'")) << all(p);
}

TEST(PlannerRegistryMutation, ALearnedPolicyIsNotACommunityExecutionEntry)
{
  // RunPolicy IS a registered node and MoveWaypoint refuses 'policy', so only the rule that the
  // registry holds exactly the modes TMR implements can catch it (ADR-0021 Decision 6).
  const auto p = Mutated([](YAML::Node& r) {
    r["execution"]["policy"]["planned"] = false;
    r["execution"]["policy"]["node"] = "RunPolicy";
  }).problems();
  EXPECT_TRUE(mentions(p, "execution 'policy' is not a mode TMR implements")) << all(p);
}

TEST(PlannerRegistryMutation, ACartesianShapeTmrDoesNotMapIsCaught)
{
  const auto p = Mutated([](YAML::Node& r) {
    r["cartesian"]["spline"]["pipeline"] = "pilz_industrial_motion_planner";
    r["cartesian"]["spline"]["planner"] = "SPLINE";
  }).problems();
  EXPECT_TRUE(mentions(p, "cartesian 'spline' is not a shape TMR maps")) << all(p);
}

// A PlannerMode added to the enum and to both switches but not to kPlannerModes (and so not to
// the registry) compiles; this is what makes it fail: it prints a token the parser rejects.
TEST(PlannerRegistry, EveryEnumeratorIsInTheTokenTable)
{
  for (int i = 0; i < 16; ++i)
  {
    const auto m = static_cast<PlannerMode>(i);
    const std::string token = toString(m);   // out-of-range values fall back to 'pilz'
    EXPECT_TRUE(plannerModeFromString(token).has_value())
      << "PlannerMode " << i << " prints '" << token << "' but the parser rejects it: add it to "
         "kPlannerModes and to the registry";
  }
}

// MoveCollisionFree / MoveFree write `InputPort<std::string>("planner", "pilz")`. In BT.CPP v4
// the two-argument form takes a DESCRIPTION, so the port has no default: an unset `planner` is
// read as "" and the move takes the runtime's planner mode, as the node's comment and the Groot
// model say -- no second, typed planner default exists (verified by this test, vNext M2.1).
TEST(PlannerRegistry, AnUnsetPlannerPortMeansTheRuntimesMode)
{
  BT::BehaviorTreeFactory factory;
  registerAllNodes(factory);
  for (const char* node : {"MoveCollisionFree", "MoveFree"})
  {
    const auto& ports = factory.manifests().at(node).ports;
    ASSERT_TRUE(ports.count("planner")) << node;
    EXPECT_TRUE(ports.at("planner").defaultValueString().empty())
      << node << ": the planner port now has a default ('"
      << ports.at("planner").defaultValueString()
      << "'): it must be the registry's default_planner_mode, or none";
  }
}
