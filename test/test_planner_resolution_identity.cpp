// Resolution identity (vNext M2.1): what every planner mode resolves to, for every
// intent and pipeline availability, is FROZEN in test/data/planner_resolution_m2_1.txt.
//
// The table was dumped from the TMR 1.7.0 binary BEFORE the planner registry existed,
// and this test was run against that code first: the registry must change who KNOWS the
// vocabulary, never what a move resolves to. A deliberate change of resolution (a new
// planner, a different degrade) edits the table in the same commit, line by line,
// with the reason in the commit message.
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "trainit_motion_runtime/manual_motion_strategy_selector.hpp"

using namespace trainit;

namespace
{

std::string dumpResolution()
{
  const std::vector<std::pair<std::string, std::vector<std::string>>> availability = {
    {"default", {"ompl", "pilz_industrial_motion_planner", "chomp"}},
    {"all_plus_stomp", {"ompl", "pilz_industrial_motion_planner", "chomp", "stomp"}},
    {"no_chomp", {"ompl", "pilz_industrial_motion_planner"}},
    {"empty_list", {}},
  };
  const std::vector<std::pair<std::string, PlannerMode>> modes = {
    {"pilz", PlannerMode::PILZ}, {"ompl", PlannerMode::OMPL},
    {"ompl_chomp", PlannerMode::OMPL_CHOMP}, {"stomp", PlannerMode::STOMP}};
  const std::vector<std::pair<std::string, MotionIntent>> intents = {
    {"PTP", MotionIntent::PTP}, {"LINEAR", MotionIntent::LINEAR},
    {"CIRCULAR", MotionIntent::CIRCULAR}, {"COLLISION_FREE", MotionIntent::COLLISION_FREE},
    {"COLLISION_FREE_OPTIMIZED", MotionIntent::COLLISION_FREE_OPTIMIZED},
    {"CARTESIAN_PROCESS_PATH", MotionIntent::CARTESIAN_PROCESS_PATH},
    {"ONLINE_SERVO", MotionIntent::ONLINE_SERVO}};

  std::ostringstream out;
  for (const auto& [aname, available] : availability)
    for (const auto& [mname, mode] : modes)
    {
      ManualMotionStrategySelector s(mode, ProcessPathBackend::PILZ_SEQUENCE, available);
      for (const auto& [iname, intent] : intents)
      {
        const auto p = s.selectPlanner(intent, "x");
        out << aname << ' ' << mname << ' ' << iname << " -> eff=" << toString(s.mode())
            << " pipeline=" << p.pipeline_id << " planner=" << p.planner_id
            << " two_stage=" << p.two_stage_optimize << " optimizer=" << p.optimizer_pipeline_id
            << '\n';
      }
      // the path moveCollisionFree takes: the mode is set per move
      ManualMotionStrategySelector t(PlannerMode::PILZ, ProcessPathBackend::PILZ_SEQUENCE, available);
      t.setMode(mode);
      out << aname << ' ' << mname << " setMode -> eff=" << toString(t.mode()) << '\n';
    }
  // the last two are candidate providers (ADR-0019): never a planner mode
  for (const char* token : {"pilz", "ompl", "ompl_chomp", "stomp", "", "bogus", "PILZ", "chomp", "Pilz",
                            "cumotion", "curobo"})
  {
    const auto m = plannerModeFromString(token);
    out << "parse '" << token << "' -> " << (m ? toString(*m) : std::string("<none>")) << '\n';
  }
  return out.str();
}

std::vector<std::string> lines(const std::string& text)
{
  std::vector<std::string> v;
  std::istringstream in(text);
  for (std::string l; std::getline(in, l);) v.push_back(l);
  return v;
}

}  // namespace

TEST(PlannerResolutionIdentity, EveryModeIntentAndAvailabilityResolvesAsBeforeTheRegistry)
{
  std::ifstream f(std::string(TRAINIT_TEST_DATA_DIR) + "/planner_resolution_m2_1.txt");
  ASSERT_TRUE(f.good()) << "missing " << TRAINIT_TEST_DATA_DIR << "/planner_resolution_m2_1.txt";
  std::stringstream frozen;
  frozen << f.rdbuf();

  const auto want = lines(frozen.str());
  const auto got = lines(dumpResolution());
  ASSERT_EQ(want.size(), got.size()) << "the resolution table changed shape";
  for (size_t i = 0; i < want.size(); ++i)
    EXPECT_EQ(want[i], got[i]) << "resolution row " << i + 1 << " changed";
}
