// vNext M2.6 AFTER (ADR-0022), M1: TMR's parser of the `eef` parameter block IS the reference
// normaliser. Pure: no ROS graph (rcl's YAML parser only).
//
//   - the byte copies in test/data/m2_6_contract/ are the pinned ones (sha256);
//   - the conformance fixture's dump equals eef_conformance.expected.txt byte for byte;
//   - every row of eef_refusal_mutations.txt (a mutation of the fixture, and what normalize_eef.py
//     answers for it -- the same table test_m2_6_eef_reference.py re-computes from the reference)
//     gives the SAME answer: the same refusal key and text, the same dump, or LEGACY;
//   - TMR's own token tables equal the pinned eef_contract_tokens.txt (the token parity);
//   - the fixture is representable as typed data (P1): 6 actuators on 3 interfaces, 2 sites.
#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <trainit_interfaces/msg/end_effector_actuator_state.hpp>
#include <trainit_interfaces/msg/end_effector_state.hpp>

#include "m2_6/eef_after_contract.hpp"
#include "trainit_motion_runtime/actuator_outcome.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"
#include "trainit_motion_runtime/end_effector_reason.hpp"
#include "trainit_motion_runtime/verify_grasp.hpp"

namespace
{
namespace eef = trainit::eef;
namespace after = m26::after;
using State = trainit_interfaces::msg::EndEffectorState;
using Act = trainit_interfaces::msg::EndEffectorActuatorState;

// group -> name -> value, from the pinned token file
std::map<std::string, std::map<std::string, std::string>> tokens()
{
  std::istringstream in(after::readFile(after::contractFile("eef_contract_tokens.txt")));
  std::map<std::string, std::map<std::string, std::string>> t;
  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    std::string g, n, v;
    ls >> g >> n >> v;
    t[g][n] = v;
  }
  return t;
}
}  // namespace

TEST(M26EefContract, the_byte_copies_are_the_pinned_ones)
{
  for (const auto& [file, sha] : after::pinned())
    EXPECT_EQ(m25::sha256(after::readFile(after::contractFile(file))), sha)
      << file << " is not the pinned byte copy (EEF_CONTRACT_SPEC.md): a copy is never edited locally";
}

TEST(M26EefContract, the_fixture_dump_equals_the_expected_file)
{
  const auto c = eef::parseContract(after::loadParams(after::contractFile("eef_conformance.yaml")));
  ASSERT_TRUE(c.canonical());
  EXPECT_EQ(c.dump(), after::readFile(after::contractFile("eef_conformance.expected.txt")));
  EXPECT_EQ(c.ids(), (std::vector<std::string>{"composite_tool", "suction_simple", "jaw_simple", "jaw_binary"}));
}

TEST(M26EefContract, every_mutation_row_gives_the_reference_answer)
{
  const auto rows = after::loadMutationTable();
  ASSERT_GE(rows.size(), 100u) << "the mutation table lost rows";
  std::set<std::string> refused_keys;
  int refused = 0, accepted = 0, legacy = 0;
  for (const auto& row : rows)
  {
    auto params = after::loadParams(after::contractFile("eef_conformance.yaml"));
    for (const auto& m : row.mutations) after::applyMutation(params, m);
    const std::string got = after::outcome(params);
    EXPECT_EQ(got, row.expected) << "row " << row.id;
    if (row.expected.rfind("REFUSED ", 0) == 0)
    {
      ++refused;
      refused_keys.insert(row.expected.substr(8, row.expected.find(": ") - 8));
    }
    else if (row.expected == "LEGACY") ++legacy;
    else ++accepted;
  }
  // every class of answer is exercised, and the refusals name many distinct keys
  EXPECT_GT(refused, 80);
  EXPECT_GT(accepted, 10);
  EXPECT_EQ(legacy, 2);
  EXPECT_GT(refused_keys.size(), 50u);
}

TEST(M26EefContract, legacy_is_the_absence_of_the_block)
{
  EXPECT_EQ(after::outcome({}), "LEGACY");
  eef::ParameterMap legacy{{"gripper_action", rclcpp::ParameterValue(std::string("/suctioncup_controller/gripper_command"))},
                           {"gripper_cmd_topic", rclcpp::ParameterValue(std::string("/gripper_cmd"))},
                           {"gripper_close_position", rclcpp::ParameterValue(0.7691)}};
  const auto c = eef::parseContract(legacy);
  EXPECT_FALSE(c.canonical());
  EXPECT_EQ(c.dump(), "");
  // ... and ANY eef.* key makes the project claim the contract: without eef.ids it is refused
  legacy["eef.main.scene_signal"] = rclcpp::ParameterValue(std::string("adapter"));
  EXPECT_EQ(after::outcome(legacy), "REFUSED eef.ids: a non-empty string array is required");
}

// P1: the R1/R2-like device is representable, and so are the three short-form devices.
TEST(M26EefContract, the_fixture_is_representable)
{
  const auto c = after::fixtureContract();
  const eef::DeviceSpec* d = c.device("composite_tool");
  ASSERT_NE(d, nullptr);
  ASSERT_EQ(d->actuators.size(), 6u);
  std::vector<eef::CommandInterface> ifaces;
  for (const auto& a : d->actuators) ifaces.push_back(a.command_interface);
  using CI = eef::CommandInterface;
  EXPECT_EQ(ifaces, (std::vector<CI>{CI::FOLLOW_JOINT_TRAJECTORY, CI::FOLLOW_JOINT_TRAJECTORY,
                                     CI::FOLLOW_JOINT_TRAJECTORY, CI::GRIPPER_COMMAND, CI::SET_BOOL, CI::SET_BOOL}));
  EXPECT_EQ(std::set<CI>(ifaces.begin(), ifaces.end()).size(), 3u);
  EXPECT_EQ(d->actuator("wrist_mechanism")->joints,
            (std::vector<std::string>{"robot_a_wrist_pitch_joint", "robot_a_wrist_roll_joint"}));
  EXPECT_EQ(d->actuator("wrist_mechanism")->targets.at("present"), (std::vector<double>{0.3, -0.3}));
  EXPECT_EQ(d->actuator("rotary_axis")->path_tolerance, (std::vector<double>{0.2}));
  EXPECT_TRUE(d->actuator("linear_axis")->path_tolerance.empty());
  EXPECT_EQ(d->actuator("gripper_a")->max_effort.value_or(0.0), 20.0);
  EXPECT_EQ(d->actuator("gripper_a")->result_policy, eef::ResultPolicy::SUCCEEDED_OR_STALLED);
  EXPECT_TRUE(d->actuator("binary_lock")->actuation_feedback);
  EXPECT_TRUE(d->actuator("binary_lock")->joints.empty());
  EXPECT_EQ(d->actuator("gripper_b")->engaged_positions, (std::vector<double>{0.0}));
  ASSERT_EQ(d->sites.size(), 2u);
  EXPECT_EQ(d->site("site_a")->grip, "gripper_a");
  EXPECT_EQ(d->site("site_a")->engage, "closed");
  EXPECT_EQ(d->site("site_b")->grip, "gripper_b");
  EXPECT_EQ(d->siteGrippedBy("linear_axis"), nullptr);   // an axis grips nothing
  EXPECT_EQ(d->scene_signal, eef::SceneSignal::NONE);   // multi-site: no scene signal
  EXPECT_EQ(d->verification.accepted_evidence, (std::vector<eef::EvidenceClass>{eef::EvidenceClass::SENSOR}));

  // the simple suction stays simple: one set_bool actuator, one site, the adapter's scene signal
  const eef::DeviceSpec* s = c.device("suction_simple");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->actuator_ids, (std::vector<std::string>{"vacuum"}));
  EXPECT_EQ(s->sites.size(), 1u);
  EXPECT_EQ(s->scene_signal, eef::SceneSignal::ADAPTER);
  EXPECT_EQ(s->verification.accepted_evidence,
            (std::vector<eef::EvidenceClass>{eef::EvidenceClass::SENSOR, eef::EvidenceClass::SIM_GROUND_TRUTH}));
  EXPECT_EQ(c.device("jaw_simple")->scene_signal, eef::SceneSignal::TMR);
  EXPECT_EQ(c.device("jaw_simple")->actuator("gripper")->targets.at("closed"), (std::vector<double>{0.7}));
  EXPECT_EQ(c.device("jaw_binary")->actuator("gripper")->command_interface, CI::SET_BOOL);
}

// The same ids on another robot: only the joints (global TF names) change.
TEST(M26EefContract, a_second_robot_has_identical_ids)
{
  const auto a = after::fixtureContract();
  const auto b = after::fixtureContract("robot_b_");
  EXPECT_EQ(a.device("composite_tool")->actuator_ids, b.device("composite_tool")->actuator_ids);
  EXPECT_EQ(a.device("composite_tool")->site_ids, b.device("composite_tool")->site_ids);
  EXPECT_EQ(b.device("composite_tool")->actuator("linear_axis")->joints,
            (std::vector<std::string>{"robot_b_linear_axis_joint"}));
}

// P10, the refusal half: an axis without its tolerance or its duration is refused, so the runner
// exits with code 2 (test_m2_6_eef_runner_child) -- with the JTC's default 0 tolerances a blocked
// axis would end SUCCEEDED.
TEST(M26EefContract, an_axis_without_tolerance_or_duration_is_refused)
{
  for (const char* key : {"goal_tolerance", "duration_s"})
  {
    auto params = after::loadParams(after::contractFile("eef_conformance.yaml"));
    params.erase(std::string("eef.composite_tool.actuators.wrist_mechanism.") + key);
    EXPECT_EQ(after::outcome(params),
              std::string("REFUSED eef.composite_tool.actuators.wrist_mechanism.") + key + ": required");
  }
}

// P4 / EEF-D8: command_echo and sim_attachment are never accepted evidence.
TEST(M26EefContract, an_echo_or_a_weld_is_never_accepted_evidence)
{
  for (const char* never : {"command_echo", "sim_attachment"})
  {
    auto params = after::loadParams(after::contractFile("eef_conformance.yaml"));
    params["eef.suction_simple.verification.accepted_evidence"] =
      rclcpp::ParameterValue(std::vector<std::string>{"sensor", never});
    EXPECT_EQ(after::outcome(params),
              "REFUSED eef.suction_simple.verification.accepted_evidence: unique values of "
              "('sensor', 'sim_ground_truth', 'actuation_inference')") << never;
    EXPECT_FALSE(eef::acceptedEvidenceFromToken(never).has_value());
  }
}

// --- the token parity -----------------------------------------------------------------------------
TEST(M26EefContract, the_token_tables_equal_the_pinned_token_file)
{
  const auto t = tokens();
  auto expectGroup = [&](const std::string& group, const std::map<std::string, std::string>& mine) {
    ASSERT_TRUE(t.count(group)) << "the token file has no group " << group;
    EXPECT_EQ(t.at(group), mine) << "group " << group;
  };
  auto byValue = [](const char* (*name)(uint8_t), uint8_t n) {
    std::map<std::string, std::string> m;
    for (uint8_t v = 0; v < n; ++v) m[name(v)] = std::to_string(v);
    return m;
  };
  std::map<std::string, std::string> ci;
  for (const auto i : {eef::CommandInterface::UNKNOWN, eef::CommandInterface::SET_BOOL,
                       eef::CommandInterface::GRIPPER_COMMAND, eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY})
    ci[eef::toToken(i)] = std::to_string(static_cast<int>(i));
  expectGroup("command_interface", ci);
  // the parser's interface values are the message's
  EXPECT_EQ(static_cast<int>(eef::CommandInterface::SET_BOOL), Act::INTERFACE_SET_BOOL);
  EXPECT_EQ(static_cast<int>(eef::CommandInterface::GRIPPER_COMMAND), Act::INTERFACE_GRIPPER_COMMAND);
  EXPECT_EQ(static_cast<int>(eef::CommandInterface::FOLLOW_JOINT_TRAJECTORY), Act::INTERFACE_FOLLOW_JOINT_TRAJECTORY);
  expectGroup("tool_status", byValue(eef::toolStatusToken, 5));
  expectGroup("actuator_status", byValue(eef::actuatorStatusToken, 5));
  expectGroup("binary", byValue(eef::binaryToken, 3));
  expectGroup("command", byValue(eef::commandToken, 4));
  expectGroup("grasp", byValue(eef::graspToken, 6));
  expectGroup("evidence", byValue(eef::evidenceToken, 6));
  std::map<std::string, std::string> ev;
  for (uint8_t v = 0; v < 6; ++v)
    ev[eef::toToken(static_cast<eef::EvidenceClass>(v))] = std::to_string(v);
  expectGroup("evidence", ev);
  std::map<std::string, std::string> accepted;
  for (const auto& [name, value] : t.at("evidence"))
    if (eef::acceptedEvidenceFromToken(name)) accepted[name] = "1";
  expectGroup("accepted_evidence", accepted);
  expectGroup("result_policy", {{eef::toToken(eef::ResultPolicy::SUCCEEDED), "1"},
                                {eef::toToken(eef::ResultPolicy::SUCCEEDED_OR_STALLED), "1"}});
  expectGroup("scene_signal", {{eef::toToken(eef::SceneSignal::ADAPTER), "1"}, {eef::toToken(eef::SceneSignal::TMR), "1"},
                               {eef::toToken(eef::SceneSignal::NONE), "1"}});
  std::map<std::string, std::string> reasons;
  for (const auto r : eef::kAllReasons) reasons[eef::toToken(r)] = eef::toToken(eef::classOf(r));
  expectGroup("reason", reasons);
  // VERDICT is the only class a generated retry may absorb
  for (const auto r : eef::kAllReasons)
    EXPECT_EQ(eef::isFault(r), eef::classOf(r) != eef::ReasonClass::VERDICT) << eef::toToken(r);
  // capabilities are configuration TMR never interprets (ADR-0022 D6): the group exists for TSA
  // and the adapter rows, and TMR keeps no table of it
  EXPECT_EQ(t.at("capability").size(), 10u);
  const std::set<std::string> known{"command_interface", "tool_status", "actuator_status", "binary", "command",
                                    "grasp", "evidence", "accepted_evidence", "result_policy", "scene_signal",
                                    "capability", "reason"};
  for (const auto& [group, names] : t) EXPECT_TRUE(known.count(group)) << "an unchecked token group: " << group;
}
