// vNext M2.6 AFTER test harness -- the canonical end-effector contract's shared data (ADR-0022).
//
//   pinned()           the sha256 of every BYTE COPY in test/data/m2_6_contract/ (the fixture, its
//                      expected dump, the token file, the reference normaliser): a local edit of a
//                      copy is caught before any comparison rests on it
//   loadParams(path)   a ROS 2 params file as the flat {dotted name: value} map a node sees, read by
//                      rcl's own YAML parser (rclcpp::parameter_map_from_yaml_file)
//   yamlValue(text)    ONE YAML flow value typed exactly as rcl types it in a params file
//   mutation table     eef_refusal_mutations.txt: rows, and how a row is applied to the map
//   outcome(params)    "REFUSED <key>: <why>" | "ACCEPT <sha256 of the dump>" | "LEGACY", the
//                      reference normaliser's own vocabulary
//   contractFromYaml   a contract from an inline params-file text (the AFTER tests' variants)
//
// Reused from M2.5 (test/m2_5/capture.hpp): sha256.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_CONTRACT_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_CONTRACT_HPP_

#include <unistd.h>

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/parameter_map.hpp>
#include <rclcpp/parameter_value.hpp>

#include "m2_5/capture.hpp"
#include "trainit_motion_runtime/end_effector_contract.hpp"

namespace m26
{
namespace after
{

inline std::string contractDir() { return TRAINIT_M26_CONTRACT_DIR; }
inline std::string contractFile(const std::string& name) { return contractDir() + "/" + name; }

inline std::string readFile(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) throw std::runtime_error("cannot read " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// The byte copies and their pins (EEF_CONTRACT_SPEC.md, ADR-0022 D8 "Conformance").
inline std::map<std::string, std::string> pinned()
{
  return {
    {"eef_conformance.yaml", "873a1d582c4e3947a62026d153bd49ccb02d7b4d38e5cb3a5b4b3a8f5a27a04f"},
    {"eef_conformance.expected.txt", "1f6bd1d2ac2fb18865ee67a815877483537d0a325fbc9a5400cf2977710d242c"},
    {"eef_contract_tokens.txt", "55048ea44d466979f57c28a634295a060a79b837fe614a30c49f60572a2bd5de"},
    {"normalize_eef.py", "9fba00c3981762974d9e916464fd670c8f83964392649c3e0854a94149290a0b"}};
}

// A params file, flattened by rcl exactly as a node's overrides are: the `/**` (or first) node's
// ros__parameters.
inline trainit::eef::ParameterMap loadParams(const std::string& path)
{
  const auto all = rclcpp::parameter_map_from_yaml_file(path);
  const auto it = all.count("/**") ? all.find("/**") : all.begin();
  if (it == all.end()) throw std::runtime_error(path + ": no ros__parameters");
  trainit::eef::ParameterMap out;
  for (const auto& p : it->second) out[p.get_name()] = p.get_parameter_value();
  return out;
}

inline std::string scratchFile(const std::string& stem)
{
  return std::string(TRAINIT_M26_SCRATCH_DIR) + "/" + stem + "_" + std::to_string(::getpid()) + ".yaml";
}

inline void writeText(const std::string& path, const std::string& text)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
}

// ONE YAML flow value, typed by rcl: "5" integer, "5.0" double, "\"x\"" string, "[1, 2]" integer
// array, "true" bool.
inline rclcpp::ParameterValue yamlValue(const std::string& text)
{
  const std::string path = scratchFile("m26_value");
  writeText(path, "/**:\n  ros__parameters:\n    v: " + text + "\n");
  const auto m = loadParams(path);
  ::unlink(path.c_str());
  return m.at("v");
}

// A contract from an inline params-file text ("/**:\n  ros__parameters:\n    eef: ...").
inline trainit::eef::Contract contractFromYaml(const std::string& yaml, const std::string& stem = "m26_contract")
{
  const std::string path = scratchFile(stem);
  writeText(path, yaml);
  const auto params = loadParams(path);
  ::unlink(path.c_str());
  return trainit::eef::parseContract(params);
}

// --- the mutation table ---------------------------------------------------------------------------
struct MutationRow
{
  std::string id;
  std::vector<std::string> mutations;   // "erase <key>" | "erase_prefix <p>" | "set <key> = <yaml>"
  std::string expected;                 // "REFUSED <key>: <why>" | "ACCEPT <sha256>" | "LEGACY"
};

inline std::string trim(const std::string& s)
{
  const auto a = s.find_first_not_of(' ');
  const auto b = s.find_last_not_of(' ');
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

inline std::vector<std::string> splitOn(const std::string& s, const std::string& sep)
{
  std::vector<std::string> out;
  std::size_t start = 0;
  for (;;)
  {
    const auto at = s.find(sep, start);
    out.push_back(s.substr(start, at == std::string::npos ? std::string::npos : at - start));
    if (at == std::string::npos) return out;
    start = at + sep.size();
  }
}

inline std::vector<MutationRow> loadMutationTable()
{
  std::istringstream in(readFile(contractFile("eef_refusal_mutations.txt")));
  std::vector<MutationRow> rows;
  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#') continue;
    const auto f = splitOn(line, " | ");
    if (f.size() != 3) throw std::runtime_error("malformed mutation row: " + line);
    MutationRow r;
    r.id = trim(f[0]);
    if (trim(f[1]) != "(none)")
      for (const auto& m : splitOn(f[1], " ; ")) r.mutations.push_back(trim(m));
    r.expected = trim(f[2]);
    rows.push_back(r);
  }
  return rows;
}

inline void applyMutation(trainit::eef::ParameterMap& params, const std::string& m)
{
  if (m.rfind("erase_prefix ", 0) == 0)
  {
    const std::string prefix = m.substr(13);
    for (auto it = params.begin(); it != params.end();)
      it = it->first.rfind(prefix, 0) == 0 ? params.erase(it) : std::next(it);
  }
  else if (m.rfind("erase ", 0) == 0)
  {
    if (params.erase(m.substr(6)) != 1) throw std::runtime_error(m + ": no such key in the fixture");
  }
  else if (m.rfind("set ", 0) == 0)
  {
    const auto eq = m.find(" = ");
    if (eq == std::string::npos) throw std::runtime_error("bad mutation " + m);
    params[m.substr(4, eq - 4)] = yamlValue(m.substr(eq + 3));
  }
  else
  {
    throw std::runtime_error("bad mutation " + m);
  }
}

// What trainit::eef::parseContract answers, in the reference normaliser's vocabulary.
inline std::string outcome(const trainit::eef::ParameterMap& params)
{
  try
  {
    const auto c = trainit::eef::parseContract(params);
    if (!c.canonical()) return "LEGACY";
    return "ACCEPT " + m25::sha256(c.dump());
  }
  catch (const trainit::eef::Refused& r)
  {
    return "REFUSED " + std::string(r.what());
  }
}

// The fixture's contract, and its composite device re-targeted onto another robot's joints
// (robot_a_* -> <prefix>*): IDENTICAL eef, actuator and site ids (ADR-0022 "R1/R2-like fixture").
inline trainit::eef::Contract fixtureContract(const std::string& joint_prefix = "robot_a_")
{
  auto params = loadParams(contractFile("eef_conformance.yaml"));
  if (joint_prefix != "robot_a_")
    for (auto& [k, v] : params)
      if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        auto l = v.get<std::vector<std::string>>();
        for (auto& s : l)
          if (s.rfind("robot_a_", 0) == 0) s = joint_prefix + s.substr(8);
        v = rclcpp::ParameterValue(l);
      }
  return trainit::eef::parseContract(params);
}

// Only the named devices of a contract, in its eef.ids order: what one robot's runner loads (the
// normalised dump is not carried over; the registry reads the typed devices only).
inline trainit::eef::Contract onlyDevices(const trainit::eef::Contract& c, const std::vector<std::string>& ids)
{
  trainit::eef::Contract out;
  for (const auto& d : c.devices)
    for (const auto& id : ids)
      if (d.id == id) out.devices.push_back(d);
  return out;
}

}  // namespace after
}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_CONTRACT_HPP_
