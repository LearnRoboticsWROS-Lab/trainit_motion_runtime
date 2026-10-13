// =============================================================================
// end_effector_contract.cpp — the `eef` parameter block parser (ADR-0022 D6).
//
// A line-by-line port of the reference normaliser (normalize_eef.py, pinned in
// test/data/m2_6_contract/). The ORDER of the checks is the reference's order, because the
// refusal KEY must be the same for any input, not only for inputs with a single defect; the
// comments name the reference's predicates. Python's truthiness and isinstance() are mapped
// onto rclcpp::ParameterType: a YAML int is PARAMETER_INTEGER, a YAML float PARAMETER_DOUBLE,
// a bool PARAMETER_BOOL (never a number), a sequence a typed array (rcl refuses empty and
// mixed sequences, so neither reaches this parser from a params file).
//
// There is no deliberate difference: for every input this parser gives the reference's outcome,
// the same refusal key and text or the same dump (test_m2_6_eef_contract, on the mutation table
// the reference itself answers). test/data/m2_6_contract/README.md records the reference's
// history: its first copy also accepted non-ASCII letters in a name token; both now use the
// ASCII rule [A-Za-z][A-Za-z0-9_]*.
// =============================================================================
#include "trainit_motion_runtime/end_effector_contract.hpp"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <set>
#include <tuple>
#include <utility>

#include <rclcpp/node.hpp>

namespace trainit
{
namespace eef
{

namespace
{
using PV = rclcpp::ParameterValue;
using PT = rclcpp::ParameterType;

// --- the reference's predicates -------------------------------------------------------------
bool isStr(const PV& v) { return v.get_type() == PT::PARAMETER_STRING; }
bool isBool(const PV& v) { return v.get_type() == PT::PARAMETER_BOOL; }
bool isInt(const PV& v) { return v.get_type() == PT::PARAMETER_INTEGER; }

// _num(v): int or float, never bool
bool isNum(const PV& v) { return isInt(v) || v.get_type() == PT::PARAMETER_DOUBLE; }
double num(const PV& v) { return isInt(v) ? static_cast<double>(v.get<int64_t>()) : v.get<double>(); }

// _nums(v): a non-empty list of numbers
bool isNums(const PV& v)
{
  if (v.get_type() == PT::PARAMETER_INTEGER_ARRAY) return !v.get<std::vector<int64_t>>().empty();
  if (v.get_type() == PT::PARAMETER_DOUBLE_ARRAY) return !v.get<std::vector<double>>().empty();
  return false;
}
std::vector<double> nums(const PV& v)
{
  if (v.get_type() == PT::PARAMETER_INTEGER_ARRAY)
  {
    std::vector<double> out;
    for (const int64_t x : v.get<std::vector<int64_t>>()) out.push_back(static_cast<double>(x));
    return out;
  }
  return v.get<std::vector<double>>();
}

// _strs(v): a non-empty list of strings
bool isStrs(const PV& v)
{
  return v.get_type() == PT::PARAMETER_STRING_ARRAY && !v.get<std::vector<std::string>>().empty();
}

// isinstance(x, int) and not isinstance(x, bool) and x > 0
bool isPositiveInt(const PV& v) { return isInt(v) && v.get<int64_t>() > 0; }

// Python truthiness, for `params.get('gripper_action', '')`
bool truthy(const PV& v)
{
  switch (v.get_type())
  {
    case PT::PARAMETER_NOT_SET: return false;
    case PT::PARAMETER_BOOL: return v.get<bool>();
    case PT::PARAMETER_INTEGER: return v.get<int64_t>() != 0;
    case PT::PARAMETER_DOUBLE: return v.get<double>() != 0.0;
    case PT::PARAMETER_STRING: return !v.get<std::string>().empty();
    case PT::PARAMETER_BYTE_ARRAY: return !v.get<std::vector<uint8_t>>().empty();
    case PT::PARAMETER_BOOL_ARRAY: return !v.get<std::vector<bool>>().empty();
    case PT::PARAMETER_INTEGER_ARRAY: return !v.get<std::vector<int64_t>>().empty();
    case PT::PARAMETER_DOUBLE_ARRAY: return !v.get<std::vector<double>>().empty();
    case PT::PARAMETER_STRING_ARRAY: return !v.get<std::vector<std::string>>().empty();
  }
  return false;
}

bool isAsciiAlpha(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
bool isAsciiAlnum(char c) { return isAsciiAlpha(c) || (c >= '0' && c <= '9'); }

// _is_token(s): a ROS name token, [A-Za-z][A-Za-z0-9_]*
bool isToken(const std::string& s)
{
  if (s.empty() || !isAsciiAlpha(s[0])) return false;
  return std::all_of(s.begin(), s.end(), [](char c) { return isAsciiAlnum(c) || c == '_'; });
}

// _relative_action(s): tokens separated by '/', no leading '/' or '~'
bool isRelativeName(const std::string& s)
{
  if (s.empty() || s[0] == '/' || s[0] == '~') return false;
  std::size_t start = 0;
  for (;;)
  {
    const std::size_t end = s.find('/', start);
    if (!isToken(s.substr(start, end == std::string::npos ? std::string::npos : end - start))) return false;
    if (end == std::string::npos) return true;
    start = end + 1;
  }
}

// Python's repr() of a str, for the refusal text "bad device id 'x'".
std::string pyRepr(const std::string& s)
{
  const bool has_single = s.find('\'') != std::string::npos;
  const bool has_double = s.find('"') != std::string::npos;
  const char quote = (has_single && !has_double) ? '"' : '\'';
  std::string out(1, quote);
  for (const char c : s)
  {
    const auto u = static_cast<unsigned char>(c);
    if (c == '\\') out += "\\\\";
    else if (c == quote) out += std::string("\\") + quote;
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if (u < 0x20 || u == 0x7f)
    {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\x%02x", u);
      out += buf;
    }
    else out += c;
  }
  return out + quote;
}

// --- the dump's formatting (fmt) ----------------------------------------------------------------
std::string fmtDouble(double v)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}
std::string fmtDoubles(const std::vector<double>& v)
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + fmtDouble(v[i]);
  return s;
}
std::string fmtStrs(const std::vector<std::string>& v)
{
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + v[i];
  return s;
}
std::string fmtBool(bool b) { return b ? "true" : "false"; }

// The reference's tuples, as Python prints them inside an f-string.
constexpr const char* kInterfacesRepr = "('set_bool', 'gripper_command', 'follow_joint_trajectory')";
constexpr const char* kResultPoliciesRepr = "('succeeded', 'succeeded_or_stalled')";
constexpr const char* kSceneSignalsRepr = "('adapter', 'tmr', 'none')";
constexpr const char* kAcceptedEvidenceRepr = "('sensor', 'sim_ground_truth', 'actuation_inference')";

std::optional<CommandInterface> interfaceFromToken(const std::string& t)
{
  if (t == "set_bool") return CommandInterface::SET_BOOL;
  if (t == "gripper_command") return CommandInterface::GRIPPER_COMMAND;
  if (t == "follow_joint_trajectory") return CommandInterface::FOLLOW_JOINT_TRAJECTORY;
  return std::nullopt;
}

std::optional<SceneSignal> sceneSignalFromToken(const std::string& t)
{
  if (t == "adapter") return SceneSignal::ADAPTER;
  if (t == "tmr") return SceneSignal::TMR;
  if (t == "none") return SceneSignal::NONE;
  return std::nullopt;
}

std::optional<ResultPolicy> resultPolicyFromToken(const std::string& t)
{
  if (t == "succeeded") return ResultPolicy::SUCCEEDED;
  if (t == "succeeded_or_stalled") return ResultPolicy::SUCCEEDED_OR_STALLED;
  return std::nullopt;
}

std::string lastSegment(const std::string& k)
{
  const auto dot = k.rfind('.');
  return dot == std::string::npos ? k : k.substr(dot + 1);
}
}  // namespace

// --- tokens -------------------------------------------------------------------------------------
const char* toToken(CommandInterface i)
{
  switch (i)
  {
    case CommandInterface::SET_BOOL: return "set_bool";
    case CommandInterface::GRIPPER_COMMAND: return "gripper_command";
    case CommandInterface::FOLLOW_JOINT_TRAJECTORY: return "follow_joint_trajectory";
    case CommandInterface::UNKNOWN: return "unknown";
  }
  return "unknown";
}

const char* toToken(ResultPolicy p)
{
  return p == ResultPolicy::SUCCEEDED ? "succeeded" : "succeeded_or_stalled";
}

const char* toToken(SceneSignal s)
{
  switch (s)
  {
    case SceneSignal::ADAPTER: return "adapter";
    case SceneSignal::TMR: return "tmr";
    case SceneSignal::NONE: return "none";
  }
  return "none";
}

const char* toToken(EvidenceClass e)
{
  switch (e)
  {
    case EvidenceClass::NONE: return "none";
    case EvidenceClass::COMMAND_ECHO: return "command_echo";
    case EvidenceClass::ACTUATION_INFERENCE: return "actuation_inference";
    case EvidenceClass::SENSOR: return "sensor";
    case EvidenceClass::SIM_GROUND_TRUTH: return "sim_ground_truth";
    case EvidenceClass::SIM_ATTACHMENT: return "sim_attachment";
  }
  return "none";
}

std::optional<EvidenceClass> acceptedEvidenceFromToken(const std::string& token)
{
  if (token == "sensor") return EvidenceClass::SENSOR;
  if (token == "sim_ground_truth") return EvidenceClass::SIM_GROUND_TRUTH;
  if (token == "actuation_inference") return EvidenceClass::ACTUATION_INFERENCE;
  return std::nullopt;
}

Refused::Refused(std::string k, std::string w)
: std::runtime_error(k + ": " + w), key(std::move(k)), why(std::move(w))
{
}

// --- the typed contract -------------------------------------------------------------------------
bool ActuatorSpec::hasTarget(const std::string& name) const
{
  if (command_interface == CommandInterface::SET_BOOL) return name == "engage" || name == "release";
  return targets.count(name) != 0;
}

std::vector<std::string> ActuatorSpec::targetNames() const
{
  if (command_interface == CommandInterface::SET_BOOL) return {"engage", "release"};
  std::vector<std::string> out;
  for (const auto& kv : targets) out.push_back(kv.first);
  return out;
}

bool VerificationSpec::accepts(EvidenceClass e) const
{
  return std::find(accepted_evidence.begin(), accepted_evidence.end(), e) != accepted_evidence.end();
}

const ActuatorSpec* DeviceSpec::actuator(const std::string& a) const
{
  for (const auto& x : actuators)
    if (x.id == a) return &x;
  return nullptr;
}

const SiteSpec* DeviceSpec::site(const std::string& s) const
{
  for (const auto& x : sites)
    if (x.id == s) return &x;
  return nullptr;
}

const SiteSpec* DeviceSpec::siteGrippedBy(const std::string& a) const
{
  for (const auto& x : sites)
    if (x.grip == a) return &x;
  return nullptr;
}

const DeviceSpec* Contract::device(const std::string& id) const
{
  for (const auto& d : devices)
    if (d.id == id) return &d;
  return nullptr;
}

std::vector<std::string> Contract::ids() const
{
  std::vector<std::string> out;
  for (const auto& d : devices) out.push_back(d.id);
  return out;
}

std::string Contract::dump() const
{
  std::string s;
  for (const auto& [k, v] : normalized) s += "eef." + k + " = " + v + "\n";
  return s;
}

// --- the parser: normalize_eef.py parse(), in its order ----------------------------------------
Contract parseContract(const ParameterMap& params)
{
  // eef = {k[4:]: v for k, v in params.items() if k.startswith('eef.')}
  std::map<std::string, PV> eef;
  for (const auto& [k, v] : params)
    if (k.rfind("eef.", 0) == 0) eef.emplace(k.substr(4), v);
  Contract c;
  if (eef.empty()) return c;   // LEGACY

  const auto ids_it = eef.find("ids");
  if (ids_it == eef.end() || !isStrs(ids_it->second))
    throw Refused("eef.ids", "a non-empty string array is required");
  const auto ids = ids_it->second.get<std::vector<std::string>>();
  // legacy and canonical keys mixed
  const auto ga = params.find("gripper_action");
  if (ga != params.end() && truthy(ga->second))
    throw Refused("gripper_action", "a legacy key next to an eef block");
  for (const char* k : {"gripper_cmd_topic", "gripper_close_position", "gripper_open_position"})
    if (params.count(k) != 0) throw Refused(k, "a legacy key next to an eef block");
  if (std::set<std::string>(ids.begin(), ids.end()).size() != ids.size())
    throw Refused("eef.ids", "duplicate device id");

  auto& out = c.normalized;
  out["ids"] = fmtStrs(ids);
  std::set<std::string> known{"ids"};

  for (const auto& dev : ids)
  {
    if (!isToken(dev) || dev == "eef") throw Refused("eef.ids", "bad device id " + pyRepr(dev));
    const std::string p = dev + ".";
    const std::string e = "eef." + p;   // the refusal-key prefix, f'eef.{dev}.'

    // get(key, default=None, required=False): marks a present key known
    auto get = [&](const std::string& key, bool required) -> const PV* {
      const std::string full = p + key;
      const auto it = eef.find(full);
      if (it != eef.end())
      {
        known.insert(full);
        return &it->second;
      }
      if (required) throw Refused("eef." + full, "required");
      return nullptr;
    };

    DeviceSpec d;
    d.id = dev;
    const PV* act_v = get("actuator_ids", true);
    const PV* site_v = get("site_ids", true);
    auto tokenList = [](const PV& v) {
      if (!isStrs(v)) return false;
      const auto l = v.get<std::vector<std::string>>();
      return std::set<std::string>(l.begin(), l.end()).size() == l.size() &&
             std::all_of(l.begin(), l.end(), isToken);
    };
    if (!tokenList(*act_v)) throw Refused(e + "actuator_ids", "non-empty, unique ROS name tokens required");
    if (!tokenList(*site_v)) throw Refused(e + "site_ids", "non-empty, unique ROS name tokens required");
    d.actuator_ids = act_v->get<std::vector<std::string>>();
    d.site_ids = site_v->get<std::vector<std::string>>();
    for (const auto& a : d.actuator_ids)
      if (std::find(d.site_ids.begin(), d.site_ids.end(), a) != d.site_ids.end())
        throw Refused("eef." + dev, "one token used as both an actuator and a site id");

    if (const PV* v = get("command_timeout_ms", false))
    {
      if (!isPositiveInt(*v)) throw Refused(e + "command_timeout_ms", "a positive integer");
      d.command_timeout_ms = v->get<int64_t>();
    }
    const PV* scene_v = get("scene_signal", true);
    const auto scene = isStr(*scene_v) ? sceneSignalFromToken(scene_v->get<std::string>()) : std::nullopt;
    if (!scene) throw Refused(e + "scene_signal", std::string("one of ") + kSceneSignalsRepr);
    if (*scene != SceneSignal::NONE && d.site_ids.size() != 1)
      throw Refused(e + "scene_signal", "only a single-site device has a scene signal");
    d.scene_signal = *scene;
    out[p + "actuator_ids"] = fmtStrs(d.actuator_ids);
    out[p + "site_ids"] = fmtStrs(d.site_ids);
    out[p + "command_timeout_ms"] = std::to_string(d.command_timeout_ms);
    out[p + "scene_signal"] = toToken(d.scene_signal);

    std::map<std::string, std::string> actions;       // action -> actuator
    std::map<std::string, std::string> joints_seen;   // joint -> actuator
    std::map<std::string, CommandInterface> interfaces;
    std::map<std::string, std::set<std::string>> targets_of;

    for (const auto& a : d.actuator_ids)
    {
      const std::string q = "actuators." + a + ".";
      const std::string eq = e + q;
      ActuatorSpec s;
      s.id = a;
      const PV* iface_v = get(q + "command_interface", true);
      const auto iface = isStr(*iface_v) ? interfaceFromToken(iface_v->get<std::string>()) : std::nullopt;
      if (!iface) throw Refused(eq + "command_interface", std::string("one of ") + kInterfacesRepr);
      s.command_interface = *iface;
      interfaces[a] = *iface;
      out[p + q + "command_interface"] = toToken(*iface);

      const PV* joints_v = get(q + "joints", false);
      if (joints_v)
      {
        bool ok = isStrs(*joints_v);
        if (ok)
        {
          const auto l = joints_v->get<std::vector<std::string>>();
          ok = std::set<std::string>(l.begin(), l.end()).size() == l.size();
        }
        if (!ok) throw Refused(eq + "joints", "a non-empty array of unique names");
        s.joints = joints_v->get<std::vector<std::string>>();
        for (const auto& j : s.joints)
        {
          const auto seen = joints_seen.find(j);
          if (seen != joints_seen.end()) throw Refused(eq + "joints", "joint " + j + " shared with " + seen->second);
          joints_seen[j] = a;
        }
        out[p + q + "joints"] = fmtStrs(s.joints);
      }

      if (*iface == CommandInterface::SET_BOOL)
      {
        const PV* jp_e = get(q + "joint_positions.engaged", false);
        const PV* jp_r = get(q + "joint_positions.released", false);
        if ((jp_e == nullptr) != (jp_r == nullptr) || (joints_v == nullptr) != (jp_e == nullptr))
          throw Refused(eq + "joint_positions", "joints and both joint_positions, or none");
        if (joints_v)
        {
          if (!isNums(*jp_e) || !isNums(*jp_r) || nums(*jp_e).size() != s.joints.size() ||
              nums(*jp_r).size() != s.joints.size())
            throw Refused(eq + "joint_positions", "one number per joint");
          s.engaged_positions = nums(*jp_e);
          s.released_positions = nums(*jp_r);
          out[p + q + "joint_positions.engaged"] = fmtDoubles(s.engaged_positions);
          out[p + q + "joint_positions.released"] = fmtDoubles(s.released_positions);
        }
        if (const PV* fb = get(q + "actuation_feedback", false))
        {
          if (!isBool(*fb)) throw Refused(eq + "actuation_feedback", "a bool");
          s.actuation_feedback = fb->get<bool>();
        }
        if (const PV* ct = get(q + "confirm_timeout_ms", false))
        {
          if (!isPositiveInt(*ct)) throw Refused(eq + "confirm_timeout_ms", "a positive integer");
          s.confirm_timeout_ms = ct->get<int64_t>();
        }
        out[p + q + "actuation_feedback"] = fmtBool(s.actuation_feedback);
        out[p + q + "confirm_timeout_ms"] = std::to_string(s.confirm_timeout_ms);
        targets_of[a] = {"engage", "release"};
        d.actuators.push_back(std::move(s));
        continue;
      }

      const PV* action_v = get(q + "action", true);
      if (!isStr(*action_v) || !isRelativeName(action_v->get<std::string>()))
        throw Refused(eq + "action", "a relative ROS name");
      s.action = action_v->get<std::string>();
      const auto shared = actions.find(s.action);
      if (shared != actions.end()) throw Refused(eq + "action", "shared with " + shared->second);
      actions[s.action] = a;
      out[p + q + "action"] = s.action;
      if (!joints_v) throw Refused(eq + "joints", "required");
      // tnames = sorted({k.split('.')[-1] for k in eef if k.startswith(p + q + 'targets.')})
      std::set<std::string> tnames;
      const std::string tprefix = p + q + "targets.";
      for (const auto& kv : eef)
        if (kv.first.rfind(tprefix, 0) == 0) tnames.insert(lastSegment(kv.first));
      if (tnames.empty()) throw Refused(eq + "targets", "required");
      targets_of[a] = tnames;

      if (*iface == CommandInterface::GRIPPER_COMMAND)
      {
        if (s.joints.size() != 1) throw Refused(eq + "joints", "exactly one driven joint");
        if (tnames.count("open") == 0 || tnames.count("closed") == 0)
          throw Refused(eq + "targets", "open and closed are required");
        for (const auto& t : tnames)
        {
          const PV* v = get(q + "targets." + t, false);
          if (!v || !isNum(*v)) throw Refused(eq + "targets." + t, "a number");
          s.targets[t] = {num(*v)};
          out[p + q + "targets." + t] = fmtDouble(num(*v));
        }
        // an adapter tells ENGAGE / RELEASE / TARGET apart by the goal's position:
        // len(set(values)) != len(values), Python's float equality (0.0 == -0.0, NaN != NaN)
        for (auto i = tnames.begin(); i != tnames.end(); ++i)
          for (auto j = std::next(i); j != tnames.end(); ++j)
            if (s.targets.at(*i).front() == s.targets.at(*j).front())
              throw Refused(eq + "targets", "distinct values (a command is told by its position)");
        const PV* rp_v = get(q + "result_policy", true);
        const auto rp = isStr(*rp_v) ? resultPolicyFromToken(rp_v->get<std::string>()) : std::nullopt;
        if (!rp) throw Refused(eq + "result_policy", std::string("one of ") + kResultPoliciesRepr);
        s.result_policy = *rp;
        out[p + q + "result_policy"] = toToken(*rp);
        if (const PV* me = get(q + "max_effort", false))
        {
          if (!isNum(*me) || num(*me) <= 0) throw Refused(eq + "max_effort", "a number > 0");
          s.max_effort = num(*me);
          out[p + q + "max_effort"] = fmtDouble(num(*me));
        }
        d.actuators.push_back(std::move(s));
        continue;
      }

      // follow_joint_trajectory
      const std::size_t n = s.joints.size();
      for (const auto& t : tnames)
      {
        const PV* v = get(q + "targets." + t, false);
        if (!v || !isNums(*v) || nums(*v).size() != n)
          throw Refused(eq + "targets." + t, std::to_string(n) + " numbers");
        s.targets[t] = nums(*v);
        out[p + q + "targets." + t] = fmtDoubles(nums(*v));
      }
      // a scalar tolerance is one value per joint; then n numbers > 0
      auto perJoint = [n](const PV& v) -> std::optional<std::vector<double>> {
        std::vector<double> l;
        if (isNum(v)) l.assign(n, num(v));
        else if (isNums(v)) l = nums(v);
        else return std::nullopt;
        if (l.size() != n || std::any_of(l.begin(), l.end(), [](double x) { return x <= 0; })) return std::nullopt;
        return l;
      };
      const PV* gt_v = get(q + "goal_tolerance", true);
      const auto gt = perJoint(*gt_v);
      if (!gt) throw Refused(eq + "goal_tolerance", std::to_string(n) + " numbers > 0 (or one scalar)");
      s.goal_tolerance = *gt;
      out[p + q + "goal_tolerance"] = fmtDoubles(*gt);
      const PV* d_v = get(q + "duration_s", true);
      if (!isNum(*d_v) || num(*d_v) <= 0) throw Refused(eq + "duration_s", "a number > 0");
      s.duration_s = num(*d_v);
      out[p + q + "duration_s"] = fmtDouble(s.duration_s);
      if (const PV* gtt = get(q + "goal_time_tolerance_s", false))
      {
        if (!isNum(*gtt) || num(*gtt) <= 0) throw Refused(eq + "goal_time_tolerance_s", "a number > 0");
        s.goal_time_tolerance_s = num(*gtt);
      }
      out[p + q + "goal_time_tolerance_s"] = fmtDouble(s.goal_time_tolerance_s);
      if (const PV* pt_v = get(q + "path_tolerance", false))
      {
        const auto pt = perJoint(*pt_v);
        if (!pt) throw Refused(eq + "path_tolerance", std::to_string(n) + " numbers > 0 (or one scalar)");
        s.path_tolerance = *pt;
        out[p + q + "path_tolerance"] = fmtDoubles(*pt);
      }
      d.actuators.push_back(std::move(s));
    }

    std::map<std::string, std::string> grips;   // actuator -> site
    for (const auto& sid : d.site_ids)
    {
      const std::string q = "sites." + sid + ".";
      const std::string eq = e + q;
      const PV* grip_v = get(q + "grip", true);
      if (!isStr(*grip_v) || interfaces.count(grip_v->get<std::string>()) == 0)
        throw Refused(eq + "grip", "an actuator of this device");
      const std::string grip = grip_v->get<std::string>();
      if (interfaces.at(grip) == CommandInterface::FOLLOW_JOINT_TRAJECTORY)
        throw Refused(eq + "grip", "a follow_joint_trajectory actuator never grips");
      const auto taken = grips.find(grip);
      if (taken != grips.end()) throw Refused(eq + "grip", "already grips " + taken->second);
      grips[grip] = sid;
      const PV* eng_v = get(q + "engage", true);
      const PV* rel_v = get(q + "release", true);
      for (const auto& [name, v] : {std::make_pair("engage", eng_v), std::make_pair("release", rel_v)})
        if (!isStr(*v) || targets_of.at(grip).count(v->get<std::string>()) == 0)
          throw Refused(eq + name, "a target of " + grip);
      // eng == rel (both are strings here)
      if (eng_v->get<std::string>() == rel_v->get<std::string>())
        throw Refused(eq + "release", "differs from engage");
      // a set_bool grip engages with SetBool data:true; its polarity is the adapter's, never the block's
      if (interfaces.at(grip) == CommandInterface::SET_BOOL && eng_v->get<std::string>() != "engage")
        throw Refused(eq + "engage", "engage for a set_bool grip (its polarity is the adapter's)");
      SiteSpec site;
      site.id = sid;
      site.grip = grip;
      site.engage = eng_v->get<std::string>();
      site.release = rel_v->get<std::string>();
      out[p + q + "grip"] = site.grip;
      out[p + q + "engage"] = site.engage;
      out[p + q + "release"] = site.release;
      d.sites.push_back(std::move(site));
    }
    for (const auto& a : d.actuator_ids)   // interfaces.items(), in actuator order
      if (interfaces.at(a) == CommandInterface::GRIPPER_COMMAND && grips.count(a) == 0)
        throw Refused(e + "actuators." + a, "a gripper_command actuator must grip a site");

    const std::string v = "verification.";
    for (const auto& [key, field] :
         {std::make_pair("require_grasp_confirmation", &d.verification.require_grasp_confirmation),
          std::make_pair("require_release_confirmation", &d.verification.require_release_confirmation)})
    {
      const PV* b = get(v + key, true);
      if (!isBool(*b)) throw Refused(e + v + key, "a bool");
      *field = b->get<bool>();
      out[p + v + key] = fmtBool(*field);
    }
    std::vector<std::string> ae{"sensor", "sim_ground_truth"};
    if (const PV* ae_v = get(v + "accepted_evidence", false))
    {
      bool ok = isStrs(*ae_v);
      if (ok)
      {
        ae = ae_v->get<std::vector<std::string>>();
        ok = std::all_of(ae.begin(), ae.end(), [](const std::string& x) { return acceptedEvidenceFromToken(x).has_value(); }) &&
             std::set<std::string>(ae.begin(), ae.end()).size() == ae.size();
      }
      if (!ok) throw Refused(e + v + "accepted_evidence", std::string("unique values of ") + kAcceptedEvidenceRepr);
    }
    for (const auto& x : ae) d.verification.accepted_evidence.push_back(*acceptedEvidenceFromToken(x));
    out[p + v + "accepted_evidence"] = fmtStrs(ae);
    for (const auto& [key, def, field] :
         {std::make_tuple("timeout_ms", int64_t{3000}, &d.verification.timeout_ms),
          std::make_tuple("max_age_ms", int64_t{500}, &d.verification.max_age_ms),
          std::make_tuple("clock_tolerance_ms", int64_t{100}, &d.verification.clock_tolerance_ms)})
    {
      int64_t x = def;
      if (const PV* xv = get(v + key, false))
      {
        if (!isPositiveInt(*xv)) throw Refused(e + v + key, "a positive integer");
        x = xv->get<int64_t>();
      }
      *field = x;
      out[p + v + key] = std::to_string(x);
    }
    c.devices.push_back(std::move(d));
  }

  // unknown = sorted(set(eef) - known)
  for (const auto& kv : eef)
    if (known.count(kv.first) == 0) throw Refused("eef." + kv.first, "unknown key");
  return c;
}

Contract parseContract(rclcpp::Node& node)
{
  const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  return parseContract(ParameterMap(overrides.begin(), overrides.end()));
}

}  // namespace eef
}  // namespace trainit
