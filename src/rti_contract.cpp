// rti_contract.cpp -- the Robot Task Interface v1 vocabulary and the exported-task table (ADR-0032). Pure.
#include "trainit_motion_runtime/rti_contract.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace trainit::rti
{

const char* cmdName(uint8_t cmd)
{
  switch (cmd)
  {
    case 0: return "NONE";
    case 1: return "START";
    case 2: return "CANCEL";
    case 3: return "RESET";
    default: return "?";
  }
}

const char* stateName(State state)
{
  switch (state)
  {
    case State::UNAVAILABLE: return "UNAVAILABLE";
    case State::IDLE: return "IDLE";
    case State::RUNNING: return "RUNNING";
    case State::STOPPING: return "STOPPING";
    case State::FAULTED: return "FAULTED";
  }
  return "?";
}

const char* ackName(Ack ack)
{
  switch (ack)
  {
    case Ack::NONE: return "NONE";
    case Ack::ACCEPTED: return "ACCEPTED";
    case Ack::REJECTED: return "REJECTED";
  }
  return "?";
}

const char* resultName(Result result)
{
  switch (result)
  {
    case Result::NONE: return "NONE";
    case Result::SUCCEEDED: return "SUCCEEDED";
    case Result::ENDED: return "ENDED";
    case Result::CANCELLED: return "CANCELLED";
    case Result::FAILED: return "FAILED";
  }
  return "?";
}

const char* codeName(uint16_t c)
{
  switch (c)
  {
    case code::NONE: return "";
    case code::OK: return "OK";
    case code::BIN_EMPTY: return "BIN_EMPTY";
    case code::NOTHING_PICKABLE: return "NOTHING_PICKABLE";
    case code::PICKABLE_EXHAUSTED: return "PICKABLE_EXHAUSTED";
    case code::RECOVERY_EXHAUSTED: return "RECOVERY_EXHAUSTED";
    case code::CANCELLED: return "CANCELLED";
    case code::TERMINAL_FAULT: return "TERMINAL_FAULT";
    case code::TASK_FAILED: return "TASK_FAILED";
    case code::TASK_TIMEOUT: return "TASK_TIMEOUT";
    case code::REQUESTER_LOST: return "REQUESTER_LOST";
    case code::RUNTIME_LOST: return "RUNTIME_LOST";
    case code::STATION_TIMEOUT: return "STATION_TIMEOUT";
    case code::STATION_FAULT: return "STATION_FAULT";
    case code::CANCELLED_WHILE_HOLDING: return "CANCELLED_WHILE_HOLDING";
    case code::EEF_FAULT: return "EEF_FAULT";
    case code::CANCELLED_HOLD_UNKNOWN: return "CANCELLED_HOLD_UNKNOWN";
    case code::STATION_UNAVAILABLE: return "STATION_UNAVAILABLE";
    case code::SERVER_STARTED: return "SERVER_STARTED";
    case code::REQUESTER_CONFLICT: return "REQUESTER_CONFLICT";
    case code::INVALID_REQUEST: return "INVALID_REQUEST";
    case code::UNKNOWN_TASK: return "UNKNOWN_TASK";
    case code::BUSY: return "BUSY";
    case code::NOT_READY: return "NOT_READY";
    case code::FAULTED: return "FAULTED";
    case code::DUPLICATE_JOB: return "DUPLICATE_JOB";
    case code::NOT_CURRENT_JOB: return "NOT_CURRENT_JOB";
    case code::INVALID_STATE: return "INVALID_STATE";
    case code::SEQUENCE_ERROR: return "SEQUENCE_ERROR";
    case code::NOT_SYNCHRONIZED: return "NOT_SYNCHRONIZED";
    default: return "?";
  }
}

uint32_t nextSeq(uint32_t seq)
{
  return seq == UINT32_MAX ? 1u : seq + 1u;
}

const ExportEntry* ExportTable::byNumber(uint16_t number) const
{
  for (const auto& e : entries)
    if (e.number == number) return &e;
  return nullptr;
}

const ExportEntry* ExportTable::byName(const std::string& name) const
{
  for (const auto& e : entries)
    if (e.name == name) return &e;
  return nullptr;
}

uint32_t crc32(const std::string& bytes)
{
  static const std::array<uint32_t, 256> table = [] {
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i)
    {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      t[i] = c;
    }
    return t;
  }();
  uint32_t c = 0xFFFFFFFFu;
  for (const unsigned char b : bytes) c = table[(c ^ b) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

std::string exportTableText(const std::vector<ExportEntry>& entries)
{
  std::vector<const ExportEntry*> sorted;
  for (const auto& e : entries) sorted.push_back(&e);
  std::sort(sorted.begin(), sorted.end(), [](const ExportEntry* a, const ExportEntry* b) { return a->number < b->number; });
  std::ostringstream os;
  for (const auto* e : sorted) os << e->number << ':' << e->name << ':' << e->timeout_ms << '\n';
  return os.str();
}

uint32_t exportTableId(const std::vector<ExportEntry>& entries)
{
  return crc32(exportTableText(entries));
}

namespace
{
const std::regex kName("^[A-Za-z][A-Za-z0-9_]*$");   // TSA's canonical name (model/program.py NAME)

[[noreturn]] void refuse(const std::string& path, const std::string& why)
{
  throw TableRefused("tasks file " + path + ": " + why + " (ADR-0032 Decision 3)");
}

double number(const YAML::Node& n, const std::string& path, const std::string& key)
{
  if (!n || !n.IsScalar()) refuse(path, key + " is missing or not a number");
  try
  {
    return n.as<double>();
  }
  catch (const YAML::Exception&)
  {
    refuse(path, key + " is not a number: '" + n.Scalar() + "'");
  }
}
}  // namespace

ExportTable loadExportTable(const std::string& path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    refuse(path, std::string("does not load: ") + e.what());
  }
  if (!root.IsMap()) refuse(path, "is not a mapping");
  if (!root["schema_version"] || root["schema_version"].as<std::string>("") != "1")
    refuse(path, "schema_version is not 1");
  const YAML::Node tasks = root["tasks"];
  if (!tasks || !tasks.IsMap()) refuse(path, "tasks is missing or not a mapping");

  ExportTable table;
  std::set<long> numbers;
  for (const auto& kv : tasks)
  {
    const std::string name = kv.first.as<std::string>("");
    const YAML::Node task = kv.second;
    if (!task.IsMap()) refuse(path, "task '" + name + "' is not a mapping");
    const YAML::Node ex = task["export"];
    if (!ex) continue;   // a task the PLC cannot request: internal to the application
    const std::string where = "tasks." + name + ".export";
    if (!std::regex_match(name, kName)) refuse(path, "task '" + name + "' is not a canonical name");
    if (!ex.IsMap()) refuse(path, where + " is not a mapping");
    for (const auto& k : ex)
    {
      const std::string key = k.first.as<std::string>("");
      if (key != "number" && key != "timeout_s") refuse(path, where + " has an unknown key '" + key + "'");
    }
    const double n = number(ex["number"], path, where + ".number");
    if (n != std::floor(n) || n < 1 || n > 65535) refuse(path, where + ".number must be an integer 1..65535");
    if (!numbers.insert(static_cast<long>(n)).second)
      refuse(path, where + ".number " + std::to_string(static_cast<long>(n)) + " is used by another task");
    const double t = number(ex["timeout_s"], path, where + ".timeout_s");
    if (!(t > 0.0) || t > 86400.0) refuse(path, where + ".timeout_s must be > 0 and <= 86400");
    ExportEntry e;
    e.number = static_cast<uint16_t>(n);
    e.name = name;
    e.timeout_s = t;
    e.timeout_ms = std::llround(t * 1000.0);
    // the canonical text is in milliseconds (Decision 3): TSA refuses a timeout that is not a whole number of them
    if (e.timeout_ms < 1 || std::fabs(t * 1000.0 - static_cast<double>(e.timeout_ms)) > 1e-6)
      refuse(path, where + ".timeout_s must be a whole number of milliseconds, at least 0.001");
    table.entries.push_back(e);
  }
  if (table.entries.empty()) refuse(path, "no task is exported: the task server has nothing to serve");
  std::sort(table.entries.begin(), table.entries.end(),
            [](const ExportEntry& a, const ExportEntry& b) { return a.number < b.number; });
  table.id = exportTableId(table.entries);

  const YAML::Node declared = root["export_table_id"];
  if (!declared) refuse(path, "export_table_id is missing");
  uint32_t declared_id = 0;
  try
  {
    const long long v = declared.as<long long>();
    if (v < 0 || v > static_cast<long long>(UINT32_MAX)) refuse(path, "export_table_id is not a 32-bit value");
    declared_id = static_cast<uint32_t>(v);
  }
  catch (const YAML::Exception&)
  {
    refuse(path, "export_table_id is not an integer");
  }
  if (declared_id != table.id)
  {
    std::ostringstream os;
    os << "export_table_id " << declared_id << " differs from the table's own " << table.id
       << " (an edited or mixed bundle: regenerate it with TSA)";
    refuse(path, os.str());
  }
  return table;
}

}  // namespace trainit::rti
