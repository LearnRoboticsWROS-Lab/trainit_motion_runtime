// =============================================================================
// rti_contract.hpp -- the Robot Task Interface v1 vocabulary and the exported-task table (ADR-0032), PURE: no ROS.
//
// The values mirror trainit_interfaces/msg/RobotTaskCommand and RobotTaskStatus one for one (the ROS binding
// static_asserts it). Codes are stable public contract values: never renumbered or reused (ADR-0032 Decision 7).
//
// The exported-task table is read from the bundle's config/tasks.yaml (TSA >= 8.9.0): every task with an
// `export: {number, timeout_s}` entry, and the top-level `export_table_id` -- CRC-32 (IEEE 802.3) of one line
// "<number>:<name>:<timeout_ms>\n" per exported task in increasing number order (ADR-0032 Decision 3). A job of an
// exported task runs the tree `job_<task>` (TSA's carrier, like `task_<task>`).
// NOT a safety interface (ADR-0033).
// =============================================================================
#ifndef TRAINIT_MOTION_RUNTIME__RTI_CONTRACT_HPP_
#define TRAINIT_MOTION_RUNTIME__RTI_CONTRACT_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace trainit::rti
{

constexpr uint16_t kInterfaceVersion = 1;
constexpr const char* kCommandTopic = "trainit/robot_task/command";   // relative: the robot's slot (ADR-0023)
constexpr const char* kStatusTopic = "trainit/robot_task/status";
constexpr const char* kJobTreePrefix = "job_";                          // TSA's carrier, like task_<name>

enum class Cmd : uint8_t { NONE = 0, START = 1, CANCEL = 2, RESET = 3 };
enum class State : uint8_t { UNAVAILABLE = 0, IDLE = 1, RUNNING = 2, STOPPING = 3, FAULTED = 4 };
enum class Ack : uint8_t { NONE = 0, ACCEPTED = 1, REJECTED = 2 };
enum class Result : uint8_t { NONE = 0, SUCCEEDED = 1, ENDED = 2, CANCELLED = 3, FAILED = 4 };

// Result codes (status.result_code / fault_code) and rejection codes (status.ack_code).
namespace code
{
constexpr uint16_t NONE = 0;
constexpr uint16_t OK = 100;
constexpr uint16_t BIN_EMPTY = 101;
constexpr uint16_t NOTHING_PICKABLE = 102;
constexpr uint16_t PICKABLE_EXHAUSTED = 110;
constexpr uint16_t RECOVERY_EXHAUSTED = 111;
constexpr uint16_t CANCELLED = 200;
constexpr uint16_t TERMINAL_FAULT = 300;
constexpr uint16_t TASK_FAILED = 301;
constexpr uint16_t TASK_TIMEOUT = 302;
constexpr uint16_t REQUESTER_LOST = 303;
constexpr uint16_t RUNTIME_LOST = 304;
constexpr uint16_t STATION_TIMEOUT = 305;
constexpr uint16_t STATION_FAULT = 306;
constexpr uint16_t CANCELLED_WHILE_HOLDING = 307;
constexpr uint16_t EEF_FAULT = 308;
constexpr uint16_t CANCELLED_HOLD_UNKNOWN = 309;
constexpr uint16_t STATION_UNAVAILABLE = 310;
constexpr uint16_t SERVER_STARTED = 311;
constexpr uint16_t REQUESTER_CONFLICT = 312;
constexpr uint16_t INVALID_REQUEST = 400;
constexpr uint16_t UNKNOWN_TASK = 401;
constexpr uint16_t BUSY = 402;
constexpr uint16_t NOT_READY = 403;
constexpr uint16_t FAULTED = 404;
constexpr uint16_t DUPLICATE_JOB = 405;
constexpr uint16_t NOT_CURRENT_JOB = 406;
constexpr uint16_t INVALID_STATE = 407;
constexpr uint16_t SEQUENCE_ERROR = 408;
constexpr uint16_t NOT_SYNCHRONIZED = 409;
}  // namespace code

// Names for logs and the fake PLC's plain words ("?" for a value outside the vocabulary).
const char* cmdName(uint8_t cmd);
const char* stateName(State state);
const char* ackName(Ack ack);
const char* resultName(Result result);
const char* codeName(uint16_t code);   // "OK", "BIN_EMPTY", ..., "NOT_SYNCHRONIZED"; "" for 0

// The next sequence number after `seq`: +1, wrapping from 2^32-1 to 1 (0 is skipped: "no command yet").
uint32_t nextSeq(uint32_t seq);

// One exported task.
struct ExportEntry
{
  uint16_t number{0};
  std::string name;
  double timeout_s{0.0};
  int64_t timeout_ms{0};   // llround(timeout_s * 1000): the canonical text's value
  std::string treeId() const { return kJobTreePrefix + name; }
};

// The exported-task table, sorted by number.
struct ExportTable
{
  std::vector<ExportEntry> entries;
  uint32_t id{0};   // exportTableId(entries)
  const ExportEntry* byNumber(uint16_t number) const;
  const ExportEntry* byName(const std::string& name) const;
};

// A tasks.yaml that does not describe a valid exported-task table: a configuration error (the server exits 2).
struct TableRefused : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

// CRC-32 (IEEE 802.3, reflected 0xEDB88320, init and final xor 0xFFFFFFFF): what Python's zlib.crc32 computes.
uint32_t crc32(const std::string& bytes);
// The canonical text of the table ("<number>:<name>:<timeout_ms>\n" by increasing number) and its CRC-32.
std::string exportTableText(const std::vector<ExportEntry>& entries);
uint32_t exportTableId(const std::vector<ExportEntry>& entries);

// Reads the exported tasks of a bundle's tasks.yaml (schema_version 1). Refused (TableRefused, naming the key):
// a file that does not load, schema_version != 1, an export with a number outside 1..65535 or used twice, a
// timeout_s missing, not a number, <= 0 or above 86400, a task name that is not a canonical name, NO exported task
// (a task server with nothing to serve), or an `export_table_id` missing or different from the table's own.
ExportTable loadExportTable(const std::string& tasks_yaml_path);

}  // namespace trainit::rti

#endif  // TRAINIT_MOTION_RUNTIME__RTI_CONTRACT_HPP_
