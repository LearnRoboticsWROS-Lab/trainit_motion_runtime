// =============================================================================
// trainit_fake_plc_node.cpp -- the FAKE PLC: the Robot Task Interface's reference requester (ADR-0032 Decision
// 10a) as an interactive command line, for commissioning and tests. A TEST / COMMISSIONING TOOL: never a machine
// master in production, never started by TSA, and NOT a safety function (ADR-0033).
//
// It owns the protocol (session, heartbeat, sequence numbers, acknowledgements, the retry, the version / table check):
// its user only says what a PLC programmer says -- reset, start a task as a job, cancel a job -- and reads Ready /
// Busy / Done / Result / Fault in plain words.
//
//   ros2 run trainit_motion_runtime trainit_fake_plc --ros-args [-r __ns:=/<robot slot>] \
//     -p tasks_file:=<bundle>/config/tasks.yaml
// Parameters: tasks_file (REQUIRED: the exported tasks, names <-> numbers, and the export table it checks the server
// against -- Decision 10a rule 1: a requester never commands a server whose table it was not generated for), script ("reset; start place_part 1; sleep 500; ..." run first), stay_on_eof (keep heartbeating when
// stdin closes: a child process in a test).
// =============================================================================
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "trainit_motion_runtime/reference_requester.hpp"
#include "trainit_motion_runtime/rti_contract.hpp"

using trainit::rti::ExportTable;
using trainit::rti::ReferenceRequester;

namespace
{
std::string now()
{
  const auto t = std::chrono::system_clock::now();
  const auto s = std::chrono::system_clock::to_time_t(t);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count() % 1000;
  char buf[32];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&s));
  char out[40];
  std::snprintf(out, sizeof(out), "%s.%03lld", buf, static_cast<long long>(ms));
  return out;
}

// ONE write per line under one lock: the requester's transitions (its executor thread) and the replies to the
// commands (this thread) never interleave inside a line
std::mutex& outMutex()
{
  static std::mutex m;
  return m;
}

void print(const std::string& text)
{
  std::lock_guard<std::mutex> lock(outMutex());
  std::cout << text << std::flush;
}

void say(const std::string& line)
{
  print("[fake PLC " + now() + "] " + line + "\n");
}

const char* kHelp =
  "commands:\n"
  "  reset                         acknowledge the start-up / a fault (Robot needs reset -> Reset -> Ready)\n"
  "  start <task> [job]            request an exported task (its name or PLC number) as a job (default: the next)\n"
  "  cancel [job]                  cancel the running job (default: the last one started)\n"
  "  status                        Ready / Busy / Done / Result / Fault for the last job\n"
  "  tasks                         the exported tasks (name, PLC number, timeout)\n"
  "  pause | resume                stop / restart this PLC's heartbeat (simulates a PLC that hangs)\n"
  "  sleep <ms>                    wait (scripts)\n"
  "  help | quit";

struct Plc
{
  ReferenceRequester& req;
  const ExportTable* table;
  uint32_t next_job{1};
  uint32_t last_job{0};

  std::optional<uint16_t> taskNumber(const std::string& word) const
  {
    if (table)
      if (const auto* e = table->byName(word)) return e->number;
    try
    {
      const long n = std::stol(word);
      if (n >= 0 && n <= 65535) return static_cast<uint16_t>(n);
    }
    catch (...)
    {
    }
    return std::nullopt;
  }

  void report(const ReferenceRequester::Reply& r) { say(r.text); }

  // false: quit
  bool run(const std::string& line)
  {
    std::istringstream in(line);
    std::string cmd;
    if (!(in >> cmd)) return true;
    if (cmd == "quit" || cmd == "exit") return false;
    if (cmd == "help")
    {
      print(std::string(kHelp) + "\n");
    }
    else if (cmd == "reset")
    {
      say("-> RESET");
      report(req.reset());
    }
    else if (cmd == "start")
    {
      std::string task;
      if (!(in >> task))
      {
        say("usage: start <task> [job]");
        return true;
      }
      const auto number = taskNumber(task);
      if (!number)
      {
        say("'" + task + "' is neither an exported task name nor a number (see: tasks)");
        return true;
      }
      uint32_t job = next_job;
      if (long j; in >> j) job = static_cast<uint32_t>(j);
      say("-> START task #" + std::to_string(*number) + " as job " + std::to_string(job));
      const auto r = req.start(*number, job);
      report(r);
      last_job = job;
      if (job >= next_job) next_job = job + 1;
    }
    else if (cmd == "cancel")
    {
      uint32_t job = last_job;
      if (long j; in >> j) job = static_cast<uint32_t>(j);
      say("-> CANCEL job " + std::to_string(job));
      report(req.cancel(job));
    }
    else if (cmd == "status")
    {
      const auto v = req.view(last_job);
      const auto s = req.lastStatus();
      std::ostringstream os;
      os << "Ready=" << (v.ready ? "yes" : "no") << "  Busy=" << (v.busy ? "yes" : "no") << "  Done=" << (v.done ? "yes" : "no")
         << "  Result=" << (v.done ? std::string(trainit::rti::resultName(v.result)) + " " +
                                       trainit::rti::codeName(v.result_code)
                                   : std::string("-"))
         << "  Fault=" << (v.fault ? std::string("yes ") + trainit::rti::codeName(v.fault_code) : std::string("no"));
      if (!v.server_alive) os << "  (robot task server NOT answering)";
      if (v.job_lost) os << "  JOB LOST";
      say(os.str() + (last_job ? "   [job " + std::to_string(last_job) + "]" : ""));
      if (s && !s->detail.empty()) say("detail: " + s->detail);
    }
    else if (cmd == "tasks")
    {
      if (!table)
      {
        say("no tasks_file: tasks by PLC number only");
        return true;
      }
      for (const auto& e : table->entries)
        say("  #" + std::to_string(e.number) + "  " + e.name + "  (timeout " + std::to_string(e.timeout_ms / 1000.0) + " s)");
    }
    else if (cmd == "pause")
    {
      req.setHeartbeatPaused(true);
      say("heartbeat PAUSED: the robot task server will declare this PLC lost after its timeout");
    }
    else if (cmd == "resume")
    {
      req.setHeartbeatPaused(false);
      say("heartbeat resumed");
    }
    else if (cmd == "sleep")
    {
      long ms = 0;
      in >> ms;
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    else
    {
      say("unknown command '" + cmd + "' (help)");
    }
    return true;
  }
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("trainit_fake_plc");
  const std::string tasks_file = node->declare_parameter<std::string>("tasks_file", "");
  const std::string script = node->declare_parameter<std::string>("script", "");
  const bool stay_on_eof = node->declare_parameter<bool>("stay_on_eof", false);

  std::optional<ExportTable> table;
  ReferenceRequester::Options opts;
  if (tasks_file.empty())
  {
    std::cerr << "param 'tasks_file' is required (the bundle's config/tasks.yaml): the fake PLC checks the server's "
                 "export table before commanding (ADR-0032 Decision 10a rule 1) -- exit 2" << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  try
  {
    table = trainit::rti::loadExportTable(tasks_file);
    opts.expected_table_id = table->id;
  }
  catch (const std::exception& e)
  {
    std::cerr << "tasks_file refused: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  ReferenceRequester req(node, opts);
  req.setObserver([](const std::string& line) { say(line); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] { exec.spin(); });

  print(std::string("TrainIt FAKE PLC -- the Robot Task Interface's reference requester (a test / commissioning tool, "
                    "not a\nmachine master; NOT a safety function). Session ") +
        std::to_string(req.session()) +
        (node->get_namespace() == std::string("/") ? "" : std::string(", slot ") + node->get_namespace()) + ".\n" +
        kHelp + "\n");

  Plc plc{req, table ? &*table : nullptr};
  bool go = true;
  if (!script.empty())
  {
    std::istringstream s(script);
    std::string line;
    while (go && rclcpp::ok() && std::getline(s, line, ';')) go = plc.run(line);
  }
  std::string line;
  while (go && rclcpp::ok())
  {
    if (!std::getline(std::cin, line))
    {
      if (!stay_on_eof) break;
      while (rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      break;
    }
    go = plc.run(line);
  }
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
