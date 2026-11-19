// bt_job_executor.cpp -- trainit_task_server's job executor (P0.a). See the header.
#include "trainit_motion_runtime/bt_job_executor.hpp"

#include <thread>

#include <behaviortree_cpp/scripting/script_parser.hpp>

namespace trainit::rti
{

std::optional<bool> readHeld(const BT::Blackboard::Ptr& bb)
{
  const auto entry = bb->getEntry(kHoldKey);
  if (!entry) return std::nullopt;
  std::unique_lock<std::mutex> lock(entry->entry_mutex);
  const BT::Any& v = entry->value;
  if (v.empty()) return std::nullopt;
  try
  {
    if (v.isType<bool>()) return v.cast<bool>();
    if (v.isNumber())
    {
      const double d = v.cast<double>();
      if (d == 0.0) return false;
      if (d == 1.0) return true;
      return std::nullopt;
    }
    if (v.isString())
    {
      const auto s = v.cast<std::string>();
      if (s == "true") return true;
      if (s == "false") return false;
    }
  }
  catch (const std::exception&)
  {
  }
  return std::nullopt;   // anything else: UNKNOWN (fail closed)
}

void seedHeld(const BT::Blackboard::Ptr& bb, bool held)
{
  BT::Ast::Environment env{bb, nullptr};
  const auto r = BT::ParseScriptAndExecute(env, std::string(kHoldKey) + (held ? " := true" : " := false"));
  if (!r) throw std::runtime_error("seeding " + std::string(kHoldKey) + " failed: " + r.error());
}

namespace
{
std::string readString(const BT::Blackboard::Ptr& bb, const char* key)
{
  try
  {
    if (!bb->getEntry(key)) return "";
    return bb->get<std::string>(key);
  }
  catch (const std::exception&)
  {
    return "";
  }
}
}  // namespace

BtJobExecutor::BtJobExecutor(RunnerStack& stack, BT::BehaviorTreeFactory& factory) : stack_(stack), factory_(factory)
{
  events_pub_ = stack_.node->create_publisher<ExecutionEventPublisher::Msg>(kExecutionEventsTopic,
                                                                           ExecutionEventPublisher::qos());
}

void BtJobExecutor::start(const StartJob& job)
{
  {
    std::lock_guard<std::mutex> lock(m_);
    // The state machine starts a job only from IDLE, after the previous job ENDED (jobEnded): a pending job that was
    // never picked up cannot exist. Logged, not asserted, should that invariant ever break.
    if (pending_)
      RCLCPP_ERROR(stack_.node->get_logger(), "job %u replaces job %u that never started (an RTI invariant broke)",
                   job.job_id, pending_->job_id);
    pending_ = job;
  }
  cv_.notify_all();
}

void BtJobExecutor::requestStop(uint64_t counter)
{
  // Invariant: `stop_` only ever names the current or the pending job, and only that job's runJob clears it (a
  // compare-exchange on its own counter), so a stop can never reach the job after it.
  bool running = false;
  {
    std::lock_guard<std::mutex> lock(m_);
    if (current_ == counter) running = true;
    else if (!pending_ || pending_->counter != counter) return;   // not this job any more: never stop the next one
    stop_ = counter;
  }
  if (running)
  {
    // the motion in flight stops now (move_group's execution "stop"); the tick thread halts the tree once the
    // blocked tick returns (a halt is not preemption: H0). Re-asserted every status period while STOPPING.
    try
    {
      const MotionResult r = stack_.runtime->stop();
      if (!r.success) RCLCPP_ERROR(stack_.node->get_logger(), "MotionRuntime::stop failed: %s", r.stage.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(stack_.node->get_logger(), "MotionRuntime::stop threw: %s (re-asserted next period)", e.what());
    }
  }
}

void BtJobExecutor::run()
{
  while (rclcpp::ok())
  {
    StartJob job;
    {
      std::unique_lock<std::mutex> lock(m_);
      cv_.wait_for(lock, std::chrono::milliseconds(100), [&] { return pending_.has_value(); });
      if (!pending_) continue;
      job = *pending_;
      pending_.reset();
      current_ = job.counter;
    }
    try
    {
      runJob(job);
    }
    catch (const std::exception& e)
    {
      failJob(job, e.what());
    }
    catch (...)
    {
      failJob(job, "an unknown exception");
    }
    clearStop(job.counter);
    {
      std::lock_guard<std::mutex> lock(m_);
      current_ = 0;
    }
  }
}

void BtJobExecutor::clearStop(uint64_t counter)
{
  uint64_t mine = counter;
  stop_.compare_exchange_strong(mine, 0);   // only THIS job's stop: a stop already naming another job stays
}

void BtJobExecutor::failJob(const StartJob& job, const std::string& what)
{
  // Anything that escaped runJob: the job FAILS (once -- a report after the job already ended is ignored by the
  // state machine), its held state UNKNOWN (where it stopped is not known: fail closed). The text stays in the log.
  RCLCPP_ERROR(stack_.node->get_logger(), "job %u (%s): an internal error ended it: %s", job.job_id,
               job.task.c_str(), what.c_str());
  JobEnd end;
  end.tree = JobEnd::Tree::FAILURE;
  end.holding = std::nullopt;
  end.reason = "an internal error ended the job";
  try
  {
    if (server_) server_->jobEnded(job.counter, end);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(stack_.node->get_logger(), "job %u: reporting its end threw: %s", job.job_id, e.what());
  }
}

void BtJobExecutor::runJob(const StartJob& job)
{
  const auto log = stack_.node->get_logger();
  auto bb = BT::Blackboard::create();
  JobEnd end;
  end.holding = job.holding;
  BT::Tree tree;
  try
  {
    stack_.seed(bb, false);
    if (job.holding) seedHeld(bb, *job.holding);   // unknown: the key stays absent (read as HELD by the motions)
    tree = factory_.createTree(job.tree_id, bb);
  }
  catch (const std::exception& e)
  {
    // nothing has moved: the held state is still the seed's
    RCLCPP_ERROR(log, "job %u (%s): the job tree %s did not load: %s", job.job_id, job.task.c_str(),
                 job.tree_id.c_str(), e.what());
    end.tree = JobEnd::Tree::FAILURE;
    end.reason = "the job could not be prepared";   // neutral on the wire; the specifics are logged above
    if (server_) server_->jobEnded(job.counter, end);
    return;
  }

  const auto clock = stack_.node->get_clock();
  const auto pub = events_pub_;
  auto events = std::make_unique<ExecutionEventPublisher>(
    tree, [pub](const ExecutionEventPublisher::Msg& m) { pub->publish(m); }, [clock]() { return clock->now(); },
    server_ ? server_->runId(job.counter) : std::to_string(job.counter));
  events->setJob(job.job_id);
  *stack_.process_observer = [&events](const ProcessEvent& e) { events->processDispatched(e); };
  struct ResetObserver
  {
    std::shared_ptr<ObservedProcessController::Observer> observer;
    ~ResetObserver() { *observer = nullptr; }
  } const observer_reset{stack_.process_observer};
  const EndEventsOnShutdown end_events_on_shutdown(*events);
  const std::size_t faults_before = stack_.eefs ? stack_.eefs->latchedFaults().size() : 0;

  RCLCPP_INFO(log, "job %u (%s, task #%u): run %s, tree %s, held state %s", job.job_id, job.task.c_str(), job.number,
              events->runId().c_str(), job.tree_id.c_str(),
              job.holding ? (*job.holding ? "HELD" : "free") : "UNKNOWN");
  // P0.b (D2, D3, D5): the machine-channel registry's JOB scope -- this job's outstanding requests and its first
  // machine failure start empty; a machine step's activity reaches the RTI detail while the job is RUNNING. The
  // PROCESS scope (the last request written, the seq) survives the job: a cancelled job's request still counts.
  MachineChannels* const machines = stack_.machines.get();
  if (machines)
  {
    RobotTaskServer* const server = server_;
    const uint64_t counter = job.counter;
    machines->beginJob([server, counter](const std::string& text) {
      if (server) server->jobActivity(counter, text);
    });
  }
  struct EndMachineJob
  {
    MachineChannels* machines;
    ~EndMachineJob()
    {
      if (machines) machines->endJob();
    }
  } const end_machine_job{machines};
  events->runStarted();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  bool halted = false;
  bool threw = false;
  try
  {
    const StopOnShutdown stop_on_shutdown(*stack_.runtime, log);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      if (stop_.load() == job.counter)
      {
        tree.haltTree();   // the tick has returned: halt what is still running
        halted = true;
        break;
      }
      status = tree.tickOnce();
      if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(log, "job %u: the tree threw: %s; it is halted and the job fails", job.job_id, e.what());
    try
    {
      tree.haltTree();
    }
    catch (const std::exception& h)
    {
      RCLCPP_ERROR(log, "halting the tree threw: %s", h.what());
    }
    status = BT::NodeStatus::FAILURE;
    threw = true;
    end.reason = "the task raised an internal error";   // neutral on the wire; the specifics are logged above
  }
  const bool shutting_down = !rclcpp::ok() && status == BT::NodeStatus::RUNNING && !halted;
  if (shutting_down)
  {
    try
    {
      tree.haltTree();
    }
    catch (...)
    {
    }
    halted = true;
  }

  end.tree = halted ? JobEnd::Tree::HALTED
           : status == BT::NodeStatus::SUCCESS ? JobEnd::Tree::SUCCESS : JobEnd::Tree::FAILURE;
  end.outcome = readString(bb, kOutcomeKey);
  // where a throwing tree stopped is not known: its held state is UNKNOWN (fail closed), whatever the key says
  end.holding = threw ? std::nullopt : readHeld(bb);
  const std::string gate = readString(bb, kGateKey);
  if (!gate.empty()) end.reason += (end.reason.empty() ? "" : "; ") + gate;
  if (machines)
  {
    // P0.b (D3): the job's machine failure, whatever the tree's fault gate -- decide() gives its STATION code
    end.station_code = machines->jobStationCode();
    end.station_text = machines->jobStationText();
  }
  if (stack_.eefs)
  {
    const auto faults = stack_.eefs->latchedFaults();
    end.eef_fault = faults.size() > faults_before;
    for (std::size_t k = faults_before; k < faults.size(); ++k)
      RCLCPP_ERROR(log, "job %u: end-effector FAULT latched: %s", job.job_id, faults[k].c_str());
  }
  std::optional<Decided> d;
  if (!shutting_down && server_) d = server_->jobEnded(job.counter, end);
  events->runEnded(status, halted, d ? d->result_code : 0);
}

}  // namespace trainit::rti
