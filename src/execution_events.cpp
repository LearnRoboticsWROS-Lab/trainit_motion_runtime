// The canonical execution events of an application run (D0.5, ADR-0030): see execution_events.hpp.
#include "trainit_motion_runtime/execution_events.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <random>
#include <utility>

#include <behaviortree_cpp/decorators/subtree_node.h>

namespace trainit
{

std::optional<StepTag> parseStepName(const std::string& name)
{
  const std::string prefix = kStepNamePrefix;
  if (name.rfind(prefix, 0) != 0) return std::nullopt;
  const std::string rest = name.substr(prefix.size());
  const auto colon = rest.find(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= rest.size()) return std::nullopt;
  StepTag tag{rest.substr(0, colon), rest.substr(colon + 1)};
  if (tag.kind.find(':') != std::string::npos) return std::nullopt;
  return tag;
}

std::optional<std::string> taskOfTreeId(const std::string& tree_id)
{
  const std::string prefix = kTaskTreePrefix;
  if (tree_id.rfind(prefix, 0) != 0 || tree_id.size() == prefix.size()) return std::nullopt;
  return tree_id.substr(prefix.size());
}

std::string makeRunId()
{
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm utc{};
  gmtime_r(&now, &utc);
  char when[32];
  std::strftime(when, sizeof(when), "%Y%m%dT%H%M%SZ", &utc);
  std::random_device rd;
  char hex[16];
  std::snprintf(hex, sizeof(hex), "%08x", static_cast<unsigned>(rd()));
  return std::string("run-") + when + "-" + hex;
}

std::pair<std::string, double> processKindAndValue(const ProcessEvent& e)
{
  switch (e.type)
  {
    case ProcessEventType::ENABLE_PROCESS: return {"process_on", 1.0};
    case ProcessEventType::DISABLE_PROCESS: return {"process_off", 0.0};
    case ProcessEventType::SET_ANALOG_VALUE: return {"set_value", e.value};
    case ProcessEventType::TRIGGER: return {"external", 0.0};
  }
  return {"external", 0.0};
}

// --- the observed process controller ----------------------------------------------------------------

ObservedProcessController::ObservedProcessController(std::unique_ptr<ProcessController> inner,
                                                     std::shared_ptr<Observer> observer)
  : inner_(std::move(inner)), observer_(std::move(observer))
{
}

bool ObservedProcessController::executeEvent(const ProcessEvent& event)
{
  const bool dispatched = inner_->executeEvent(event);
  if (dispatched && observer_ && *observer_) (*observer_)(event);
  return dispatched;
}

void ObservedProcessController::prepare(const std::string& channel, bool analog)
{
  inner_->prepare(channel, analog);
}

// --- the publisher --------------------------------------------------------------------------------

rclcpp::QoS ExecutionEventPublisher::qos()
{
  // VOLATILE: a recorder that subscribes later must never receive an event of the past (D0.5 §8)
  return rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
}

std::unique_ptr<ExecutionEventPublisher> ExecutionEventPublisher::forNode(const rclcpp::Node::SharedPtr& node,
                                                                          BT::Tree& tree)
{
  auto pub = node->create_publisher<Msg>(kExecutionEventsTopic, qos());
  auto clock = node->get_clock();
  return std::make_unique<ExecutionEventPublisher>(
    tree, [pub](const Msg& m) { pub->publish(m); }, [clock]() { return clock->now(); }, makeRunId());
}

ExecutionEventPublisher::ExecutionEventPublisher(BT::Tree& tree, Sink sink, Clock clock, std::string run_id)
  : sink_(std::move(sink)), clock_(std::move(clock)), run_id_(std::move(run_id))
{
  for (const auto& subtree : tree.subtrees)
  {
    const auto owner = taskOfTreeId(subtree->tree_ID);
    for (const auto& node : subtree->nodes)
    {
      if (const auto* call = dynamic_cast<const BT::SubTreeNode*>(node.get()))
      {
        if (const auto called = taskOfTreeId(call->subtreeID()))
        {
          Info info;
          info.is_task = true;
          info.task = *called;
          info_[node.get()] = info;
        }
        continue;
      }
      const auto tag = parseStepName(node->name());
      if (!tag) continue;
      Info info;
      info.task = owner.value_or("");
      info.step = tag->id;
      info.kind = tag->kind;
      if (tag->kind == "grasp")
      {
        BT::applyRecursiveVisitor(node.get(), [&info](const BT::TreeNode* n) {
          if (!info.candidate && n->registrationName() == "SetWaypointFromCandidate") info.candidate = n;
        });
      }
      info_[node.get()] = info;
    }
  }
  enableTransitionToIdle(true);   // RUNNING -> IDLE is how a halted step ends
  subscribeToTreeChanges(tree.rootNode());
}

ExecutionEventPublisher::~ExecutionEventPublisher()
{
  unsubscribeFromTreeChanges();   // before the members callback() uses are destroyed
}

ExecutionEventPublisher::Msg ExecutionEventPublisher::base(uint8_t kind, const Info& info) const
{
  Msg m;
  m.kind = kind;
  m.task_id = info.task;
  if (!info.is_task)
  {
    m.step_id = info.step;
    m.step_kind = info.kind;
  }
  return m;
}

void ExecutionEventPublisher::emit(Msg m)
{
  if (ended_) return;                                       // nothing after RUN_ENDED
  try
  {
    m.header.stamp = clock_();
    m.run_id = run_id_;
    m.seq = seq_ + 1;
    sink_(m);
    seq_ = m.seq;
  }
  catch (...)
  {
    ++dropped_;                                             // observation never fails the run
  }
}

void ExecutionEventPublisher::candidateOf(const Info& info, Msg& m) const
{
  if (!info.candidate) return;
  const auto& ports = info.candidate->config().input_ports;
  const auto from = ports.find("from");
  const auto index = ports.find("index");
  if (from == ports.end() || index == ports.end() || from->second.empty() || index->second.empty()) return;
  if (from->second.front() == '{' || index->second.front() == '{') return;   // only literal references
  const auto bb = info.candidate->config().blackboard;
  if (!bb) return;
  // the index exactly as SetWaypointFromCandidate's integer port reads it: digits only, nothing around them
  if (index->second.size() > 9 || index->second.find_first_not_of("0123456789") != std::string::npos) return;
  const std::string key = from->second + "." + std::to_string(std::stoul(index->second));
  std::string id, provider;
  try
  {
    if (!bb->get(key + ".id", id) || !bb->get(key + ".provider", provider)) return;
  }
  catch (const std::exception&)
  {
    return;   // not (yet) a string on the blackboard: no candidate is stated
  }
  m.candidate_id = id;
  m.candidate_provider = provider;
}

void ExecutionEventPublisher::enter(const BT::TreeNode* node, const Info& info)
{
  Msg m = base(info.is_task ? Msg::TASK_ENTERED : Msg::STEP_ENTERED, info);
  if (!info.is_task && info.kind == "grasp")
  {
    const uint32_t attempt = ++attempts_[{info.task, info.step}];
    attempt_of_[node] = attempt;
    m.attempt_index = attempt;
    candidateOf(info, m);
  }
  active_.push_back(node);
  emit(std::move(m));
}

void ExecutionEventPublisher::exit(const BT::TreeNode* node, const Info& info, uint8_t status)
{
  Msg m = base(info.is_task ? Msg::TASK_EXITED : Msg::STEP_EXITED, info);
  m.status = status;
  if (!info.is_task && info.kind == "grasp")
  {
    const auto it = attempt_of_.find(node);
    m.attempt_index = it == attempt_of_.end() ? 0 : it->second;
    candidateOf(info, m);
  }
  const auto at = std::find(active_.rbegin(), active_.rend(), node);
  if (at != active_.rend()) active_.erase(std::next(at).base());
  emit(std::move(m));
}

void ExecutionEventPublisher::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                                       BT::NodeStatus prev_status, BT::NodeStatus status)
{
  const auto it = info_.find(&node);
  if (it == info_.end()) return;
  using S = BT::NodeStatus;
  try
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const bool was_active = std::find(active_.begin(), active_.end(), &node) != active_.end();
    if (status == S::SKIPPED) return;
    if (status == S::IDLE)
    {
      // a finished node reset to IDLE is nothing; a RUNNING one reset to IDLE was halted
      if (prev_status == S::RUNNING && was_active) exit(&node, it->second, Msg::HALTED);
      return;
    }
    // a node that is not active and starts: from IDLE, or re-ticked from a finished status by a reactive parent
    if (!was_active) enter(&node, it->second);
    if (status == S::SUCCESS || status == S::FAILURE)
      exit(&node, it->second, status == S::SUCCESS ? Msg::SUCCEEDED : Msg::FAILED);
  }
  catch (...)
  {
    ++dropped_;                                             // never thrown into the tick
  }
}

void ExecutionEventPublisher::runStarted()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  emit(base(Msg::RUN_STARTED, Info{}));
}

void ExecutionEventPublisher::runEnded(BT::NodeStatus final_status, bool interrupted)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (ended_) return;
  if (interrupted)
  {
    // a stopped run leaves no scope open: innermost first, every entered task / step ends HALTED
    while (!active_.empty())
    {
      const BT::TreeNode* node = active_.back();
      exit(node, info_.at(node), Msg::HALTED);
    }
  }
  Msg m = base(Msg::RUN_ENDED, Info{});
  m.status = interrupted ? Msg::HALTED
           : final_status == BT::NodeStatus::SUCCESS ? Msg::SUCCEEDED : Msg::FAILED;
  emit(std::move(m));
  ended_ = true;
}

void ExecutionEventPublisher::processDispatched(const ProcessEvent& e)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (ended_) return;
  try
  {
  Msg m = base(Msg::PROCESS_EVENT, Info{});
  // the innermost active entry: an identified step (and its task), or a task with no identified step
  // active inside it -- never a step of the CALLER of that task
  if (!active_.empty())
  {
    const Info& info = info_.at(active_.back());
    m.task_id = info.task;
    if (!info.is_task)
    {
      m.step_id = info.step;
      m.step_kind = info.kind;
    }
  }
  const auto [kind, value] = processKindAndValue(e);
  m.process_channel = e.channel;
  m.process_kind = kind;
  m.process_value = value;
  emit(std::move(m));
  }
  catch (...)
  {
    ++dropped_;                                             // never thrown into the dispatching node
  }
}

}  // namespace trainit
