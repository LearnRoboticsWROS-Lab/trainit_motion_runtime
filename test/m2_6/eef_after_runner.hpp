// vNext M2.6 AFTER test harness -- a CANONICAL runner in-process: the node a trainit_run_bt would be
// (its name, a cell namespace, optional remap arguments, the contract as parameter overrides), the
// executor sized as the runner sizes it (2 + one thread per device), the EndEffectorRegistry, and
// the BtContext with ctx.eefs set and no legacy GripperController or grasp publisher (a canonical
// run never builds them). The motion runtime is null: no end-effector node dereferences it.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_RUNNER_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_RUNNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "m2_6/capture26.hpp"
#include "trainit_motion_runtime/bt_context.hpp"
#include "trainit_motion_runtime/bt_nodes.hpp"
#include "trainit_motion_runtime/bt_profiles.hpp"
#include "trainit_motion_runtime/end_effector_registry.hpp"
#include "trainit_motion_runtime/process_controller.hpp"

namespace m26
{
namespace after
{

// One tick's outcome: the tree status and its blackboard (to read the outputs it remapped).
struct TickResult
{
  std::string status;
  BT::Blackboard::Ptr bb;
  // a blackboard string, "" when the entry is absent
  std::string get(const std::string& key) const
  {
    try { return bb->get<std::string>(key); }
    catch (const std::exception&) { return {}; }
  }
  bool has(const std::string& key) const { return bb->getEntry(key) != nullptr; }
};

class CanonicalRunner
{
public:
  CanonicalRunner(const std::string& ns, const trainit::eef::Contract& contract,
                  const std::vector<std::string>& remaps = {}, const std::string& name = "trainit_run_bt")
  {
    std::vector<std::string> args{"--ros-args"};
    for (const auto& r : remaps)
    {
      args.push_back("-r");
      args.push_back(r);
    }
    node_ = std::make_shared<rclcpp::Node>(
      name, ns, rclcpp::NodeOptions().use_global_arguments(false).arguments(args));
    registry_ = std::make_unique<trainit::EndEffectorRegistry>(node_, contract);
    process_ = std::make_unique<trainit::MockProcessController>(node_->get_logger());
    ctx_.node = node_;
    ctx_.process = process_.get();
    ctx_.eefs = registry_.get();
    spin_ = std::make_unique<Spinner<rclcpp::executors::MultiThreadedExecutor>>(rclcpp::ExecutorOptions(),
                                                                                 2 + contract.devices.size());
    spin_->executor().add_node(node_);
    spin_->start();
  }
  ~CanonicalRunner()
  {
    spin_->stop();
    spin_->executor().remove_node(node_);
    spin_.reset();
    registry_.reset();
  }
  CanonicalRunner(const CanonicalRunner&) = delete;
  CanonicalRunner& operator=(const CanonicalRunner&) = delete;

  rclcpp::Node::SharedPtr node() const { return node_; }
  // A fresh registry on the SAME node -- its state subscriptions start empty, as a new run's do
  // (DetectObject's process-static feed stays bound to this node).
  void rebuild(const trainit::eef::Contract& contract)
  {
    spin_->stop();
    spin_->executor().remove_node(node_);
    registry_.reset();
    registry_ = std::make_unique<trainit::EndEffectorRegistry>(node_, contract);
    ctx_.eefs = registry_.get();
    spin_ = std::make_unique<Spinner<rclcpp::executors::MultiThreadedExecutor>>(rclcpp::ExecutorOptions(),
                                                                                 2 + contract.devices.size());
    spin_->executor().add_node(node_);
    spin_->start();
  }
  trainit::EndEffectorRegistry& registry() { return *registry_; }
  trainit::EndEffectorDevice& device(const std::string& id) { return *registry_->device(id); }
  trainit::BtContext& context() { return ctx_; }
  // the process controller the context uses (M5 swaps in a recording one)
  void setProcess(trainit::ProcessController* p) { ctx_.process = p; }

  // Ticks `xml` (the inside of one BehaviorTree) the way trainit_run_bt does: tickOnce while
  // RUNNING, 10 ms apart. `prepare` may fill the blackboard first; `factory_hook` may replace a
  // node (M5's motion stand-in). A std::exception is the status "THROW <what>".
  TickResult tick(const std::string& xml, const std::function<void(BT::Blackboard&)>& prepare = {},
                  const std::function<void(BT::BehaviorTreeFactory&)>& factory_hook = {})
  {
    return tickDocument("<root BTCPP_format=\"4\" main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + xml +
                          "</BehaviorTree></root>",
                        "T", prepare, factory_hook);
  }

  TickResult tickDocument(const std::string& document, const std::string& tree_id,
                          const std::function<void(BT::Blackboard&)>& prepare = {},
                          const std::function<void(BT::BehaviorTreeFactory&)>& factory_hook = {})
  {
    BT::BehaviorTreeFactory factory;
    trainit::registerAllNodes(factory);
    if (factory_hook) factory_hook(factory);
    TickResult r;
    r.bb = BT::Blackboard::create();
    r.bb->set<trainit::BtContext*>(trainit::BB_BT_CONTEXT, &ctx_);
    r.bb->set<trainit::MotionProfileRegistry>(trainit::BB_MOTION_PROFILES, trainit::loadMotionProfiles(""));
    r.bb->set<rclcpp::Node::SharedPtr>(trainit::BB_ROS_NODE, node_);
    if (prepare) prepare(*r.bb);
    try
    {
      factory.registerBehaviorTreeFromText(document);
      auto tree = factory.createTree(tree_id, r.bb);
      const auto problems = registry_->checkTree(tree);
      if (!problems.empty())
      {
        r.status = "REFUSED AT START-UP: " + problems.front();
        return r;
      }
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      while (status == BT::NodeStatus::RUNNING)
      {
        status = tree.tickOnce();
        if (status == BT::NodeStatus::RUNNING) std::this_thread::sleep_for(milliseconds(10));
      }
      r.status = BT::toStr(status);
    }
    catch (const std::exception& e)
    {
      r.status = std::string("THROW ") + e.what();
    }
    return r;
  }

  // Waits until every device has a state sample whose sequence is past `after_seq` (0: any).
  bool waitForState(const std::string& device_id, milliseconds budget, uint64_t after_seq = 0)
  {
    return waitUntil([&] {
      const auto s = registry_->device(device_id)->latest();
      return s.sample != nullptr && s.seq > after_seq;
    }, budget);
  }

  // Waits until no state sample of `device_id` has arrived for `quiet` (a paused publisher's last
  // sample has landed), within `budget`.
  bool waitForQuiet(const std::string& device_id, milliseconds quiet, milliseconds budget)
  {
    uint64_t seq = registry_->device(device_id)->latest().seq;
    auto since = Clock::now();
    const auto deadline = Clock::now() + budget;
    while (Clock::now() < deadline)
    {
      std::this_thread::sleep_for(milliseconds(10));
      const uint64_t now_seq = registry_->device(device_id)->latest().seq;
      if (now_seq != seq)
      {
        seq = now_seq;
        since = Clock::now();
      }
      else if (Clock::now() - since >= quiet)
      {
        return true;
      }
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<trainit::EndEffectorRegistry> registry_;
  std::unique_ptr<trainit::ProcessController> process_;
  trainit::BtContext ctx_{};
  std::unique_ptr<Spinner<rclcpp::executors::MultiThreadedExecutor>> spin_;
};

}  // namespace after
}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_EEF_AFTER_RUNNER_HPP_
