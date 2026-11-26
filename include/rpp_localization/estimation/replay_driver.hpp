#pragma once

#include <rpp_localization/estimation/shared_estimator.hpp>
#include <functional>

namespace rpp_localization::estimation
{
// Cold native binding for an execution owner. Scripts use only Estimator;
// benchmark-policy selection stays within the estimator layer. The execution
// owner quiesces worker and replay callbacks before prepare/cancel/metrics.
class ReplayDriver
{
public:
  void prepare(rpp::ConcurrentScriptRuntime& runtime, rpp::RppDataManager& data,
    bool enabled, std::size_t queue_capacity, const std::string& linked_plugin_hint = {})
  {
    *this = ReplayDriver{};
    const auto& child = runtime.get_context().get_subcomponent_context("estimator");
    auto instance = child.get_instance<rpp_interfaces::Estimator>();
    if (auto* shared = dynamic_cast<SharedEstimatorBase<false>*>(instance.get())) {
      bind(*shared, std::move(instance), runtime, data, enabled, queue_capacity, linked_plugin_hint); return;
    }
    if (auto* shared = dynamic_cast<SharedEstimatorBase<true>*>(instance.get())) {
      bind(*shared, std::move(instance), runtime, data, enabled, queue_capacity, linked_plugin_hint); return;
    }
    if (enabled) {
      throw std::invalid_argument(
        "ROS replay requires an estimator built against the current SharedEstimatorBase headers. "
        "After rebuilding the node, run 'rpp library refresh rpp_localization --jobs 4'. "
        "For other estimator implementations, use replay_execution=worker.");
    }
  }

  bool adopt() const {return adopt_ && adopt_();}
  bool service(std::size_t limit) const {return !service_ || service_(limit);}
  void cancel() const {if (cancel_) {cancel_();}}
  AsyncReplay::Phase phase() const {return phase_ ? phase_() : AsyncReplay::Phase::Idle;}
  std::optional<ExecutionMetrics> metrics() const {return metrics_ ? metrics_() : std::nullopt;}

private:
  template<bool Profiled>
  void bind(SharedEstimatorBase<Profiled>& estimator, std::shared_ptr<rpp_interfaces::Estimator> instance,
    rpp::ConcurrentScriptRuntime& runtime, rpp::RppDataManager& data, bool enabled, std::size_t capacity,
    const std::string& linked_plugin_hint)
  {
    if (enabled && estimator.replay_enabled()) {
      const auto& child = runtime.get_context().get_subcomponent_context("estimator");
      rpp::ComponentRequest request;
      for (const auto& status : runtime.get_subcomponent_statuses("estimator")) {
        if (status.instance_id != child.get_instance_id()) {continue;}
        const auto record = data.load_component_info(status.component);
        if (const auto* component = std::get_if<rpp::ComponentRecord>(&record)) {
          request = {component->folder, component->plugin_name};
        } else {
          if (linked_plugin_hint.empty()) {throw std::invalid_argument("A linked estimator needs its plugin identity to construct replay");}
          request = {status.component, linked_plugin_hint};
        }
        break;
      }
      if (request.path.empty()) {throw std::invalid_argument("Cannot resolve the selected estimator profile for replay");}
      estimator.prepare_replica(data, request, capacity);
    }
    keep_alive_ = std::move(instance);
    adopt_ = [&estimator] {return estimator.adopt_replay();};
    service_ = [&estimator](std::size_t limit) {return estimator.service_replay(limit);};
    cancel_ = [&estimator] {estimator.cancel_replay();};
    phase_ = [&estimator] {return estimator.replay_phase();};
    metrics_ = [&estimator] {return estimator.execution_metrics();};
  }
  std::shared_ptr<rpp_interfaces::Estimator> keep_alive_;
  std::function<bool()> adopt_;
  std::function<bool(std::size_t)> service_;
  std::function<void()> cancel_;
  std::function<AsyncReplay::Phase()> phase_;
  std::function<std::optional<ExecutionMetrics>()> metrics_;
};
}  // namespace rpp_localization::estimation
