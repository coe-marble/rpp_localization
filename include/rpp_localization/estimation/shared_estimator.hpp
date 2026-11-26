#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/Estimator.hpp>
#include <rpp_schema/rpp_interfaces/EstimatorDesc.hpp>

#include <rpp_localization/estimation/rpp_model_binding.hpp>
#include <rpp_localization/estimation/replay.hpp>
#include <rpp_localization/estimation/async_replay.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>

#include <optional>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <atomic>

namespace rpp_localization::estimation
{

// Optional common adapter for filters that own a Gaussian posterior. Algorithms
// supply a numerical core and capability requirements. Normal script operations
// use the public Estimator interface; benchmark snapshots optionally inspect
// this common base. The metrics policy is fixed when the plugin is compiled.
// Bump this native ABI tag when estimator or replay storage changes. Node-side
// casts must reject plugins built against an incompatible memory layout.
template<bool CollectMetrics, std::uint32_t NativeAbiVersion = 2>
class SharedEstimatorBase : public rpp_interfaces::Estimator
{
  using Binding = rpp_localization::estimation::shared::ModelBinding;
  using Observation = rpp_localization::estimation::shared::ObservationBindingModel;
  using Pool = rpp_localization::estimation::shared::OutputPool;
  using Vector = rpp_localization::estimation::Vector;
  using Matrix = rpp_localization::estimation::Matrix;
  using Core = rpp_localization::estimation::GaussianFilter;
  using ReplayEventInfo = typename ReplayImplementation<CollectMetrics>::EventInfo;
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using FieldsDesc = rpp_schema::rpp_interfaces::FieldListDesc_Native;
  using Description = rpp_schema::rpp_interfaces::EstimatorDesc;
  using Estimate = std::tuple<rpp::ListConst<double>, rpp::ListConst<double>, std::uint64_t, std::uint8_t>;

  rpp::CallbackResult on_configure(const rpp::ComponentContext& context) override
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate the estimator before configuring");}
    clear_prepared();
    context_ = &context;
    if (!context.is_native_subcomponent("model")) {
      return rpp::CallbackResult::failure("Prepared estimation requires a native model; place RPC at ingress/egress");
    }
    const auto& child = context.get_subcomponent_context("model");
    instance_id_ = child.get_instance_id();
    auto model = child.get_instance<rpp_interfaces::Model>();
    auto description = model->getDesc().as_struct();
    binding_ = std::make_unique<Binding>(description,
      dynamic_cast<const rpp_localization::estimation::NativeStateObservation*>(model.get()));
    const auto process_features = required_process_features();
    if ((binding_->features & process_features) != process_features) {
      return rpp::CallbackResult::failure("Model is missing process operations required by this filter");
    }

    auto requested = context.get_parameter<std::vector<FieldsDesc>>("measurements");
    if (requested.empty() && binding_->observation_layout.size() != 0) {
      FieldsDesc full;
      full.fields = binding_->observation_layout.fields();
      requested.push_back(std::move(full));
    }
    const auto thresholds = context.get_parameter<std::vector<double>>("mahalanobis_thresholds");
    if (!thresholds.empty() && thresholds.size() != requested.size()) {
      return rpp::CallbackResult::failure("Provide one gating threshold per measurement layout");
    }
    const auto observation_features = required_observation_features();
    if (!requested.empty() && (binding_->features & observation_features) != observation_features)
    {
      return rpp::CallbackResult::failure("Model does not provide the required observation operations");
    }
    std::vector<rpp_localization::estimation::ObservationBinding> native_bindings;
    observations_.reserve(requested.size());
    native_bindings.reserve(requested.size());
    for (std::size_t index = 0; index < requested.size(); ++index) {
      observations_.push_back(std::make_unique<Observation>(*binding_,
        rpp_interfaces::FieldLayout(requested[index].fields)));
      native_bindings.push_back({observations_.back().get(), thresholds.empty() ?
        std::numeric_limits<double>::infinity() : thresholds[index]});
    }
    const auto capacity = context.get_parameter<std::int64_t>("history_capacity");
    const auto policy = context.get_parameter<std::string>("replay_noise_policy");
    if (capacity < 0 || capacity > 4096 || (policy != "reject_split" && policy != "constant_rate")) {
      return rpp::CallbackResult::failure("Invalid history_capacity or replay_noise_policy");
    }
    const auto width = static_cast<std::size_t>(binding_->state_space().size());
    if (width > 1024 || (capacity && width * width > 16 * 1024 * 1024 / static_cast<std::size_t>(capacity))) {
      return rpp::CallbackResult::failure("Replay storage exceeds capacity; reduce history_capacity or state width");
    }
    bindings_ = std::move(native_bindings);
    history_capacity_ = static_cast<std::size_t>(capacity);
    noise_policy_ = policy == "constant_rate" ? rpp_localization::estimation::ReplayNoise::constant_rate :
      rpp_localization::estimation::ReplayNoise::reject_split;
    const auto n = binding_->state_space().size();
    state_work_.resize(n); covariance_work_.resize(n, n);
    prepare_execution();
    input_work_.resize(binding_->input_size());
    input_cache_.setZero(binding_->input_size());
    input_cached_ = input_cache_.size() == 0;
    estimates_.prepare(static_cast<std::size_t>(n), static_cast<std::size_t>(n * n));
    configured_ = true;
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the estimator first");}
    Description result;
    auto state = result.state(); binding_->state_layout.write(state);
    auto inputs = result.inputs(); binding_->input_layout.write(inputs);
    auto measurements = result.measurements();
    measurements.resize(observations_.size());
    for (std::size_t index = 0; index < observations_.size(); ++index) {
      auto layout = measurements[index]; observations_[index]->layout.write(layout);
    }
    return Description::Const(std::move(result));
  }

  std::uint8_t initialize(rpp::ListConst<double> state, rpp::ListConst<double> covariance,
    std::uint64_t time) override
  {
    auto code = initialize_instance(state, covariance, time);
    if (!accepted_code(code)) {return code;}
    if (replica_) {
      code = replica_->initialize_instance(state, covariance, time);
      if (!accepted_code(code)) {return code;}
    }
    cancel_replay(); running_.store(this, std::memory_order_release);
    return code;
  }

  std::uint8_t initialize_instance(rpp::ListConst<double> state, rpp::ListConst<double> covariance,
    std::uint64_t time)
  {
    namespace s = rpp_localization::estimation::shared;
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    if (active_) {return rpp_interfaces::contract::CODE_REJECTED;}
    if (!binding_->state_layout.valid_row(state) || !s::read(state, state_work_) ||
      !s::read_matrix(covariance, covariance_work_) ||
      !covariance_validator_->valid(covariance_work_))
    {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    const auto status = core_->initialize(state_work_, covariance_work_, time);
    if (status == rpp_localization::estimation::Status::ok) {
      input_cache_.setZero(); input_cached_ = input_cache_.size() == 0; input_dt_ = 0;
      if (replay_) {replay_->reset();}
    }
    return s::status_code(status, rpp_interfaces::contract::CODE_OK);
  }

  std::uint8_t predict(rpp::ListConst<double> input, rpp::ListConst<double> process_covariance,
    std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt) override
  {
    if (!active_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    const auto code = execution().predict_instance(input, process_covariance, time, dt, input_dt);
    if (async_ && accepted_code(code)) {
      async_->record(AsyncReplay::Kind::Predict, 0, input, process_covariance, time, dt, input_dt);
    }
    return code;
  }

  std::uint8_t predict_instance(rpp::ListConst<double> input, rpp::ListConst<double> process_covariance,
    std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt)
  {
    namespace c = rpp_interfaces::contract;
    namespace s = rpp_localization::estimation::shared;
    if (!active_ || !core_ || !core_->initialized()) {return c::CODE_NOT_READY;}
    if (time < core_->time()) {return c::CODE_STALE_INPUT;}
    if (dt != time - core_->time() || !binding_->input_layout.valid_row(input) ||
      !s::read(input, input_work_) || !s::read_matrix(process_covariance, covariance_work_) ||
      !covariance_validator_->valid(covariance_work_)) {return c::CODE_INVALID_INPUT;}
    const auto result = context_->with_active_component<rpp_interfaces::Model>(model_slot_, selected_name_,
      [&](auto& model) {
        Binding::Call call(*binding_, model);
        const auto status = replay_ ? replay_->predict(input_work_, covariance_work_, time, dt, input_dt) :
          core_->predict(input_work_, covariance_work_, time, dt, input_dt);
        if (status == rpp_localization::estimation::Status::ok) {
          input_cache_ = input_work_; input_dt_ = input_dt; input_cached_ = true;
        }
        return s::status_code(status, binding_->last_code);
      }, instance_id_);
    return result.second.value_or(c::CODE_NOT_READY);
  }

  std::uint8_t correct(std::uint32_t index, rpp::ListConst<double> measurement,
    rpp::ListConst<double> covariance, std::uint64_t time) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return c::CODE_NOT_READY;}
    auto& live = execution();
    if (async_ && live.core_ && time < live.core_->time()) {
      const auto code = live.correct_instance(index, measurement, covariance, time, true);
      if (!accepted_code(code)) {return code;}
      if (!async_->active()) {
        if (time < live.replay_->oldest_time()) {
          if constexpr (CollectMetrics) {++live.metrics_.late_measurements;}
          return c::CODE_STALE_INPUT;
        }
        if (!async_->start([&] {return live.fork_replay_to(idle_instance());})) {return c::CODE_NOT_READY;}
        if constexpr (CollectMetrics) {++replay_jobs_started_;}
      }
      return async_->record(AsyncReplay::Kind::Correct, index, measurement, covariance, time) ?
        c::CODE_LIMITED : c::CODE_BUDGET_EXCEEDED;
    }
    const auto code = live.correct_instance(index, measurement, covariance, time);
    if (async_ && (accepted_code(code) || code == c::CODE_REJECTED)) {
      async_->record(AsyncReplay::Kind::Correct, index, measurement, covariance, time);
    }
    return code;
  }

  std::uint8_t correct_instance(std::uint32_t index, rpp::ListConst<double> measurement,
    rpp::ListConst<double> covariance, std::uint64_t time, bool validate_only = false)
  {
    namespace c = rpp_interfaces::contract;
    namespace s = rpp_localization::estimation::shared;
    if (!active_ || !core_ || !core_->initialized() || !input_cached_) {return c::CODE_NOT_READY;}
    if (time > core_->time() || index >= observations_.size()) {return c::CODE_INVALID_INPUT;}
    auto& observation = *observations_[index];
    if (!observation.layout.valid_row(measurement) || !s::read(measurement, observation.measurement) ||
      !s::read_matrix(covariance, observation.covariance) ||
      !observation.validator->valid(observation.covariance)) {return c::CODE_INVALID_INPUT;}
    if (validate_only) {return c::CODE_OK;}
    const auto result = context_->with_active_component<rpp_interfaces::Model>(model_slot_, selected_name_,
      [&](auto& model) {
        Binding::Call call(*binding_, model);
        const auto status = replay_ ? replay_->correct(index, observation.measurement,
          observation.covariance, input_cache_, time, input_dt_) : core_->correct(index,
          observation.measurement, observation.covariance, input_cache_, time, input_dt_);
        return s::status_code(status, binding_->last_code);
      }, instance_id_);
    return result.second.value_or(c::CODE_NOT_READY);
  }

  Estimate estimate() override
  {return execution().estimate_instance();}

  Estimate estimate_instance()
  {
    namespace c = rpp_interfaces::contract;
    if (!configured_ || !core_ || !core_->initialized()) {
      return {estimates_.empty(), estimates_.empty(), 0, c::CODE_NOT_READY};
    }
    const auto slot = estimates_.acquire();
    if (slot == Pool::unavailable) {
      return {estimates_.empty(), estimates_.empty(), core_->time(), c::CODE_BUDGET_EXCEEDED};
    }
    rpp_localization::estimation::shared::write_vector(core_->state(), estimates_.first(slot));
    rpp_localization::estimation::shared::write_matrix(core_->covariance(), estimates_.second(slot));
    return {estimates_.first_view(slot), estimates_.second_view(slot), core_->time(), c::CODE_OK};
  }

  std::uint8_t reset(rpp::ListConst<double> state, rpp::ListConst<double> covariance,
    std::uint64_t time) override {return initialize(state, covariance, time);}

  void reset() override
  {
    if (!active_ && core_) {
      core_->clear(); input_cached_ = input_cache_.size() == 0; input_dt_ = 0;
      if (replay_) {replay_->reset();}
      if (replica_) {replica_->reset();}
      cancel_replay(); running_.store(this, std::memory_order_release);
    }
  }

  rpp::CallbackResult on_activate() override
  {
    if (!configured_ || !core_ || !core_->initialized()) {
      return rpp::CallbackResult::failure("Initialize the estimator while inactive before activation");
    }
    const auto status = context_->activate_subcomponents();
    if (!status.success) {return status;}
    if (replica_runtime_) {
      const auto activated = replica_runtime_->activate();
      if (!activated.success) {context_->deactivate_subcomponents(); return activated;}
    }
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override
  {
    active_ = false;
    cancel_replay();
    const auto peer = replica_runtime_ ? replica_runtime_->deactivate() : rpp::CallbackResult{};
    const auto own = context_ ? context_->deactivate_subcomponents() : rpp::CallbackResult{};
    return peer.success ? own : peer;
  }
  rpp::CallbackResult on_cleanup() override
  {clear_prepared(); context_ = nullptr; return {};}

  std::optional<ExecutionMetrics> execution_metrics() const
  {
    if constexpr (CollectMetrics) {
      auto result = metrics_;
      if (replica_) {result.merge(replica_->metrics_);}
      result.replay_jobs_started = replay_jobs_started_;
      result.replay_role_switches = replay_role_switches_;
      result.replay_jobs_cancelled = replay_jobs_cancelled_;
      result.replay_pending = replay_phase() != AsyncReplay::Phase::Idle;
      return result;
    }
    else {return std::nullopt;}
  }

  bool replay_enabled() const noexcept {return configured_ && replay_ != nullptr;}

  // Both physical instances must be identically configured and quiescent.
  // The execution owner serializes this with replay and lifecycle operations.
  bool can_fork_replay(const SharedEstimatorBase& target) const
  {
    return configured_ && target.configured_ && replay_ && target.replay_ &&
      bindings_.size() == target.bindings_.size() && replay_->compatible(*target.replay_);
  }

  bool fork_replay_to(SharedEstimatorBase& target)
  {
    if (!active_ || !target.active_ || !can_fork_replay(target) || !replay_->fork_to(*target.replay_)) {return false;}
    target.input_cache_ = input_cache_;
    target.input_dt_ = input_dt_;
    target.input_cached_ = input_cached_;
    return true;
  }

  // Called once on the lifecycle owner, while inactive. The spare is another
  // estimator component with its own model subtree; the script/router stay single.
  void prepare_replica(rpp::RppDataManager& data, const rpp::ComponentRequest& request,
    std::size_t queue_capacity)
  {
    if (active_ || !configured_ || !replay_) {throw std::logic_error("Prepare replay while configured and inactive, with history enabled");}
    if (queue_capacity < 1 || queue_capacity > 4096) {throw std::invalid_argument("Replay queue capacity must be 1..4096");}
    rpp::ConcurrentComponentRuntimeBuilder builder(data);
    auto runtime = std::make_unique<rpp::ConcurrentComponentRuntime>(builder.build_component(request));
    runtime->configure().require_success("Replay estimator configuration");
    auto* peer = dynamic_cast<SharedEstimatorBase*>(runtime->get_instance<rpp_interfaces::Estimator>().get());
    if (!peer || typeid(*this) != typeid(*peer) || !can_fork_replay(*peer)) {
      throw std::invalid_argument("Replay requires identically configured estimator/model instances");
    }
    std::size_t maximum = static_cast<std::size_t>(input_cache_.size());
    const auto square = static_cast<std::size_t>(core_->state().size() * core_->state().size());
    std::size_t maximum_square = square;
    for (const auto& observation : observations_) {
      maximum = std::max(maximum, static_cast<std::size_t>(observation->measurement.size()));
      maximum_square = std::max(maximum_square, static_cast<std::size_t>(observation->covariance.size()));
    }
    if (maximum + maximum_square > 16 * 1024 * 1024 / (queue_capacity + 1)) {
      throw std::invalid_argument("Replay queue storage exceeds capacity; reduce queue capacity or state width");
    }
    auto prepared = std::make_unique<AsyncReplay>();
    prepared->prepare(queue_capacity, maximum, maximum_square);
    if (replica_runtime_) {replica_runtime_->close();}
    replica_runtime_ = std::move(runtime); replica_ = peer; async_ = std::move(prepared);
    running_.store(this, std::memory_order_release);
  }

  AsyncReplay::Phase replay_phase() const noexcept
  {return async_ ? async_->phase() : AsyncReplay::Phase::Idle;}

  // Live owner only, between operations. Adopting a candidate changes a pointer;
  // the physical instances keep their model, algorithm caches and corrected history.
  bool adopt_replay()
  {
    if (!async_ || !async_->try_switch([this] {
        auto* next = &idle_instance(); running_.store(next, std::memory_order_release);
      })) {return false;}
    if constexpr (CollectMetrics) {++replay_role_switches_;}
    return true;
  }

  // Replay/ROS owner only. It never executes the live instance.
  bool service_replay(std::size_t event_limit)
  {
    if (!async_) {return true;}
    return async_->service_batch(event_limit, [this](const AsyncReplay::Batch& batch) {
      namespace c = rpp_interfaces::contract;
      namespace s = rpp_localization::estimation::shared;
      auto& idle = idle_instance();
      if (!idle.active_ || !idle.core_ || !idle.core_->initialized() || !idle.replay_ || !idle.input_cached_) {
        return false;
      }
      const auto describe = [&](std::size_t index) {
        const auto& event = batch[index];
        return ReplayEventInfo{
          event.kind == AsyncReplay::Kind::Predict, event.time, event.dt};
      };
      const auto load = [&](std::size_t index, auto& output, bool validate) {
        const auto& event = batch[index];
        output.prediction = event.kind == AsyncReplay::Kind::Predict;
        output.time = event.time; output.dt = event.dt; output.input_dt = event.input_dt;
        if (output.prediction) {
          if (event.width != static_cast<std::size_t>(output.input.size()) ||
              event.square != static_cast<std::size_t>(output.q.size())) {return Status::invalid_input;}
          output.start = event.time - event.dt;
          for (std::size_t axis = 0; axis < event.width; ++axis) {output.input[axis] = event.values[axis];}
          for (Eigen::Index row = 0; row < output.q.rows(); ++row) {
            for (Eigen::Index column = 0; column < output.q.cols(); ++column) {
              output.q(row, column) = event.covariance[static_cast<std::size_t>(row * output.q.cols() + column)];
            }
          }
          if (validate && (!idle.binding_->input_layout.valid_row(output.input) ||
              !idle.covariance_validator_->valid(output.q))) {return Status::invalid_input;}
        } else {
          if (event.index >= idle.observations_.size()) {return Status::invalid_input;}
          auto& observation = *idle.observations_[event.index];
          if (event.width != static_cast<std::size_t>(observation.measurement.size()) ||
              event.square != static_cast<std::size_t>(observation.covariance.size())) {return Status::invalid_input;}
          output.observation = event.index; output.width = static_cast<Eigen::Index>(event.width);
          for (std::size_t axis = 0; axis < event.width; ++axis) {output.measurement[axis] = event.values[axis];}
          for (Eigen::Index row = 0; row < output.width; ++row) {
            for (Eigen::Index column = 0; column < output.width; ++column) {
              output.r(row, column) = event.covariance[static_cast<std::size_t>(row * output.width + column)];
            }
          }
          if (validate && (!observation.layout.valid_row(output.measurement.head(output.width)) ||
              !observation.validator->valid(output.r.topLeftCorner(output.width, output.width)))) {
            return Status::invalid_input;
          }
        }
        return Status::ok;
      };
      const auto result = idle.context_->template with_active_component<rpp_interfaces::Model>(
        idle.model_slot_, idle.selected_name_, [&](auto& model) {
          Binding::Call call(*idle.binding_, model);
          const auto status = idle.replay_->replay_batch(batch.size(), describe, load);
          if (status == Status::ok) {
            idle.input_cache_ = idle.replay_->latest_input(); idle.input_dt_ = idle.replay_->latest_input_dt();
          }
          return s::status_code(status, idle.binding_->last_code);
        }, idle.instance_id_);
      const auto code = result.second.value_or(c::CODE_NOT_READY);
      return accepted_code(code) || code == c::CODE_REJECTED;
    });
  }

  // Lifecycle owner only, after quiescing live and replay calls.
  void cancel_replay()
  {
    if (async_) {
      if constexpr (CollectMetrics) {if (async_->phase() != AsyncReplay::Phase::Idle) {++replay_jobs_cancelled_;}}
      async_->reset();
    }
  }

protected:
  virtual std::uint8_t required_process_features() const = 0;
  virtual std::uint8_t required_observation_features() const = 0;
  virtual std::unique_ptr<Core> create_filter(
    const rpp_localization::estimation::ProcessModel& model,
    const std::vector<rpp_localization::estimation::ObservationBinding>& observations,
    const rpp::ComponentContext& context) = 0;

  ExecutionMetrics* metrics_sink() noexcept
  {
    if constexpr (CollectMetrics) {return &metrics_;}
    else {return nullptr;}
  }

private:
  static bool accepted_code(std::uint8_t code)
  {return code == rpp_interfaces::contract::CODE_OK || code == rpp_interfaces::contract::CODE_LIMITED;}
  SharedEstimatorBase& execution() const {return *running_.load(std::memory_order_acquire);}
  SharedEstimatorBase& idle_instance() {return &execution() == this ? *replica_ : *this;}
  void prepare_execution()
  {
    namespace s = rpp_localization::estimation::shared;
    replay_.reset(); core_.reset();
    core_ = create_filter(*binding_, bindings_, *context_);
    if (!core_) {throw std::logic_error("Filter factory returned no numerical core");}
    if (history_capacity_) {
      replay_ = std::make_unique<ReplayImplementation<CollectMetrics>>(*core_, bindings_, binding_->input_size(),
        history_capacity_, noise_policy_, metrics_sink());
    }
    const auto make_validator = [&](Eigen::Index size) {
      return std::make_unique<s::CovarianceValidatorImplementation<CollectMetrics>>(size, metrics_sink());
    };
    covariance_validator_ = make_validator(binding_->state_space().size());
    for (auto& observation : observations_) {observation->validator = make_validator(observation->measurement.size());}
  }

  void clear_prepared()
  {
    configured_ = active_ = false;
    cancel_replay(); running_.store(this, std::memory_order_release);
    replica_ = nullptr;
    if (replica_runtime_) {replica_runtime_->close(); replica_runtime_.reset();}
    async_.reset();
    replay_.reset(); core_.reset(); observations_.clear(); binding_.reset(); estimates_.clear();
    bindings_.clear(); covariance_validator_.reset();
    if constexpr (CollectMetrics) {metrics_ = {};}
    replay_jobs_started_ = replay_role_switches_ = replay_jobs_cancelled_ = 0;
  }

  const std::string model_slot_{"model"}, selected_name_;
  const rpp::ComponentContext* context_{nullptr};
  std::string instance_id_;
  std::unique_ptr<Binding> binding_;
  std::vector<std::unique_ptr<Observation>> observations_;
  std::unique_ptr<Core> core_;
  std::unique_ptr<ReplayImplementation<CollectMetrics>> replay_;
  std::vector<rpp_localization::estimation::ObservationBinding> bindings_;
  std::size_t history_capacity_{0};
  rpp_localization::estimation::ReplayNoise noise_policy_{rpp_localization::estimation::ReplayNoise::reject_split};
  Vector state_work_, input_work_, input_cache_;
  Matrix covariance_work_;
  std::unique_ptr<shared::CovarianceValidatorImplementation<CollectMetrics>> covariance_validator_;
  std::conditional_t<CollectMetrics, ExecutionMetrics, std::nullptr_t> metrics_{};
  Pool estimates_;
  std::uint64_t input_dt_{0};
  bool configured_{false}, active_{false}, input_cached_{false};
  std::atomic<SharedEstimatorBase*> running_{this};
  SharedEstimatorBase* replica_{nullptr};
  std::unique_ptr<AsyncReplay> async_;
  std::uint64_t replay_jobs_started_{0}, replay_role_switches_{0}, replay_jobs_cancelled_{0};
  std::unique_ptr<rpp::ConcurrentComponentRuntime> replica_runtime_;
};
}  // namespace rpp_localization::estimation
