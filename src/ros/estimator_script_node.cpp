#include <rpp_localization/ros/estimator_script_node.hpp>
#include <rpp_localization/ros/navigation_interface.hpp>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <map>
#if defined(__linux__)
#include <pthread.h>
#endif

namespace rpp_localization
{
namespace h = ros_helpers;
namespace c = rpp_interfaces::contract;
namespace messages = rpp_ros_interfaces::msg;

EstimatorScriptNode::EstimatorScriptNode(const rclcpp::NodeOptions& options, EstimatorNodeDefaults defaults)
: Node("estimator_script", options), defaults_(std::move(defaults))
{
  activate_on_start_ = h::parameter(*this, "activate_on_start", true);
  predict_current_ = h::parameter(*this, "predict_to_current_time", true);
  initialize_from_measurement_ = h::parameter(*this, "initialize_from_measurement", true);
  const auto frequency = h::parameter(*this, "frequency", 30.0);
  const auto publish_frequency = h::parameter(*this, "publish_frequency", 30.0);
  const auto timeout = h::parameter(*this, "input_timeout_sec", 0.5);
  const auto sensor_timeout = h::parameter(*this, "sensor_timeout", 0.0);
  const auto capacity = h::parameter<std::int64_t>(*this, "queue_capacity", 128);
  if (!std::isfinite(frequency) || frequency <= 0.0 || frequency > 1e9 ||
    !std::isfinite(publish_frequency) || publish_frequency <= 0.0 || publish_frequency > 1e9 ||
    !std::isfinite(timeout) || timeout <= 0.0 || timeout > 1e9 ||
    !std::isfinite(sensor_timeout) || sensor_timeout < 0.0 || sensor_timeout > 1e9 || capacity < 1 || capacity > 4096) {
    throw std::invalid_argument("Invalid frequency, timeouts or queue_capacity");
  }
  worker_period_ = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / frequency));
  const bool profile = h::parameter(*this, "benchmark.enabled", false);
  if (profile && !benchmark_metrics_compiled) {
    throw std::invalid_argument("Benchmark metrics require RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=ON");
  }
  benchmark_.configure(profile, worker_period_.count());
  input_timeout_ = static_cast<std::uint64_t>(timeout * 1e9);
  sensor_timeout_ = static_cast<std::uint64_t>(sensor_timeout * 1e9);
  queue_capacity_ = static_cast<std::size_t>(capacity);
  const auto execution = h::parameter<std::string>(*this, "replay_execution", "ros");
  const auto replay_capacity = h::parameter<std::int64_t>(*this, "replay_queue_capacity", 4096);
  const auto replay_events = h::parameter<std::int64_t>(*this, "replay_events_per_tick", 64);
  if ((execution != "ros" && execution != "worker") || replay_capacity < 1 || replay_capacity > 4096 ||
    replay_events < 1 || replay_events > 4096) {throw std::invalid_argument("Invalid replay execution or queue parameters");}
  replay_on_ros_ = execution == "ros"; replay_capacity_ = static_cast<std::size_t>(replay_capacity);
  replay_events_per_tick_ = static_cast<std::size_t>(replay_events);
  h::parameter(*this, "initial_state", std::vector<double>{});
  h::parameter(*this, "initial_covariance", std::vector<double>{});
  h::parameter(*this, "process_noise_density", std::vector<double>{});
  layout_publisher_ = create_publisher<messages::EstimatorLayout>("~/layout", rclcpp::QoS(1).transient_local());
  estimate_publisher_ = create_publisher<Estimate>("~/estimate", rclcpp::QoS(10));
  state_publisher_ = create_publisher<messages::NumericSample>("~/state", rclcpp::QoS(10));
  status_publisher_ = create_publisher<messages::EstimatorStatus>("~/status", rclcpp::QoS(1).transient_local());
  build_runtime(); prepare();
  if (h::parameter(*this, "expose_navigation_topics", defaults_.expose_navigation)) {navigation_ = std::make_unique<NavigationInterface>(*this);}
  rpp_ros2::SlotManagerRosCallbacks callbacks;
  callbacks.before_access = [this] {enter_management();};
  callbacks.after_access = [this] {
    if (management_depth_ == 1) {observe_estimator();}
    leave_management();
  };
  callbacks.before_operation = [this](const auto& operation) {before_slot_operation(operation);};
  callbacks.after_success = [this](const auto& operation) {slot_changed(operation); return rpp::CallbackResult{};};
  slots_ = std::make_unique<rpp_ros2::SlotManagerRos<rpp::ConcurrentScriptRuntime>>(
    *this, *runtime_, rpp_ros2::SlotManagerRosOptions{}, std::move(callbacks));
  measurement_subscription_ = create_subscription<messages::EstimatorMeasurement>("~/measurement", rclcpp::QoS(100),
    [this](messages::EstimatorMeasurement::ConstSharedPtr message) {
      const auto stamp = h::timestamp(message->stamp); const auto current = now().nanoseconds();
      if (!stamp || current < 0 || *stamp > static_cast<std::uint64_t>(current) || message->layout_revision != revision_) {
        status(c::CODE_INVALID_INPUT, "Invalid measurement time or layout revision"); return;
      }
      status(submit_measurement(message->measurement_index, message->values, message->covariance, *stamp));
    });
  prediction_subscription_ = create_subscription<messages::EstimatorPrediction>("~/prediction", rclcpp::QoS(100),
    [this](messages::EstimatorPrediction::ConstSharedPtr message) {
      const auto stamp = h::timestamp(message->stamp); const auto current = now().nanoseconds();
      if (!stamp || current < 0 || *stamp > static_cast<std::uint64_t>(current) || message->layout_revision != revision_) {
        status(c::CODE_INVALID_INPUT, "Invalid prediction time or layout revision"); return;
      }
      status(submit_prediction(message->inputs, message->process_covariance, *stamp, message->dt_ns, message->input_dt_ns));
    });
  input_subscription_ = create_subscription<messages::EstimatorInput>("~/input", rclcpp::QoS(10),
    [this](messages::EstimatorInput::ConstSharedPtr message) {
      const auto stamp = h::timestamp(message->stamp); const auto current = now().nanoseconds();
      if (!stamp || current < 0 || *stamp > static_cast<std::uint64_t>(current) || message->layout_revision != revision_) {
        status(c::CODE_INVALID_INPUT, "Invalid input time or layout revision"); return;
      }
      status(submit_input(message->values, *stamp, message->input_dt_ns));
    });
  reset_service_ = create_service<rpp_ros_interfaces::srv::ResetEstimate>("~/reset",
    [this](rpp_ros_interfaces::srv::ResetEstimate::Request::SharedPtr request,
      rpp_ros_interfaces::srv::ResetEstimate::Response::SharedPtr response) {reset(*request, *response);});
  enable_service_ = create_service<std_srvs::srv::SetBool>("~/enable",
    [this](std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      ManagementPause pause(*this);
      if (!configured_) {response->message = "Estimator routes are unavailable"; return;}
      const bool active = script_->is_active();
      if (request->data != enabled_ || request->data != active) {cancel_replay();}
      const bool accepted = request->data ? (active || script_->activate()) : script_->deactivate();
      if (accepted) {enabled_ = request->data;}
      response->success = accepted; response->message = accepted ? "Estimator execution updated" : "Lifecycle operation failed";
      status(accepted ? c::CODE_OK : c::CODE_NOT_READY);
    });
  if (benchmark_.enabled()) {
    benchmark_service_ = create_service<std_srvs::srv::Trigger>("~/benchmark_stats",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        ManagementPause pause(*this);
        response->success = true;
        const auto counts = replay_driver_.metrics();
        response->message = benchmark_.json(counts ? &counts->replay_states : nullptr);
        response->message.pop_back();
        response->message += std::string(",\"replay_execution\":\"") + (replay_on_ros_ ? "ros" : "worker") + "\"}";
        if (counts) {
          response->message.pop_back();
          response->message += ",\"execution_counts\":" + estimation::execution_metrics_json(*counts) + "}";
        }
        if (worker_metrics_.available()) {
          response->message.pop_back();
          response->message += ",\"worker_metrics\":" + worker_metrics_.json() + "}";
        }
      });
  }
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
    get_clock(), std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / publish_frequency)),
    [this] {tick();}, get_node_base_interface()->get_context());
  get_node_timers_interface()->add_timer(timer_, nullptr);
  if (replay_on_ros_) {replay_timer_ = create_wall_timer(worker_period_, [this] {run_replay();});}
  status(c::CODE_OK);
  worker_ = std::thread([this] {run_worker();});
}

EstimatorScriptNode::~EstimatorScriptNode()
{
  timer_.reset(); replay_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    stop_.store(true, std::memory_order_release);
  }
  wake_.notify_one();
  if (worker_.joinable()) {worker_.join();}
  slots_.reset(); navigation_.reset(); replay_driver_ = {}; script_.reset();
  if (runtime_) {runtime_->close();}
}

void EstimatorScriptNode::enter_management()
{
  if (management_depth_++ == 0) {
    paused_.store(true, std::memory_order_release);
    management_lock_ = std::unique_lock<std::mutex>(execution_mutex_);
  }
}
void EstimatorScriptNode::leave_management()
{
  if (--management_depth_ == 0) {
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      management_lock_.unlock(); paused_.store(false, std::memory_order_release);
    }
    wake_.notify_one();
  }
}
void EstimatorScriptNode::set_prediction_policy(PredictionPolicy policy)
{ManagementPause pause(*this); cancel_replay(); prediction_policy_ = std::move(policy);}

void EstimatorScriptNode::build_runtime()
{
  const auto library = h::parameter<std::string>(*this, "script_library", "rpp_localization");
  const auto name = h::parameter<std::string>(*this, "script_name", defaults_.script_name);
  const auto configuration = h::parameter<std::string>(*this, "configuration", defaults_.configuration);
  const auto workspace = h::parameter<std::string>(*this, "rpp_workspace", "");
  data_manager_ = std::make_unique<rpp::RppDataManager>(rpp::RPP_HOME,
    workspace.empty() ? ament_index_cpp::get_package_share_path(library).string() : workspace);
  rpp::ConcurrentComponentRuntimeBuilder builder(*data_manager_, rpp::ClockOptions{}, true);
  runtime_ = std::make_unique<rpp::ConcurrentScriptRuntime>(builder.build_script({
    rpp::LibraryScript{library, name}, configuration.empty() ? std::nullopt :
      std::optional<std::string>{configuration}, std::nullopt}));
  if (!runtime_->get_context().is_native_subcomponent("estimator")) {
    throw std::invalid_argument("The numeric worker requires a native estimator; RPC belongs outside the hot path");
  }
  runtime_->configure().require_success("Estimator script configuration");
  const auto path = data_manager_->get_script_description_path_from_library(library, name);
  const auto description = data_manager_->load_script_description(path);
  const auto& assignments = description.get_components(configuration.empty() ? std::nullopt :
    std::optional<std::string>{configuration}).at("estimator");
  if (assignments.size() == 1) {replay_plugin_hint_ = assignments.front().plugin_name;}
  script_ = std::make_unique<GenericLocalizationScript>(runtime_->get_context());
}

void EstimatorScriptNode::prepare()
{
  // Constructor or management thread only, with the worker paused.
  cancel_replay(); replay_estimate_pending_ = false;
  configured_ = initialized_ = primed_ = false;
  if (!runtime_->get_context().is_native_subcomponent("estimator")) {
    throw std::invalid_argument("The numeric worker requires a native estimator");
  }
  replay_driver_.prepare(*runtime_, *data_manager_, replay_on_ros_, replay_capacity_, replay_plugin_hint_);
  script_->prepare();
  const auto& state = script_->state_layout(); const auto n = state.tangent_size();
  const auto square = h::covariance_size(state);
  initial_state_.resize(state.size()); reset_state_.resize(state.size());
  const auto initial = get_parameter("initial_state").as_double_array();
  if (!initial.empty() && initial.size() != state.size()) {throw std::invalid_argument("initial_state has the wrong width");}
  for (std::size_t index = 0; index < state.size(); ++index) {initial_state_[index] = initial.empty() ? 0.0 : initial[index];}
  if (initial.empty()) {
    for (std::size_t field = 0; field < state.fields().size(); ++field) {
      if (state.fields()[field].kind == c::COORDINATE_QUATERNION) {initial_state_[state.ranges()[field].offset + 3] = 1.0;}
    }
  }
  h::covariance_parameter(get_parameter("initial_covariance").as_double_array(), n, initial_covariance_, 1e-9);
  reset_covariance_.resize(square);
  h::covariance_parameter(get_parameter("process_noise_density").as_double_array(), n, density_, 0.0);
  q_.resize(square); input_.resize(script_->input_layout().size()); prediction_input_.resize(input_.size());
  for (std::size_t index = 0; index < input_.size(); ++index) {input_[index] = 0.0;}
  input_ready_ = input_.empty(); input_time_ = input_dt_ = 0;
  ports_.clear(); ports_.reserve(script_->measurement_layouts().size());
  messages::EstimatorLayout description;
  description.revision = ++revision_; description.state = h::describe(state); description.inputs = h::describe(script_->input_layout());
  std::size_t maximum_width = std::max(state.size(), input_.size()), maximum_square = square;
  for (const auto& layout : script_->measurement_layouts()) {
    Port port; port.values.resize(layout.size()); port.covariance.resize(h::covariance_size(layout));
    if (rpp_interfaces::validate_required_fields(state, layout)) {
      port.initial_route = rpp_interfaces::field_route(state, layout);
      for (const auto& field : layout.fields()) {
        const auto selected = rpp_interfaces::find_field(state, field.name);
        const auto range = state.ranges()[selected.value()];
        for (std::size_t axis = 0; axis < range.tangent_size; ++axis) {
          port.initial_covariance_route.push_back(range.tangent_offset + axis);
        }
      }
    }
    maximum_width = std::max(maximum_width, layout.size()); maximum_square = std::max(maximum_square, port.covariance.size());
    ports_.push_back(std::move(port)); description.measurements.push_back(h::describe(layout));
  }
  // Bound aggregate transport storage before allocating the ring.
  if (maximum_width + maximum_square > 16 * 1024 * 1024 / (queue_capacity_ + 1)) {
    throw std::invalid_argument("Numeric queue storage exceeds capacity; reduce queue_capacity or dimensions");
  }
  incoming_.prepare(queue_capacity_, [&] {return Event(maximum_width, maximum_square);});
  outgoing_.prepare(3, [&] {return Output(state.size(), square);});
  ++epoch_; epoch_start_ = 0;
  estimate_.layout_revision = worker_estimate_.layout_revision = revision_;
  estimate_.state.resize(state.size()); estimate_.covariance.resize(square);
  worker_estimate_.state.resize(state.size()); worker_estimate_.covariance.resize(square);
  state_routes_.clear();
  auto next_definitions = state_interface_definitions_;
  std::map<std::string, std::vector<rpp_interfaces::Field>> resources;
  for (const auto& field : state.fields()) {resources[field.resource.empty() ? field.name : field.resource].push_back(field);}
  for (auto& [resource, fields] : resources) {
    const rpp_interfaces::FieldLayout subset(std::move(fields));
    auto existing = next_definitions.find(resource);
    if (existing == next_definitions.end()) {existing = next_definitions.emplace(resource, subset).first;}
    else if (existing->second.size() != subset.size() || !rpp_interfaces::validate_required_fields(subset, existing->second)) {
      throw std::invalid_argument("Cannot redefine published numeric interface '" + resource + "'");
    }
    StateRoute route; route.message.interface_name = resource; route.message.values.resize(subset.size());
    route.indices = rpp_interfaces::field_route(state, existing->second); state_routes_.push_back(std::move(route));
    messages::NumericInterface published; published.name = resource; published.layout = h::describe(existing->second);
    description.state_interfaces.push_back(std::move(published));
  }
  const auto current = now().nanoseconds();
  const std::uint64_t initial_time = ports_.empty() && current >= 0 ? static_cast<std::uint64_t>(current) : 0;
  const auto code = script_->initialize(initial_state_, initial_covariance_, initial_time, activate_on_start_);
  if (!h::success(code)) {throw std::invalid_argument("Estimator rejected the initial state or covariance");}
  state_interface_definitions_ = std::move(next_definitions);
  configured_ = true; routes_dirty_ = false; time_ = initial_time; epoch_start_ = initial_time;
  estimator_instance_id_ = runtime_->get_context().get_subcomponent_context("estimator").get_instance_id();
  initialized_ = ports_.empty();
  for (std::size_t index = 0; index < initial_state_.size(); ++index) {worker_estimate_.state[index] = initial_state_[index];}
  for (std::size_t index = 0; index < initial_covariance_.size(); ++index) {worker_estimate_.covariance[index] = initial_covariance_[index];}
  layout_publisher_->publish(description);
  if (navigation_) {navigation_->accept_layout();}
}

std::uint8_t EstimatorScriptNode::begin(std::uint64_t time, std::optional<std::uint32_t> measurement)
{
  h::copy(initial_state_, reset_state_); h::copy(initial_covariance_, reset_covariance_);
  bool copied = false;
  if (measurement && initialize_from_measurement_ && !ports_[*measurement].initial_route.empty()) {
    const auto& port = ports_[*measurement]; const auto& route = port.initial_route;
    const auto& covariance_route = port.initial_covariance_route;
    const auto n = script_->state_layout().tangent_size();
    for (std::size_t row = 0; row < route.size(); ++row) {reset_state_[route[row]] = port.values[row];}
    // The first direct observation establishes an independent measured block.
    for (const auto selected : covariance_route) {
      for (std::size_t other = 0; other < n; ++other) {
        reset_covariance_[selected * n + other] = 0.0; reset_covariance_[other * n + selected] = 0.0;
      }
    }
    for (std::size_t row = 0; row < covariance_route.size(); ++row) {
      for (std::size_t column = 0; column < covariance_route.size(); ++column) {
        reset_covariance_[covariance_route[row] * n + covariance_route[column]] = port.covariance[row * covariance_route.size() + column];
      }
    }
    copied = true;
  }
  const auto code = script_->reset(reset_state_, reset_covariance_, time, true);
  if (!h::success(code)) {return code;}
  initialized_ = true; primed_ = false; time_ = time;
  replay_estimate_pending_ = false;
  epoch_start_ = time;
  for (std::size_t index = 0; index < reset_state_.size(); ++index) {worker_estimate_.state[index] = reset_state_[index];}
  for (std::size_t index = 0; index < reset_covariance_.size(); ++index) {worker_estimate_.covariance[index] = reset_covariance_[index];}
  capture_estimate(code, false);
  // Initialization is management work; let the worker publish the first snapshot.
  worker_status_.store(c::CODE_OK, std::memory_order_release);
  return copied ? c::CODE_LIMITED : c::CODE_OK;
}

std::uint8_t EstimatorScriptNode::enqueue(EventKind kind, std::uint32_t index,
  const std::vector<double>& values, const std::vector<double>& covariance,
  std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt)
{
  auto* event = incoming_.write();
  if (!event) {return c::CODE_BUDGET_EXCEEDED;}
  if (values.size() > event->values.size() || covariance.size() > event->covariance.size()) {return c::CODE_INVALID_INPUT;}
  for (const auto value : values) {if (!std::isfinite(value)) {return c::CODE_INVALID_INPUT;}}
  for (const auto value : covariance) {if (!std::isfinite(value)) {return c::CODE_INVALID_INPUT;}}
  std::copy(values.begin(), values.end(), event->values.begin());
  std::copy(covariance.begin(), covariance.end(), event->covariance.begin());
  event->kind = kind; event->index = index; event->epoch = epoch_; event->time = time;
  event->dt = dt; event->input_dt = input_dt; event->width = values.size(); event->square = covariance.size();
  event->enqueued_ns = benchmark_.enabled() ? BenchmarkTiming::wall_now() : 0;
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    incoming_.commit();
  }
  wake_.notify_one(); return c::CODE_OK;
}

std::uint8_t EstimatorScriptNode::submit_measurement(std::uint32_t index,
  const std::vector<double>& values, const std::vector<double>& covariance, std::uint64_t time)
{
  if (!configured_ || !enabled_) {return c::CODE_NOT_READY;}
  const auto now_ns = now().nanoseconds();
  if (now_ns < 0 || time > static_cast<std::uint64_t>(now_ns)) {return c::CODE_INVALID_INPUT;}
  if (index >= ports_.size() || values.size() != ports_[index].values.size() ||
    covariance.size() != ports_[index].covariance.size() || !script_->measurement_layouts()[index].valid_row(values)) {return c::CODE_INVALID_INPUT;}
  if (!initialized_) {
    ManagementPause pause(*this);
    if (!script_->is_active()) {return c::CODE_NOT_READY;}
    if (!h::copy(values, ports_[index].values) || !h::copy(covariance, ports_[index].covariance)) {return c::CODE_INVALID_INPUT;}
    const auto code = begin(time, index);
    if (!h::success(code)) {return code;}
    if (code == c::CODE_LIMITED) {return c::CODE_OK;}
  }
  return enqueue(EventKind::Measurement, index, values, covariance, time, 0, 0);
}

std::uint8_t EstimatorScriptNode::submit_prediction(const std::vector<double>& input,
  const std::vector<double>& q, std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt)
{
  if (!configured_ || !enabled_) {return c::CODE_NOT_READY;}
  const auto now_ns = now().nanoseconds();
  if (now_ns < 0 || time > static_cast<std::uint64_t>(now_ns)) {return c::CODE_INVALID_INPUT;}
  if (dt > time || input.size() != input_.size() || q.size() != q_.size() || !script_->input_layout().valid_row(input)) {return c::CODE_INVALID_INPUT;}
  if (!initialized_) {
    ManagementPause pause(*this);
    if (!script_->is_active()) {return c::CODE_NOT_READY;}
    const auto code = begin(time - dt); if (!h::success(code)) {return code;}
  }
  return enqueue(EventKind::Prediction, 0, input, q, time, dt, input_dt);
}
std::uint8_t EstimatorScriptNode::submit_input(const std::vector<double>& values, std::uint64_t time, std::uint64_t input_dt)
{
  if (!configured_) {return c::CODE_NOT_READY;}
  const auto now_ns = now().nanoseconds();
  if (now_ns < 0 || time > static_cast<std::uint64_t>(now_ns) || time < epoch_start_) {return c::CODE_INVALID_INPUT;}
  if (values.size() != input_.size() || !script_->input_layout().valid_row(values)) {return c::CODE_INVALID_INPUT;}
  static const std::vector<double> empty;
  return enqueue(EventKind::Input, 0, values, empty, time, 0, input_dt);
}

std::uint8_t EstimatorScriptNode::predict_to(std::uint64_t time)
{
  if (time < time_) {return c::CODE_STALE_INPUT;}
  const auto dt = time - time_;
  if (!prediction_policy_ && !input_.empty() &&
    (!input_ready_ || input_time_ > time || time - input_time_ > input_timeout_)) {return c::CODE_NOT_READY;}
  for (std::size_t index = 0; index < q_.size(); ++index) {q_[index] = density_[index] * (static_cast<double>(dt) * 1e-9);}
  if (prediction_policy_) {
    const auto code = prediction_policy_(time, dt, worker_estimate_, input_, q_);
    if (!h::success(code)) {return code;}
  }
  const auto code = [this, time, dt] {
    BenchmarkTiming::Scope scope(benchmark_, BenchmarkTiming::Operation::Predict);
    return script_->predict(input_, q_, time, dt, input_dt_);
  }();
  if (h::success(code)) {time_ = time; primed_ = true;}
  return code;
}

std::uint8_t EstimatorScriptNode::process(const Event& event)
{
  if (event.epoch != epoch_) {return c::CODE_STALE_INPUT;}
  if (event.kind == EventKind::Input) {
    if (input_ready_ && !input_.empty() && event.time <= input_time_) {return c::CODE_STALE_INPUT;}
    for (std::size_t index = 0; index < event.width; ++index) {input_[index] = event.values[index];}
    input_time_ = event.time; input_dt_ = event.input_dt; input_ready_ = true; return c::CODE_OK;
  }
  if (!enabled_ || !configured_ || !initialized_ || !script_->is_active()) {return c::CODE_NOT_READY;}
  if (event.kind == EventKind::Prediction) {
    for (std::size_t index = 0; index < event.width; ++index) {prediction_input_[index] = event.values[index];}
    for (std::size_t index = 0; index < event.square; ++index) {q_[index] = event.covariance[index];}
    const auto code = [this, &event] {
      BenchmarkTiming::Scope scope(benchmark_, BenchmarkTiming::Operation::Predict);
      return script_->predict(prediction_input_, q_, event.time, event.dt, event.input_dt);
    }();
    if (h::success(code)) {
      h::copy(prediction_input_, input_); input_time_ = event.time; input_dt_ = event.input_dt;
      input_ready_ = primed_ = true; time_ = event.time;
    }
    return code;
  }
  if (event.time > time_ || !primed_) {
    const auto code = predict_to(std::max(event.time, time_));
    if (!h::success(code)) {return code;}
  }
  auto& port = ports_[event.index];
  for (std::size_t index = 0; index < event.width; ++index) {port.values[index] = event.values[index];}
  for (std::size_t index = 0; index < event.square; ++index) {port.covariance[index] = event.covariance[index];}
  BenchmarkTiming::Scope scope(benchmark_, BenchmarkTiming::Operation::Correct);
  return script_->correct(event.index, port.values, port.covariance, event.time);
}

bool EstimatorScriptNode::capture_estimate(std::uint8_t code, bool publish)
{
  auto* output = publish ? outgoing_.write() : nullptr;
  if (publish && !output && !prediction_policy_) {return true;}
  auto [state, covariance, time, result] = script_->estimate();
  if (!h::success(result) || state.size() != worker_estimate_.state.size() || covariance.size() != worker_estimate_.covariance.size()) {return false;}
  const auto stamp = h::timestamp(time);
  if (!stamp) {return false;}
  worker_estimate_.stamp = *stamp;
  for (std::size_t index = 0; index < state.size(); ++index) {worker_estimate_.state[index] = state[index];}
  for (std::size_t index = 0; index < covariance.size(); ++index) {worker_estimate_.covariance[index] = covariance[index];}
  time_ = time;
  if (!output) {return true;}  // Publication may drop snapshots; estimation keeps running.
  std::copy(worker_estimate_.state.begin(), worker_estimate_.state.end(), output->state.begin());
  std::copy(worker_estimate_.covariance.begin(), worker_estimate_.covariance.end(), output->covariance.begin());
  output->time = time; output->epoch = epoch_; output->code = code;
  output->replay_revision = replay_revision_.load(std::memory_order_relaxed);
  outgoing_.commit(); return true;
}

void EstimatorScriptNode::run_replay()
{
  if (routes_dirty_) {tick();}
  if (!configured_ || !replay_on_ros_) {return;}
  if (published_replay_revision_ != replay_revision_.load(std::memory_order_acquire)) {publish_estimate();}
  if (replay_driver_.phase() == estimation::AsyncReplay::Phase::Cancelled) {
    ManagementPause pause(*this);
    cancel_replay();
    status(c::CODE_REJECTED, "Replay cancelled; the live estimate was retained");
    return;
  }
  if (replay_driver_.phase() != estimation::AsyncReplay::Phase::Running) {return;}
  const bool serviced = [this] {
    BenchmarkTiming::Scope scope(benchmark_, BenchmarkTiming::Operation::Replay);
    return replay_driver_.service(replay_events_per_tick_);
  }();
  if (!serviced) {
    status(c::CODE_INFEASIBLE, "Replay failed; the live estimate was retained");
  } else if (replay_driver_.phase() == estimation::AsyncReplay::Phase::Ready) {
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      replay_ready_.store(true, std::memory_order_release);
    }
    wake_.notify_one();
  }
}

bool EstimatorScriptNode::adopt_replay()
{
  // Worker owner, between operations. Refresh before a state-dependent input
  // policy runs; pool exhaustion delays execution instead of using stale state.
  if (replay_ready_.load(std::memory_order_acquire) &&
    replay_ready_.exchange(false, std::memory_order_acq_rel) && replay_driver_.adopt()) {
    replay_revision_.fetch_add(1, std::memory_order_release);
    replay_estimate_pending_.store(true, std::memory_order_release);
  }
  if (replay_estimate_pending_.load(std::memory_order_acquire)) {
    if (!capture_estimate(c::CODE_OK, false)) {return false;}
    replay_estimate_pending_.store(false, std::memory_order_release);
  }
  return true;
}

void EstimatorScriptNode::cancel_replay()
{
  // ROS/lifecycle owner with the worker paused and no concurrent replay callback.
  replay_driver_.cancel(); replay_ready_.store(false, std::memory_order_release);
}

void EstimatorScriptNode::run_worker()
{
  if (benchmark_.enabled()) {
#if defined(__linux__)
    (void)pthread_setname_np(pthread_self(), "rpp_estimator");
#endif
    worker_metrics_.begin();
  }
  using Clock = std::chrono::steady_clock;
  auto deadline = Clock::now();
  while (!stop_.load(std::memory_order_acquire)) {
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_.wait_until(lock, deadline, [&] {
        return stop_.load(std::memory_order_acquire) ||
          (!paused_.load(std::memory_order_acquire) &&
          ((!replay_estimate_pending_.load(std::memory_order_acquire) && !incoming_.empty()) ||
          replay_ready_.load(std::memory_order_acquire)));
      });
    }
    if (stop_.load(std::memory_order_acquire)) {return;}
    if (paused_.load(std::memory_order_acquire)) {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_.wait(lock, [&] {return stop_.load(std::memory_order_acquire) || !paused_.load(std::memory_order_acquire);});
      deadline = Clock::now(); continue;
    }
    std::unique_lock<std::mutex> execution(execution_mutex_, std::try_to_lock);
    if (!execution.owns_lock()) {deadline = Clock::now() + worker_period_; continue;}
    BenchmarkTiming::Scope batch(benchmark_, BenchmarkTiming::Operation::Estimation,
      benchmark_.enabled() && configured_ && enabled_ && initialized_ && script_->is_active());
    try {
      if (!adopt_replay()) {
        worker_status_.store(c::CODE_BUDGET_EXCEEDED, std::memory_order_release);
        deadline = Clock::now() + worker_period_; continue;
      }
      auto code = c::CODE_OK;
      // Bound ingress work so an overloaded producer cannot starve prediction.
      for (std::size_t processed = 0; processed < queue_capacity_; ++processed) {
        const auto* event = incoming_.front(); if (!event) {break;}
        if (!adopt_replay()) {code = c::CODE_BUDGET_EXCEEDED; break;}
        benchmark_.queued(event->enqueued_ns);
        const auto kind = event->kind;
        code = process(*event); incoming_.pop();
        if (prediction_policy_ && kind != EventKind::Input && initialized_ && script_->is_active()) {
          capture_estimate(code, false);
        }
      }
      const auto current = now().nanoseconds();
      if (!adopt_replay()) {code = c::CODE_BUDGET_EXCEEDED;}
      else if (configured_ && enabled_ && initialized_ && script_->is_active()) {
        if (Clock::now() >= deadline && current >= 0 && static_cast<std::uint64_t>(current) > time_) {
          const auto end = static_cast<std::uint64_t>(current);
          if (predict_current_ || (sensor_timeout_ && end - time_ >= sensor_timeout_)) {code = predict_to(end);}
        }
        capture_estimate(code);
      } else {code = c::CODE_NOT_READY;}
      worker_status_.store(code, std::memory_order_release);
    } catch (...) {
      // No ROS logging or exception-message construction on the numeric worker.
      worker_status_.store(c::CODE_INFEASIBLE, std::memory_order_release);
    }
    if (Clock::now() >= deadline) {deadline = Clock::now() + worker_period_;}
  }
}

void EstimatorScriptNode::publish_estimate()
{
  bool available = false;
  for (std::size_t count = 0; count < 3; ++count) {
    const auto* output = outgoing_.front(); if (!output) {break;}
    if (output->epoch == epoch_) {
      if (const auto stamp = h::timestamp(output->time)) {
        estimate_.stamp = *stamp;
        std::copy(output->state.begin(), output->state.end(), estimate_.state.begin());
        std::copy(output->covariance.begin(), output->covariance.end(), estimate_.covariance.begin());
        published_replay_revision_ = output->replay_revision;
        available = true;
      }
    }
    outgoing_.pop();
  }
  if (!available) {return;}
  estimate_publisher_->publish(estimate_);
  for (auto& route : state_routes_) {
    route.message.stamp = estimate_.stamp;
    for (std::size_t index = 0; index < route.indices.size(); ++index) {route.message.values[index] = estimate_.state[route.indices[index]];}
    state_publisher_->publish(route.message);
  }
  if (navigation_) {navigation_->publish(estimate_);}
}

void EstimatorScriptNode::reset(const rpp_ros_interfaces::srv::ResetEstimate::Request& request,
  rpp_ros_interfaces::srv::ResetEstimate::Response& response, bool preserve_enable)
{
  ManagementPause pause(*this);
  const auto stamp = h::timestamp(request.stamp); const auto current = now().nanoseconds();
  if (!configured_ || !stamp || current < 0 || *stamp > static_cast<std::uint64_t>(current) ||
    !script_->state_layout().valid_row(request.state) || !h::copy(request.state, reset_state_) ||
    !h::copy(request.covariance, reset_covariance_)) {response.status = c::CODE_INVALID_INPUT; return;}
  response.status = script_->reset(reset_state_, reset_covariance_, *stamp, request.activate);
  if (h::success(response.status)) {
    initialized_ = true; primed_ = false; time_ = *stamp; epoch_start_ = *stamp;
    replay_estimate_pending_ = false;
    if (!preserve_enable) {enabled_ = request.activate;}
    incoming_.reset(); outgoing_.reset(); ++epoch_;
    input_ready_ = input_.empty(); input_time_ = input_dt_ = 0;
    for (std::size_t index = 0; index < reset_state_.size(); ++index) {worker_estimate_.state[index] = reset_state_[index];}
    for (std::size_t index = 0; index < reset_covariance_.size(); ++index) {worker_estimate_.covariance[index] = reset_covariance_[index];}
    if (request.activate) {capture_estimate(response.status, false);}
    if (navigation_) {navigation_->reset();}
  }
  status(response.status);
}

void EstimatorScriptNode::slot_changed(const rpp_ros2::SlotOperation& operation)
{
  using Kind = rpp_ros2::SlotOperation::Kind;
  if (operation.kind == Kind::Configure || operation.kind == Kind::Reload || operation.kind == Kind::Switch) {
    configured_ = false; routes_dirty_ = true;
    replay_estimate_pending_ = false;
  }
  if (operation.kind == Kind::Cleanup || operation.kind == Kind::Unload) {
    configured_ = initialized_ = false; incoming_.reset(); outgoing_.reset(); ++epoch_;
    replay_estimate_pending_ = false;
  }
}

void EstimatorScriptNode::before_slot_operation(const rpp_ros2::SlotOperation& operation)
{
  cancel_replay();
  if (operation.kind == rpp_ros2::SlotOperation::Kind::Reload && operation.slot == "estimator") {
    const auto& target = runtime_->get_context().get_subcomponent_context("estimator", operation.target);
    // A fresh estimator needs initialize() before activate(). Reload an inactive
    // root, then let prepare() initialize both physical instances together.
    if (target.is_active()) {
      configured_ = false; routes_dirty_ = true;
      target.deactivate().require_success("Pause estimator for reload");
    }
  }
}

void EstimatorScriptNode::observe_estimator() noexcept
{
  // A failed reload may still have replaced the component. Check identity after
  // every slot access, while paused, before any old native callback can resume.
  std::string selected;
  try {selected = runtime_->get_context().get_subcomponent_context("estimator").get_instance_id();}
  catch (...) {}
  if (selected == estimator_instance_id_) {return;}
  estimator_instance_id_ = std::move(selected);
  configured_ = initialized_ = false; routes_dirty_ = !estimator_instance_id_.empty();
  replay_driver_ = {}; replay_ready_.store(false, std::memory_order_release);
  replay_estimate_pending_.store(false, std::memory_order_release);
  incoming_.reset(); outgoing_.reset(); ++epoch_;
}

void EstimatorScriptNode::tick()
{
  try {
    if (routes_dirty_) {
      ManagementPause pause(*this);
      const auto& selected = runtime_->get_context().get_subcomponent_context("estimator");
      if (selected.is_active() && !selected.deactivate().success) {
        status(c::CODE_NOT_READY, "Cannot pause estimator for route preparation"); return;
      }
      prepare();
    }
    const auto current = now().nanoseconds();
    if (current < 0) {status(c::CODE_INVALID_INPUT, "Negative ROS time"); return;}
    const auto time = static_cast<std::uint64_t>(current);
    if (time < previous_now_ && configured_) {
      ManagementPause pause(*this);
      const auto code = script_->reset(initial_state_, initial_covariance_, time, enabled_ && script_->is_active());
      if (!h::success(code)) {status(code, "Clock-reset initialization failed"); return;}
      initialized_ = ports_.empty(); primed_ = false; time_ = time;
      replay_estimate_pending_ = false;
      input_ready_ = input_.empty(); input_time_ = input_dt_ = 0;
      for (std::size_t index = 0; index < initial_state_.size(); ++index) {worker_estimate_.state[index] = initial_state_[index];}
      for (std::size_t index = 0; index < initial_covariance_.size(); ++index) {worker_estimate_.covariance[index] = initial_covariance_[index];}
      epoch_start_ = time;
      incoming_.reset(); outgoing_.reset(); ++epoch_;
      if (navigation_) {navigation_->reset();}
    }
    previous_now_ = time;
    if (navigation_) {navigation_->process_measurements(time);}
    publish_estimate(); status(worker_status_.load(std::memory_order_acquire));
  } catch (const std::exception& error) {
    ManagementPause pause(*this); configured_ = false; status(c::CODE_NOT_READY, error.what());
  }
}

void EstimatorScriptNode::status(std::uint8_t code, const std::string& message)
{
  messages::EstimatorStatus output;
  output.stamp = now(); output.layout_revision = revision_; output.configured = configured_;
  output.enabled = enabled_; output.initialized = initialized_; output.status = code; output.message = message;
  status_publisher_->publish(output);
}
}  // namespace rpp_localization
