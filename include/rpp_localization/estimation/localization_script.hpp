#pragma once

#include <rpp_cpp/context.hpp>
#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/Estimator.hpp>
#include <rpp_interfaces/signal_router.hpp>

#include <optional>
#include <tuple>

namespace rpp_localization
{

// Shared-interface localization coordinator. ROS ingress can supply values in
// source field order; the router and covariance routes are prepared once.
// The application serializes calls and pauses execution before prepare().
class GenericLocalizationScript
{
  using Router = rpp_interfaces::SignalRouter;
public:
  using Estimate = std::tuple<rpp::ListConst<double>, rpp::ListConst<double>, std::uint64_t, std::uint8_t>;
  RPP_COMPONENTS({"estimator", "rpp_interfaces::Estimator"})

  explicit GenericLocalizationScript(const rpp::ComponentContext& context) : context_(context)
  {empty_.resize(0);}

  void prepare(std::optional<rpp_interfaces::FieldLayout> input_source = {},
    const std::vector<rpp_interfaces::FieldLayout>& measurement_sources = {})
  {
    const auto& child = context_.get_subcomponent_context(estimator_slot_);
    if (child.is_active()) {throw std::logic_error("Deactivate the estimator before preparing routes");}
    prepared_ = false;
    auto estimator = child.get_instance<rpp_interfaces::Estimator>();
    const auto description = estimator->getDesc().as_struct();
    instance_id_ = child.get_instance_id();
    state_layout_ = rpp_interfaces::FieldLayout(description.state.fields);
    if (!measurement_sources.empty() && measurement_sources.size() != description.measurements.size()) {
      throw std::invalid_argument("Provide one source layout per estimator measurement port");
    }
    router_ = Router{};
    measurements_.clear();
    state_signal_ = router_.add_signal("estimate");
    router_.add_source(state_signal_, "estimator", state_layout_);
    state_writer_ = router_.connect_output(state_signal_, state_layout_);
    state_reader_ = router_.connect_input(state_signal_, state_layout_);
    const rpp_interfaces::FieldLayout required_input(description.inputs.fields);
    input_layout_ = required_input;
    measurement_layouts_.clear();
    const auto available_input = input_source ? *input_source : required_input;
    input_signal_ = router_.add_signal("input");
    router_.add_source(input_signal_, "input", available_input);
    input_writer_ = router_.connect_output(input_signal_, available_input);
    input_reader_ = router_.connect_input(input_signal_, required_input);
    measurements_.reserve(description.measurements.size());
    for (std::size_t index = 0; index < description.measurements.size(); ++index) {
      const rpp_interfaces::FieldLayout required(description.measurements[index].fields);
      measurement_layouts_.push_back(required);
      const auto& available = measurement_sources.empty() ? required : measurement_sources[index];
      const auto signal = router_.add_signal("measurement" + std::to_string(index));
      router_.add_source(signal, "measurement", available);
      MeasurementRoute route;
      route.writer = router_.connect_output(signal, available);
      route.reader = router_.connect_input(signal, required);
      route.source_width = available.tangent_size();
      route.covariance_indices.reserve(required.tangent_size());
      for (const auto& field : required.fields()) {
        const auto source = rpp_interfaces::find_field(available, field.name);
        const auto range = available.ranges().at(source.value());
        for (std::size_t axis = 0; axis < range.tangent_size; ++axis) {
          route.covariance_indices.push_back(range.tangent_offset + axis);
        }
      }
      route.covariance.resize(matrix_size(required.tangent_size()));
      measurements_.push_back(std::move(route));
    }
    covariance_.resize(matrix_size(state_layout_.tangent_size()));
    router_.prepare_buffers();
    prepared_ = true;
  }

  std::uint8_t initialize(rpp::ListConst<double> state, rpp::ListConst<double> covariance,
    std::uint64_t time, bool activate = true)
  {
    namespace c = rpp_interfaces::contract;
    if (!prepared_) {prepare();}
    const auto& child = context_.get_subcomponent_context(estimator_slot_);
    if (child.is_active()) {return c::CODE_REJECTED;}
    if (child.get_instance_id() != instance_id_) {return c::CODE_NOT_READY;}
    const auto status = child.get_instance<rpp_interfaces::Estimator>()->initialize(state, covariance, time);
    if ((status != c::CODE_OK && status != c::CODE_LIMITED) || !activate) {return status;}
    return child.activate().success ? status : c::CODE_NOT_READY;
  }

  std::uint8_t reset(rpp::ListConst<double> state, rpp::ListConst<double> covariance,
    std::uint64_t time, bool activate = true)
  {
    namespace c = rpp_interfaces::contract;
    if (!prepared_) {return c::CODE_NOT_READY;}
    const auto& child = context_.get_subcomponent_context(estimator_slot_);
    if (child.get_instance_id() != instance_id_) {return c::CODE_NOT_READY;}
    const bool was_active = child.is_active();
    if (was_active && !child.deactivate().success) {return c::CODE_NOT_READY;}
    const auto status = child.get_instance<rpp_interfaces::Estimator>()->reset(state, covariance, time);
    if (status != c::CODE_OK && status != c::CODE_LIMITED) {
      if (was_active) {child.activate();}
      return status;
    }
    if (!activate) {return status;}
    return child.activate().success ? status : c::CODE_NOT_READY;
  }

  bool is_active() const
  {
    const auto result = context_.with_active_component<rpp_interfaces::Estimator>(
      estimator_slot_, selected_name_, [](auto&) {}, instance_id_);
    return prepared_ && result.first;
  }

  bool activate() const
  {
    const auto& child = context_.get_subcomponent_context(estimator_slot_);
    return prepared_ && child.get_instance_id() == instance_id_ && child.activate().success;
  }
  bool deactivate() const
  {
    const auto& child = context_.get_subcomponent_context(estimator_slot_);
    return prepared_ && child.get_instance_id() == instance_id_ &&
           (!child.is_active() || child.deactivate().success);
  }

  std::uint8_t predict(rpp::ListConst<double> source_input, rpp::ListConst<double> process_covariance,
    std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt)
  {
    namespace c = rpp_interfaces::contract;
    if (!prepared_) {return c::CODE_NOT_READY;}
    const auto result = context_.with_active_component<rpp_interfaces::Estimator>(estimator_slot_, selected_name_,
      [&](auto& estimator) {
        if (!router_.write(input_writer_, source_input)) {return c::CODE_INVALID_INPUT;}
        return estimator.predict(router_.read(input_reader_), process_covariance, time, dt, input_dt);
      }, instance_id_);
    return result.second.value_or(c::CODE_NOT_READY);
  }

  std::uint8_t correct(std::uint32_t index, rpp::ListConst<double> source_measurement,
    rpp::ListConst<double> source_covariance, std::uint64_t time)
  {
    namespace c = rpp_interfaces::contract;
    if (!prepared_) {return c::CODE_NOT_READY;}
    if (index >= measurements_.size()) {return c::CODE_INVALID_INPUT;}
    const auto result = context_.with_active_component<rpp_interfaces::Estimator>(estimator_slot_, selected_name_,
      [&](auto& estimator) {
        auto& route = measurements_[index];
        if (source_covariance.size() != route.source_width * route.source_width ||
          !router_.write(route.writer, source_measurement)) {return c::CODE_INVALID_INPUT;}
        const auto& indices = route.covariance_indices;
        for (std::size_t row = 0; row < indices.size(); ++row) {
          for (std::size_t column = 0; column < indices.size(); ++column) {
            route.covariance[row * indices.size() + column] =
              source_covariance[indices[row] * route.source_width + indices[column]];
          }
        }
        return estimator.correct(index, router_.read(route.reader), route.covariance, time);
      }, instance_id_);
    return result.second.value_or(c::CODE_NOT_READY);
  }

  // Routing buffers are borrowed until the next estimate()/prepare() call.
  // Retained public snapshots are available from the Estimator plugin itself.
  Estimate estimate()
  {
    namespace c = rpp_interfaces::contract;
    if (!prepared_) {return failure(c::CODE_NOT_READY);}
    const auto result = context_.with_active_component<rpp_interfaces::Estimator>(estimator_slot_, selected_name_,
      [&](auto& estimator) -> Estimate {
        auto [state, covariance, time, status] = estimator.estimate();
        if (status != c::CODE_OK && status != c::CODE_LIMITED) {return failure(status);}
        if (covariance.size() != covariance_.size() || !router_.write(state_writer_, state)) {
          return failure(c::CODE_INCOMPATIBLE);
        }
        for (std::size_t index = 0; index < covariance.size(); ++index) {covariance_[index] = covariance[index];}
        return {router_.read(state_reader_), covariance_, time, status};
      }, instance_id_);
    if (!result.second) {return failure(c::CODE_NOT_READY);}
    return *result.second;
  }

  const rpp_interfaces::FieldLayout& state_layout() const {return state_layout_;}
  const rpp_interfaces::FieldLayout& input_layout() const {return input_layout_;}
  const std::vector<rpp_interfaces::FieldLayout>& measurement_layouts() const {return measurement_layouts_;}

private:
  struct MeasurementRoute
  {
    Router::RouteId writer{0}, reader{0};
    std::size_t source_width{0};
    std::vector<std::size_t> covariance_indices;
    rpp::List<double> covariance;
  };
  static std::size_t matrix_size(std::size_t width)
  {
    if (width != 0 && width > Router::max_values / width) {
      throw std::invalid_argument("Covariance exceeds prepared routing capacity");
    }
    return width * width;
  }
  Estimate failure(std::uint8_t status) const {return {empty_, empty_, 0, status};}

  const std::string estimator_slot_{"estimator"}, selected_name_;
  const rpp::ComponentContext& context_;
  std::string instance_id_;
  Router router_;
  Router::SignalId state_signal_{0}, input_signal_{0};
  Router::RouteId state_writer_{0}, state_reader_{0}, input_writer_{0}, input_reader_{0};
  rpp_interfaces::FieldLayout state_layout_, input_layout_;
  std::vector<rpp_interfaces::FieldLayout> measurement_layouts_;
  std::vector<MeasurementRoute> measurements_;
  rpp::List<double> covariance_, empty_;
  bool prepared_{false};
};
}  // namespace rpp_localization
