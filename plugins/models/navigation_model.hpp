#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/Model.hpp>
#include <rpp_schema/rpp_interfaces/ModelDesc.hpp>
#include <capnp_gen/rpp_interfaces/model/constants.capnp.h>

#include <rpp_localization/estimation/rpp_support.hpp>
#include <rpp_localization/models/native_navigation15.hpp>

#include <tuple>

// Implements the generated shared Model interface. The generated server/client
// adapters provide Cap'n Proto transport for these same methods.
class NavigationModel final : public rpp_interfaces::Model,
  public rpp_localization::estimation::NativeStateObservation
{
  using Vector = rpp_localization::estimation::Vector;
  using Matrix = rpp_localization::estimation::Matrix;
  using Pool = rpp_localization::estimation::shared::OutputPool;
  using Description = rpp_schema::rpp_interfaces::ModelDesc;
  static constexpr Eigen::Index n = rpp_localization::STATE_SIZE;
public:
  using Result = std::tuple<rpp::ListConst<double>, std::uint8_t>;
  using Derivatives = std::tuple<rpp::ListConst<double>, rpp::ListConst<double>, std::uint8_t>;
  using ParameterDescription = rpp::params::ParameterDescription;
  std::vector<Eigen::Index> observation_state_indices() const override
  {
    std::vector<Eigen::Index> indices(static_cast<std::size_t>(n));
    for (Eigen::Index index = 0; index < n; ++index) {indices[static_cast<std::size_t>(index)] = index;}
    return indices;
  }
  RPP_PARAMETERS(
    ParameterDescription::create<std::string>("world_frame", "map"),
    ParameterDescription::create<std::string>("body_frame", "base_link"),
    ParameterDescription::create<std::string>("resource", "base"),
    ParameterDescription::create<bool>("prepared_control", false)
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext& context) override
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate the model before configuration");}
    configured_ = false;
    const auto world = context.get_parameter<std::string>("world_frame");
    const auto body = context.get_parameter<std::string>("body_frame");
    const auto resource = context.get_parameter<std::string>("resource");
    if (world.empty() || body.empty() || resource.empty()) {
      return rpp::CallbackResult::failure("Model frames and resource must be nonempty");
    }
    prepared_control_ = context.get_parameter<bool>("prepared_control");
    model_ = std::make_unique<rpp_localization::NativeNavigationModel15>(prepared_control_);
    namespace c = rpp_interfaces::contract;
    const std::array<const char*, 15> names{
      "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
      "vroll", "vpitch", "vyaw", "ax", "ay", "az"};
    std::vector<rpp_interfaces::Field> fields;
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto quantity = index < 3 ? c::QUANTITY_LINEAR_POSITION :
        index < 6 ? c::QUANTITY_ANGULAR_POSITION : index < 9 ? c::QUANTITY_LINEAR_VELOCITY :
        index < 12 ? c::QUANTITY_ANGULAR_VELOCITY : c::QUANTITY_LINEAR_ACCELERATION;
      fields.push_back({resource + "." + names[index],
        index >= 3 && index < 6 ? c::COORDINATE_CONTINUOUS_ANGLE : c::COORDINATE_EUCLIDEAN,
        quantity, 1, index < 6 ? world : body, body, resource});
    }
    state_layout_ = rpp_interfaces::FieldLayout(std::move(fields));
    fields.clear();
    if (prepared_control_) {
      const std::array<const char*, 9> inputs{
        "control.ax", "control.ay", "control.az", "control.aroll", "control.apitch",
        "control.ayaw", "control.overrideAx", "control.overrideAy", "control.overrideAz"};
      for (std::size_t index = 0; index < inputs.size(); ++index) {
        fields.push_back({resource + "." + inputs[index], c::COORDINATE_EUCLIDEAN,
          index < 3 ? c::QUANTITY_LINEAR_ACCELERATION :
          index < 6 ? c::QUANTITY_ANGULAR_ACCELERATION : c::QUANTITY_NORMALIZED,
          1, body, body, resource});
      }
    }
    input_layout_ = rpp_interfaces::FieldLayout(std::move(fields));
    const auto inputs = model_->input_size();
    state_work_.resize(n); input_work_.resize(inputs); delta_.resize(n); value_.resize(n);
    a_.resize(n, n); b_.resize(n, inputs);
    values_.prepare(n);
    derivatives_.prepare(n * n, static_cast<std::size_t>(n * inputs));
    configured_ = true;
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the model first");}
    Description result;
    auto state = result.state(); state_layout_.write(state);
    auto input = result.inputs(); input_layout_.write(input);
    auto prediction = result.prediction(); state_layout_.write(prediction);
    auto observation = result.observations(); state_layout_.write(observation);
    namespace c = rpp_interfaces::contract;
    result.features() = c::MODEL_PREDICT | c::MODEL_LINEARIZE | c::MODEL_OBSERVE |
      c::MODEL_OBSERVATION_JACOBIAN;
    return Description::Const(std::move(result));
  }

  Result predict(rpp::ListConst<double> state, rpp::ListConst<double> input,
    std::uint64_t time, std::uint64_t dt, std::uint64_t input_dt) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return failure(c::CODE_NOT_READY);}
    if (dt > std::numeric_limits<std::uint64_t>::max() - time || !inputs(state, input)) {
      return failure(c::CODE_INVALID_INPUT);
    }
    const auto slot = values_.acquire();
    if (slot == Pool::unavailable) {return failure(c::CODE_BUDGET_EXCEEDED);}
    if (!model_->predict(state_work_, input_work_, time, dt, input_dt, value_)) {
      return failure(c::CODE_INFEASIBLE);
    }
    return value_result(slot);
  }

  Derivatives linearize(rpp::ListConst<double> state, rpp::ListConst<double> input,
    std::uint64_t time, std::uint64_t dt, std::uint64_t) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return derivative_failure(c::CODE_NOT_READY);}
    if (dt > std::numeric_limits<std::uint64_t>::max() - time || !inputs(state, input)) {
      return derivative_failure(c::CODE_INVALID_INPUT);
    }
    const auto slot = derivatives_.acquire();
    if (slot == Pool::unavailable) {return derivative_failure(c::CODE_BUDGET_EXCEEDED);}
    // Enable reuse only after a caller actually requests Jacobians. Pure UKF
    // prediction keeps the state-only model path without cache-copy overhead.
    model_->enable_evaluation_cache();
    if (!model_->linearize_full(state_work_, input_work_, dt, a_, b_)) {
      return derivative_failure(c::CODE_INFEASIBLE);
    }
    return derivative_result(slot);
  }

  Result retract(rpp::ListConst<double> state, rpp::ListConst<double> delta) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return failure(c::CODE_NOT_READY);}
    if (!rpp_localization::estimation::shared::read(state, state_work_) ||
      !rpp_localization::estimation::shared::read(delta, delta_)) {return failure(c::CODE_INVALID_INPUT);}
    const auto slot = values_.acquire();
    if (slot == Pool::unavailable) {return failure(c::CODE_BUDGET_EXCEEDED);}
    model_->state_space().retract(state_work_, delta_, value_);
    return value_result(slot);
  }

  Result difference(rpp::ListConst<double> origin, rpp::ListConst<double> target) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return failure(c::CODE_NOT_READY);}
    if (!rpp_localization::estimation::shared::read(origin, state_work_) ||
      !rpp_localization::estimation::shared::read(target, delta_)) {return failure(c::CODE_INVALID_INPUT);}
    const auto slot = values_.acquire();
    if (slot == Pool::unavailable) {return failure(c::CODE_BUDGET_EXCEEDED);}
    model_->state_space().difference(state_work_, delta_, value_);
    return value_result(slot);
  }

  Result observe(rpp::ListConst<double> state, rpp::ListConst<double> input,
    std::uint64_t, std::uint64_t) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return failure(c::CODE_NOT_READY);}
    if (!inputs(state, input)) {return failure(c::CODE_INVALID_INPUT);}
    const auto slot = values_.acquire();
    if (slot == Pool::unavailable) {return failure(c::CODE_BUDGET_EXCEEDED);}
    value_ = state_work_;
    return value_result(slot);
  }

  Derivatives observationJacobian(rpp::ListConst<double> state, rpp::ListConst<double> input,
    std::uint64_t, std::uint64_t) override
  {
    namespace c = rpp_interfaces::contract;
    if (!active_) {return derivative_failure(c::CODE_NOT_READY);}
    if (!inputs(state, input)) {return derivative_failure(c::CODE_INVALID_INPUT);}
    const auto slot = derivatives_.acquire();
    if (slot == Pool::unavailable) {return derivative_failure(c::CODE_BUDGET_EXCEEDED);}
    a_.setIdentity(); b_.setZero();
    return derivative_result(slot);
  }

  Result observationDifference(rpp::ListConst<double> origin, rpp::ListConst<double> target) override
  {return difference(origin, target);}

  Result inverseDynamics(rpp::ListConst<double>, rpp::ListConst<double>) override
  {return failure(rpp_interfaces::contract::CODE_UNSUPPORTED);}

  rpp::CallbackResult on_activate() override
  {
    if (!configured_) {return rpp::CallbackResult::failure("Configure the model first");}
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = false;
    values_.clear(); derivatives_.clear(); model_.reset();
    state_layout_ = {}; input_layout_ = {};
    return {};
  }

private:
  bool inputs(rpp::ListConst<double> state, rpp::ListConst<double> input)
  {
    if (!rpp_localization::estimation::shared::read(state, state_work_) ||
      !rpp_localization::estimation::shared::read(input, input_work_)) {return false;}
    if (prepared_control_) {
      for (Eigen::Index flag = 6; flag < 9; ++flag) {
        if (input_work_(flag) != 0.0 && input_work_(flag) != 1.0) {return false;}
      }
    }
    return true;
  }
  Result failure(std::uint8_t code) const {return {values_.empty(), code};}
  Derivatives derivative_failure(std::uint8_t code) const
  {return {derivatives_.empty(), derivatives_.empty(), code};}
  Result value_result(std::size_t slot)
  {
    if (!value_.allFinite()) {return failure(rpp_interfaces::contract::CODE_INFEASIBLE);}
    rpp_localization::estimation::shared::write_vector(value_, values_.first(slot));
    return {values_.first_view(slot), rpp_interfaces::contract::CODE_OK};
  }
  Derivatives derivative_result(std::size_t slot)
  {
    if (!a_.allFinite() || !b_.allFinite()) {
      return derivative_failure(rpp_interfaces::contract::CODE_INFEASIBLE);
    }
    rpp_localization::estimation::shared::write_matrix(a_, derivatives_.first(slot));
    rpp_localization::estimation::shared::write_matrix(b_, derivatives_.second(slot));
    return {derivatives_.first_view(slot), derivatives_.second_view(slot), rpp_interfaces::contract::CODE_OK};
  }

  std::unique_ptr<rpp_localization::NativeNavigationModel15> model_;
  rpp_interfaces::FieldLayout state_layout_, input_layout_;
  Vector state_work_, input_work_, delta_, value_;
  Matrix a_, b_;
  Pool values_, derivatives_;
  bool prepared_control_{false}, configured_{false}, active_{false};
};
