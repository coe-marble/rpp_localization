#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/NavModel15.hpp>

#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/models/constant_acceleration_model.hpp>

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// RPP entry point for the legacy 15-state constant-acceleration prediction
// model. The core equations remain in rpp_localization::ConstantAccelerationModel.
class ConstantAccelerationModel15 final : public rpp_localization::NavModel15
{
public:
  static constexpr std::size_t k_state_size = rpp_localization::STATE_SIZE;
  static constexpr std::size_t k_control_size = rpp_localization::TWIST_SIZE;
  static constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

  using ParameterDescription = rpp::params::ParameterDescription;
  RPP_PARAMETERS(
    ParameterDescription::create<std::vector<double>>(
      "process_noise_covariance", std::vector<double>{}),
    ParameterDescription::create<bool>("dynamic_process_noise_covariance", false),
    ParameterDescription::create<bool>("use_control", false),
    ParameterDescription::create<double>("control_timeout_seconds", 0.0),
    ParameterDescription::create<std::vector<bool>>(
      "control_config", std::vector<bool>(k_control_size, false)),
    ParameterDescription::create<std::vector<double>>(
      "acceleration_limits", std::vector<double>(k_control_size, 1.0)),
    ParameterDescription::create<std::vector<double>>(
      "acceleration_gains", std::vector<double>(k_control_size, 1.0)),
    ParameterDescription::create<std::vector<double>>(
      "deceleration_limits", std::vector<double>(k_control_size, 1.0)),
    ParameterDescription::create<std::vector<double>>(
      "deceleration_gains", std::vector<double>(k_control_size, 1.0))
  )

  ConstantAccelerationModel15() = default;
  ~ConstantAccelerationModel15() override = default;

  void initialize(const rpp::ComponentContext& context) override
  {
    ready_ = false;
    configured_ = false;
    model_ = std::make_unique<rpp_localization::ConstantAccelerationModel>(k_state_size);
    model_->initialize_runtime();

    load_configuration(context);
    apply_configuration();
    configured_ = true;
    ready_ = true;
  }

  void reset() override
  {
    if (!configured_ || !model_)
    {
      ready_ = false;
      return;
    }

    model_->reset();
    model_->initialize_runtime();
    apply_configuration();
    ready_ = true;
  }

  NavModelDescription15::Const describe() override
  {
    // Control is a target velocity that the model turns into an acceleration.
    NavModelDescription15 description;
    description.controlDrivesAcceleration() = true;
    return description;
  }

  LocalizationModelPredictOutput15::Const predict(LocalizationModelPredictInput15::Const input) override
  {
    LocalizationModelPredictOutput15 output;
    write_zero_output(output);

    if (!ready_ || !model_)
    {
      set_status(output, k_not_initialized, "model is not initialized");
      return output;
    }

    if (input.deltaNs() < 0)
    {
      set_status(output, k_invalid_time, "deltaNs must be non-negative");
      return output;
    }

    const auto input_state = input.state();
    const auto state_values = input_state.values();
    if (!has_expected_finite_values(state_values, k_state_size))
    {
      set_status(output, k_invalid_state, "state must contain 15 finite values");
      return output;
    }

    const auto input_covariance = input.covariance();
    const auto covariance_values = input_covariance.values();
    if (!has_valid_covariance(covariance_values))
    {
      set_status(output, k_invalid_covariance,
        "covariance must contain 225 finite values with non-negative diagonal");
      return output;
    }

    const auto control = input.control();
    if (!has_valid_control(control))
    {
      set_status(output, k_invalid_payload,
        "present control must contain six enabled flags and six finite enabled values");
      return output;
    }

    rpp_localization::StateVector state(k_state_size);
    rpp_localization::CovarianceMatrix covariance(k_state_size, k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      state(static_cast<Eigen::Index>(index)) = state_values[index];
    }
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          covariance_values[row * k_state_size + column];
      }
    }

    write_output(output, state, covariance);
    const auto original_state = state;
    const auto original_covariance = covariance;

    if (!configure_control(control, state, input.referenceTimeNs()))
    {
      set_status(output, k_invalid_time, "control timestamp arithmetic overflows");
      return output;
    }

    try
    {
      model_->set_state(state);
      model_->predict(state, covariance, input.referenceTimeNs(), input.deltaNs());
    }
    catch (const std::exception& error)
    {
      write_output(output, original_state, original_covariance);
      set_status(output, k_model_failure, error.what());
      return output;
    }

    if (!state.allFinite() || !covariance.allFinite())
    {
      write_output(output, original_state, original_covariance);
      set_status(output, k_numerical_failure, "prediction produced a non-finite value");
      return output;
    }

    write_output(output, state, covariance);
    set_status(output, k_ok, "");
    return output;
  }

private:
  static constexpr std::uint16_t k_ok = 0;
  static constexpr std::uint16_t k_invalid_payload = 1;
  static constexpr std::uint16_t k_invalid_time = 2;
  static constexpr std::uint16_t k_invalid_state = 3;
  static constexpr std::uint16_t k_invalid_covariance = 4;
  static constexpr std::uint16_t k_not_initialized = 5;
  static constexpr std::uint16_t k_model_failure = 7;
  static constexpr std::uint16_t k_numerical_failure = 9;

  template<typename Values>
  static bool has_expected_finite_values(const Values& values, const std::size_t expected_size)
  {
    if (values.size() != expected_size)
    {
      return false;
    }

    for (std::size_t index = 0; index < expected_size; ++index)
    {
      if (!std::isfinite(values[index]))
      {
        return false;
      }
    }
    return true;
  }

  template<typename Values>
  static bool has_valid_covariance(const Values& values)
  {
    if (!has_expected_finite_values(values, k_covariance_size))
    {
      return false;
    }

    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      if (values[index * k_state_size + index] < 0.0)
      {
        return false;
      }
    }
    return true;
  }

  template<typename Control>
  static bool has_valid_control(const Control& control)
  {
    if (!control.present())
    {
      return true;
    }

    const auto values = control.values();
    const auto enabled = control.enabled();
    if (values.size() != k_control_size || enabled.size() != k_control_size)
    {
      return false;
    }

    for (std::size_t index = 0; index < k_control_size; ++index)
    {
      if (enabled[index] && !std::isfinite(values[index]))
      {
        return false;
      }
    }
    return true;
  }

  static bool can_subtract(
    const rpp_localization::TimestampNs left,
    const rpp_localization::TimestampNs right)
  {
    if (right > 0)
    {
      return left >= std::numeric_limits<rpp_localization::TimestampNs>::min() + right;
    }
    if (right < 0)
    {
      return left <= std::numeric_limits<rpp_localization::TimestampNs>::max() + right;
    }
    return true;
  }

  template<typename Control>
  bool configure_control(
    const Control& control,
    const rpp_localization::StateVector& state,
    const rpp_localization::TimestampNs reference_time)
  {
    if (!use_control_)
    {
      return true;
    }

    rpp_localization::ControlCommand legacy_control;
    legacy_control.control = rpp_localization::ControlVector::Zero(k_control_size);

    if (!control.present())
    {
      if (!can_subtract(reference_time, control_timeout_ns_))
      {
        return false;
      }
      legacy_control.stamp = reference_time - control_timeout_ns_;
    }
    else
    {
      if (!can_subtract(reference_time, control.stampNs()))
      {
        return false;
      }

      const auto values = control.values();
      const auto enabled = control.enabled();
      legacy_control.stamp = control.stampNs();
      for (std::size_t index = 0; index < k_control_size; ++index)
      {
        legacy_control.control(static_cast<Eigen::Index>(index)) = enabled[index] ?
          values[index] :
          state(rpp_localization::POSITION_V_OFFSET + static_cast<Eigen::Index>(index));
      }
    }

    model_->set_control(legacy_control);
    return true;
  }

  void load_configuration(const rpp::ComponentContext& context)
  {
    const auto process_noise_values =
      context.get_parameter<std::vector<double>>("process_noise_covariance");
    if (process_noise_values.empty())
    {
      process_noise_covariance_ = model_->get_process_noise_covariance();
    }
    else
    {
      if (!has_valid_covariance(process_noise_values))
      {
        throw std::invalid_argument(
          "process_noise_covariance must contain 225 finite values with a non-negative diagonal");
      }

      process_noise_covariance_.resize(k_state_size, k_state_size);
      for (std::size_t row = 0; row < k_state_size; ++row)
      {
        for (std::size_t column = 0; column < k_state_size; ++column)
        {
          process_noise_covariance_(static_cast<Eigen::Index>(row),
            static_cast<Eigen::Index>(column)) =
            process_noise_values[row * k_state_size + column];
        }
      }
    }

    dynamic_process_noise_covariance_ =
      context.get_parameter<bool>("dynamic_process_noise_covariance");
    use_control_ = context.get_parameter<bool>("use_control");
    control_timeout_ns_ = duration_from_seconds(
      context.get_parameter<double>("control_timeout_seconds"));

    control_update_vector_ = context.get_parameter<std::vector<bool>>("control_config");
    acceleration_limits_ = context.get_parameter<std::vector<double>>("acceleration_limits");
    acceleration_gains_ = context.get_parameter<std::vector<double>>("acceleration_gains");
    deceleration_limits_ = context.get_parameter<std::vector<double>>("deceleration_limits");
    deceleration_gains_ = context.get_parameter<std::vector<double>>("deceleration_gains");

    if (!use_control_)
    {
      return;
    }

    if (control_update_vector_.size() != k_control_size ||
      !has_expected_finite_values(acceleration_limits_, k_control_size) ||
      !has_expected_finite_values(acceleration_gains_, k_control_size) ||
      !has_expected_finite_values(deceleration_limits_, k_control_size) ||
      !has_expected_finite_values(deceleration_gains_, k_control_size))
    {
      throw std::invalid_argument("control parameters must each contain six values");
    }

    for (std::size_t index = 0; index < k_control_size; ++index)
    {
      if (acceleration_limits_[index] < 0.0 || acceleration_gains_[index] < 0.0 ||
        deceleration_limits_[index] < 0.0 || deceleration_gains_[index] < 0.0)
      {
        throw std::invalid_argument("control limits and gains must be non-negative");
      }
    }
  }

  void apply_configuration()
  {
    model_->set_process_noise_covariance(process_noise_covariance_);
    model_->set_dynamic_process_noise_covariance(dynamic_process_noise_covariance_);

    if (use_control_)
    {
      model_->set_control_params(
        control_update_vector_,
        control_timeout_ns_,
        acceleration_limits_,
        acceleration_gains_,
        deceleration_limits_,
        deceleration_gains_);
    }
  }

  static rpp_localization::DurationNs duration_from_seconds(const double seconds)
  {
    constexpr double kNanosecondsPerSecond = 1e9;
    const auto max_seconds =
      static_cast<double>(std::numeric_limits<rpp_localization::DurationNs>::max()) /
      kNanosecondsPerSecond;
    if (!std::isfinite(seconds) || seconds < 0.0 || seconds > max_seconds)
    {
      throw std::invalid_argument("control_timeout_seconds must be finite and non-negative");
    }
    return static_cast<rpp_localization::DurationNs>(seconds * kNanosecondsPerSecond);
  }

  static void write_output(
    LocalizationModelPredictOutput15& output,
    const rpp_localization::StateVector& state,
    const rpp_localization::CovarianceMatrix& covariance)
  {
    auto output_state = output.state();
    auto state_values = output_state.values();
    state_values.resize(k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      state_values[index] = state(static_cast<Eigen::Index>(index));
    }

    auto output_covariance = output.covariance();
    auto covariance_values = output_covariance.values();
    covariance_values.resize(k_covariance_size);
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        covariance_values[row * k_state_size + column] =
          covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
      }
    }
  }

  static void write_zero_output(LocalizationModelPredictOutput15& output)
  {
    write_output(
      output,
      rpp_localization::StateVector::Zero(k_state_size),
      rpp_localization::CovarianceMatrix::Zero(k_state_size, k_state_size));
  }

  static void set_status(
    LocalizationModelPredictOutput15& output,
    const std::uint16_t code,
    std::string message)
  {
    auto status = output.status();
    status.code() = code;
    status.message() = std::move(message);
  }

  std::unique_ptr<rpp_localization::ConstantAccelerationModel> model_;
  rpp_localization::CovarianceMatrix process_noise_covariance_;
  rpp_localization::DurationNs control_timeout_ns_{0};
  std::vector<bool> control_update_vector_;
  std::vector<double> acceleration_limits_;
  std::vector<double> acceleration_gains_;
  std::vector<double> deceleration_limits_;
  std::vector<double> deceleration_gains_;
  bool dynamic_process_noise_covariance_{false};
  bool use_control_{false};
  bool configured_{false};
  bool ready_{false};
};
