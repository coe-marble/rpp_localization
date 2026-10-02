#pragma once

#include <rpp_plugin_types/rpp_localization/NavModel15.hpp>

#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/core/model_base.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rpp_localization::plugins::detail
{

class ModelPredictionError final : public std::runtime_error
{
public:
  ModelPredictionError(const std::uint16_t code, std::string message)
  : std::runtime_error(std::move(message)),
    code_(code)
  {
  }

  [[nodiscard]] std::uint16_t code() const noexcept
  {
    return code_;
  }

private:
  std::uint16_t code_;
};

struct ModelControl
{
  bool present{false};
  TimestampNs stamp{0};
  std::vector<double> values = std::vector<double>(TWIST_SIZE, 0.0);
  std::vector<bool> enabled = std::vector<bool>(TWIST_SIZE, false);
};

// Adapts the RPP state-in/state-out prediction call to the runtime model interface
// used by Ekf and Ukf. The RPP filter composition root owns this adapter.
class NavModel15Adapter final : public ModelBase
{
public:
  static constexpr std::size_t k_state_size = STATE_SIZE;
  static constexpr std::size_t k_control_size = TWIST_SIZE;
  static constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

  explicit NavModel15Adapter(std::shared_ptr<NavModel15> model)
  : ModelBase(STATE_SIZE),
    model_(std::move(model))
  {
    if (!model_)
    {
      throw std::invalid_argument("NavModel15 component is required");
    }
  }

  void reset_runtime()
  {
    _state.setZero();
    _state_covariance.setZero();
    control_ = ModelControl{};
  }

  void set_control(const ModelControl& control)
  {
    control_ = control;
  }

  template<typename Control>
  void set_control(const Control& control)
  {
    ModelControl input;
    input.present = control.present();
    input.stamp = control.stampNs();
    if (!input.present)
    {
      set_control(input);
      return;
    }

    const auto values = control.values();
    const auto enabled = control.enabled();
    for (std::size_t index = 0; index < k_control_size; ++index)
    {
      input.values[index] = values[index];
      input.enabled[index] = enabled[index];
    }
    set_control(input);
  }

  [[nodiscard]] const ModelControl& get_control_input() const
  {
    return control_;
  }

  void predict(
    StateVector& state,
    CovarianceMatrix& covariance,
    const TimestampNs reference_time,
    const DurationNs delta) override
  {
    NavModel15::LocalizationModelPredictInput15 input;
    write_state(input.state(), state);
    write_covariance(input.covariance(), covariance);
    write_control(input.control());
    input.referenceTimeNs() = reference_time;
    input.deltaNs() = delta;

    const auto output = model_->predict(std::move(input));
    const auto status = output.status();
    if (status.code() != k_ok)
    {
      throw ModelPredictionError(status.code(), status.message());
    }

    const auto state_values = output.state().values();
    const auto covariance_values = output.covariance().values();
    if (!has_finite_values(state_values, k_state_size))
    {
      throw ModelPredictionError(k_numerical_failure, "NavModel15 returned an invalid state");
    }
    if (!has_valid_state_covariance(covariance_values))
    {
      throw ModelPredictionError(k_numerical_failure, "NavModel15 returned an invalid covariance");
    }

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
  }

  template<typename Values>
  static bool has_finite_values(const Values& values, const std::size_t expected_size)
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
  static bool has_valid_state_covariance(const Values& values)
  {
    if (!has_finite_values(values, k_covariance_size))
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

private:
  static constexpr std::uint16_t k_ok = 0;
  static constexpr std::uint16_t k_numerical_failure = 9;

  template<typename State>
  static void write_state(State output, const StateVector& state)
  {
    auto values = output.values();
    values.resize(k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      values[index] = state(static_cast<Eigen::Index>(index));
    }
  }

  template<typename Covariance>
  static void write_covariance(Covariance output, const CovarianceMatrix& covariance)
  {
    auto values = output.values();
    values.resize(k_covariance_size);
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        values[row * k_state_size + column] =
          covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
      }
    }
  }

  template<typename Control>
  void write_control(Control output) const
  {
    output.present() = control_.present;
    output.stampNs() = control_.stamp;
    auto values = output.values();
    auto enabled = output.enabled();
    values.resize(k_control_size);
    enabled.resize(k_control_size);
    for (std::size_t index = 0; index < k_control_size; ++index)
    {
      values[index] = control_.values[index];
      enabled[index] = control_.enabled[index];
    }
  }

  std::shared_ptr<NavModel15> model_;
  ModelControl control_;
};

}  // namespace rpp_localization::plugins::detail
