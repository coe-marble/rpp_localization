#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/InertialModel.hpp>

#include <rpp_localization/inekf/inertial_process.hpp>

#include <kj/exception.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rpp_localization/inekf/inertial_payload.hpp>

// Strapdown IMU propagation as an InertialModel. The equations are in
// rpp_localization::InEKF::InertialProcess.
class ImuInertialModel final : public rpp_localization::InertialModel
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;

  // Noise values are standard deviations per square root of a second, one
  // per body axis.
  RPP_PARAMETERS(
    ParameterDescription::create<double>("gravity", 9.80665),
    ParameterDescription::create<std::vector<double>>(
      "gyro_noise", std::vector<double>(3, 1e-3)),
    ParameterDescription::create<std::vector<double>>(
      "accel_noise", std::vector<double>(3, 1e-2)),
    ParameterDescription::create<std::vector<double>>(
      "gyro_bias_noise", std::vector<double>(3, 1e-5)),
    ParameterDescription::create<std::vector<double>>(
      "accel_bias_noise", std::vector<double>(3, 1e-4))
  )

  ImuInertialModel() = default;
  ~ImuInertialModel() override = default;

  void initialize(const rpp::ComponentContext& context) override
  {
    process_ = std::make_unique<rpp_localization::InEKF::InertialProcess>(
      context.get_parameter<double>("gravity"));

    rpp_localization::InEKF::InertialProcessNoise noise;
    noise.gyro = read_axes(context, "gyro_noise");
    noise.accel = read_axes(context, "accel_noise");
    noise.gyro_bias = read_axes(context, "gyro_bias_noise");
    noise.accel_bias = read_axes(context, "accel_bias_noise");
    process_->set_noise(noise);
  }

  void reset() override {}

  InertialModelPredictOutput::Const predict(InertialModelPredictInput::Const input) override
  {
    namespace detail = rpp_localization::plugins::detail;
    InertialModelPredictOutput output;
    if (!process_) {
      set_status(output, detail::k_inertial_not_initialized, "inertial model is not initialized");
      return output;
    }
    if (input.deltaNs() < 0) {
      set_status(output, detail::k_inertial_invalid_time, "deltaNs must be non-negative");
      return output;
    }

    try {
      auto state = detail::read_inertial_state(input.state());
      rpp_localization::InEKF::InertialPredictionInput step;
      step.reference_time = input.referenceTimeNs();
      step.delta = input.deltaNs();
      step.imu = detail::read_imu(input.imu());
      process_->predict(state, step);

      if (!state().allFinite() || !state.cov().allFinite()) {
        set_status(
          output, detail::k_inertial_numerical_failure,
          "inertial prediction produced an invalid state");
        return output;
      }
      detail::write_inertial_state(output.state(), state);
      set_status(output, detail::k_inertial_ok, "");
    } catch (const detail::InertialPayloadError& error) {
      set_status(output, error.code(), error.what());
    } catch (const std::exception& error) {
      set_status(output, detail::k_inertial_model_failure, error.what());
    }
    return output;
  }

private:
  static Eigen::Vector3d read_axes(const rpp::ComponentContext& context, const std::string& name)
  {
    const auto values = context.get_parameter<std::vector<double>>(name);
    if (values.size() != 3) {
      throw std::invalid_argument(name + " must contain 3 values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  static void set_status(
    InertialModelPredictOutput& output,
    const std::uint16_t code,
    std::string message)
  {
    auto status = output.status();
    status.code() = code;
    status.message() = std::move(message);
  }

  std::unique_ptr<rpp_localization::InEKF::InertialProcess> process_;
};
