#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/more_dynamics/VehicleModel3D.hpp>
#include <rpp_plugin_types/rpp_localization/NavModel15.hpp>

#include <rpp_localization/core/filter_common.hpp>

#include <kj/exception.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Bridges a numerically propagating VehicleModel3D to the legacy state-15
// navigation-model contract. Command layout is explicit because VehicleModel3D
// deliberately permits arbitrary actuator command vectors.
class VehicleModel3DNavModel15 final : public rpp_localization::NavModel15
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;

  static constexpr std::size_t k_state_size = rpp_localization::STATE_SIZE;
  static constexpr std::size_t k_control_size = rpp_localization::TWIST_SIZE;
  static constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

  RPP_COMPONENTS(
    {"vehicle_model", "more_dynamics::VehicleModel3D"}
  )

  RPP_PARAMETERS(
    ParameterDescription::create<std::vector<std::int64_t>>(
      "command_sizes", std::vector<std::int64_t>{1, 1, 1, 1, 1, 1}),
    ParameterDescription::create<std::vector<std::int64_t>>(
      "control_command_indices", std::vector<std::int64_t>{0, 1, 2, 3, 4, 5}),
    ParameterDescription::create<std::vector<std::int64_t>>(
      "control_data_indices", std::vector<std::int64_t>{0, 0, 0, 0, 0, 0}),
    ParameterDescription::create<std::vector<double>>(
      "control_scales", std::vector<double>(k_control_size, 1.0)),
    ParameterDescription::create<std::vector<double>>(
      "control_offsets", std::vector<double>(k_control_size, 0.0)),
    ParameterDescription::create<std::vector<double>>(
      "process_noise_covariance", std::vector<double>{}),
    ParameterDescription::create<double>("jacobian_epsilon", 1e-6)
  )

  VehicleModel3DNavModel15() = default;
  ~VehicleModel3DNavModel15() override = default;

  void initialize(const rpp::ComponentContext& context) override
  {
    ready_ = false;
    load_configuration(context);
    vehicle_model_ = context.get_component<more_dynamics::VehicleModel3D>("vehicle_model");
    if (!vehicle_model_)
    {
      throw std::invalid_argument("VehicleModel3D component is required");
    }
    ready_ = true;
  }

  void reset() override
  {
    ready_ = vehicle_model_ != nullptr;
  }

  LocalizationModelPredictOutput15::Const predict(LocalizationModelPredictInput15::Const input) override
  {
    LocalizationModelPredictOutput15 output;
    write_zero_output(output);
    if (!ready_ || !vehicle_model_)
    {
      set_status(output, k_not_initialized, "VehicleModel3D bridge is not initialized");
      return output;
    }
    if (input.deltaNs() < 0)
    {
      set_status(output, k_invalid_time, "deltaNs must be non-negative");
      return output;
    }

    rpp_localization::StateVector state(k_state_size);
    rpp_localization::CovarianceMatrix covariance(k_state_size, k_state_size);
    const auto state_validation = read_state_and_covariance(input, state, covariance);
    if (state_validation.code != k_ok)
    {
      set_status(output, state_validation.code, state_validation.message);
      return output;
    }

    const auto control = input.control();
    if (!has_valid_control(control))
    {
      set_status(output, k_invalid_payload,
        "present control must contain six enabled flags and six finite enabled values");
      return output;
    }

    write_output(output, state, covariance);
    try
    {
      const auto propagated = propagate_state(
        state, control, input.referenceTimeNs(), input.deltaNs());
      const auto transition = numerical_transition(
        state, propagated, control, input.referenceTimeNs(), input.deltaNs());
      rpp_localization::CovarianceMatrix propagated_covariance =
        transition * covariance * transition.transpose();
      propagated_covariance.noalias() +=
        rpp_localization::nanoseconds_to_seconds(input.deltaNs()) * process_noise_covariance_;

      if (!propagated.allFinite() || !propagated_covariance.allFinite() ||
        !has_non_negative_diagonal(propagated_covariance))
      {
        set_status(output, k_numerical_failure, "VehicleModel3D bridge produced an invalid estimate");
        return output;
      }

      write_output(output, propagated, propagated_covariance);
      set_status(output, k_ok, "");
      return output;
    }
    catch (const std::exception& error)
    {
      set_status(output, k_model_failure, error.what());
      return output;
    }
    catch (const kj::Exception& error)
    {
      // A vehicle model in another process reports its failure this way.
      set_status(output, k_model_failure, error.getDescription().cStr());
      return output;
    }
  }

private:
  struct ValidationResult
  {
    std::uint16_t code;
    std::string message;
  };

  static constexpr std::uint16_t k_ok = 0;
  static constexpr std::uint16_t k_invalid_payload = 1;
  static constexpr std::uint16_t k_invalid_time = 2;
  static constexpr std::uint16_t k_invalid_state = 3;
  static constexpr std::uint16_t k_invalid_covariance = 4;
  static constexpr std::uint16_t k_not_initialized = 5;
  static constexpr std::uint16_t k_model_failure = 7;
  static constexpr std::uint16_t k_numerical_failure = 9;

  void load_configuration(const rpp::ComponentContext& context)
  {
    command_sizes_ = context.get_parameter<std::vector<std::int64_t>>("command_sizes");
    control_command_indices_ =
      context.get_parameter<std::vector<std::int64_t>>("control_command_indices");
    control_data_indices_ =
      context.get_parameter<std::vector<std::int64_t>>("control_data_indices");
    control_scales_ = context.get_parameter<std::vector<double>>("control_scales");
    control_offsets_ = context.get_parameter<std::vector<double>>("control_offsets");
    jacobian_epsilon_ = context.get_parameter<double>("jacobian_epsilon");
    const auto process_noise_values =
      context.get_parameter<std::vector<double>>("process_noise_covariance");

    if (command_sizes_.empty() || control_command_indices_.size() != k_control_size ||
      control_data_indices_.size() != k_control_size || control_scales_.size() != k_control_size ||
      control_offsets_.size() != k_control_size || !std::isfinite(jacobian_epsilon_) ||
      jacobian_epsilon_ <= 0.0)
    {
      throw std::invalid_argument("VehicleModel3D bridge command mapping is invalid");
    }

    for (const auto command_size : command_sizes_)
    {
      if (command_size <= 0)
      {
        throw std::invalid_argument("VehicleModel3D command_sizes must be positive");
      }
    }
    for (std::size_t axis = 0; axis < k_control_size; ++axis)
    {
      const auto command_index = control_command_indices_[axis];
      const auto data_index = control_data_indices_[axis];
      if (command_index < 0 || data_index < 0 ||
        command_index >= static_cast<std::int64_t>(command_sizes_.size()) ||
        data_index >= command_sizes_[static_cast<std::size_t>(command_index)] ||
        !std::isfinite(control_scales_[axis]) || !std::isfinite(control_offsets_[axis]))
      {
        throw std::invalid_argument("VehicleModel3D control mapping is out of range or non-finite");
      }
      for (std::size_t other = 0; other < axis; ++other)
      {
        if (command_index == control_command_indices_[other] &&
          data_index == control_data_indices_[other])
        {
          throw std::invalid_argument("VehicleModel3D control mapping targets must be unique");
        }
      }
    }

    process_noise_covariance_.setZero(k_state_size, k_state_size);
    if (process_noise_values.empty())
    {
      return;
    }
    if (process_noise_values.size() != k_covariance_size)
    {
      throw std::invalid_argument(
              "VehicleModel3D process_noise_covariance must contain 225 values");
    }
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        const double value = process_noise_values[row * k_state_size + column];
        if (!std::isfinite(value))
        {
          throw std::invalid_argument("VehicleModel3D process_noise_covariance must be finite");
        }
        process_noise_covariance_(static_cast<Eigen::Index>(row),
          static_cast<Eigen::Index>(column)) = value;
      }
    }
    if (!has_non_negative_diagonal(process_noise_covariance_))
    {
      throw std::invalid_argument(
              "VehicleModel3D process_noise_covariance must have a non-negative diagonal");
    }
  }

  template<typename Input>
  static ValidationResult read_state_and_covariance(
    const Input& input,
    rpp_localization::StateVector& state,
    rpp_localization::CovarianceMatrix& covariance)
  {
    const auto state_values = input.state().values();
    const auto covariance_values = input.covariance().values();
    if (!has_finite_values(state_values, k_state_size))
    {
      return {k_invalid_state, "state must contain 15 finite values"};
    }
    if (!has_finite_values(covariance_values, k_covariance_size))
    {
      return {k_invalid_covariance, "covariance must contain 225 finite values"};
    }
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      if (covariance_values[row * k_state_size + row] < 0.0)
      {
        return {k_invalid_covariance, "covariance must have a non-negative diagonal"};
      }
      state(static_cast<Eigen::Index>(row)) = state_values[row];
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          covariance_values[row * k_state_size + column];
      }
    }
    return {k_ok, ""};
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

  static bool has_non_negative_diagonal(const rpp_localization::CovarianceMatrix& covariance)
  {
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      if (covariance(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(index)) < 0.0)
      {
        return false;
      }
    }
    return true;
  }

  template<typename Control>
  more_dynamics::VehicleModel3D::Command::List build_commands(const Control& control) const
  {
    more_dynamics::VehicleModel3D::Command::List commands;
    commands.resize(command_sizes_.size());
    for (std::size_t command_index = 0; command_index < command_sizes_.size(); ++command_index)
    {
      auto command = commands[command_index];
      auto values = command.data();
      values.resize(static_cast<std::size_t>(command_sizes_[command_index]));
      for (std::size_t data_index = 0; data_index < static_cast<std::size_t>(command_sizes_[command_index]); ++data_index)
      {
        values[data_index] = 0.0;
      }
    }

    const auto values = control.values();
    const auto enabled = control.enabled();
    for (std::size_t axis = 0; axis < k_control_size; ++axis)
    {
      const auto command_index = static_cast<std::size_t>(control_command_indices_[axis]);
      const auto data_index = static_cast<std::size_t>(control_data_indices_[axis]);
      auto command = commands[command_index];
      auto command_values = command.data();
      const bool active = control.present() && enabled[axis];
      command_values[data_index] = active
        ? values[axis] * control_scales_[axis] + control_offsets_[axis]
        : control_offsets_[axis];
    }
    return commands;
  }

  static more_dynamics::VehicleModel3D::Odometry3D to_vehicle_state(
    const rpp_localization::StateVector& state)
  {
    more_dynamics::VehicleModel3D::Odometry3D output;
    auto pose = output.pose();
    auto position = pose.position();
    position.x() = state(rpp_localization::StateMemberX);
    position.y() = state(rpp_localization::StateMemberY);
    position.z() = state(rpp_localization::StateMemberZ);

    const double roll = state(rpp_localization::StateMemberRoll);
    const double pitch = state(rpp_localization::StateMemberPitch);
    const double yaw = state(rpp_localization::StateMemberYaw);
    const double half_roll = roll * 0.5;
    const double half_pitch = pitch * 0.5;
    const double half_yaw = yaw * 0.5;
    auto orientation = pose.orientation();
    orientation.w() = std::cos(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) +
      std::sin(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
    orientation.x() = std::sin(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) -
      std::cos(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
    orientation.y() = std::cos(half_roll) * std::sin(half_pitch) * std::cos(half_yaw) +
      std::sin(half_roll) * std::cos(half_pitch) * std::sin(half_yaw);
    orientation.z() = std::cos(half_roll) * std::cos(half_pitch) * std::sin(half_yaw) -
      std::sin(half_roll) * std::sin(half_pitch) * std::cos(half_yaw);

    auto twist = output.twist();
    auto linear = twist.linear();
    linear.x() = state(rpp_localization::StateMemberVx);
    linear.y() = state(rpp_localization::StateMemberVy);
    linear.z() = state(rpp_localization::StateMemberVz);
    auto angular = twist.angular();
    angular.x() = state(rpp_localization::StateMemberVroll);
    angular.y() = state(rpp_localization::StateMemberVpitch);
    angular.z() = state(rpp_localization::StateMemberVyaw);
    return output;
  }

  static rpp_localization::StateVector from_vehicle_state(
    const more_dynamics::VehicleModel3D::Odometry3D::Const& output,
    const rpp_localization::StateVector& previous_state,
    const rpp_localization::DurationNs delta)
  {
    const auto position = output.pose().position();
    const auto orientation = output.pose().orientation();
    const auto linear = output.twist().linear();
    const auto angular = output.twist().angular();
    const double quaternion_norm = std::sqrt(
      orientation.x() * orientation.x() + orientation.y() * orientation.y() +
      orientation.z() * orientation.z() + orientation.w() * orientation.w());
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
      !std::isfinite(position.z()) || !std::isfinite(linear.x()) || !std::isfinite(linear.y()) ||
      !std::isfinite(linear.z()) || !std::isfinite(angular.x()) || !std::isfinite(angular.y()) ||
      !std::isfinite(angular.z()) || !std::isfinite(quaternion_norm) ||
      quaternion_norm <= std::numeric_limits<double>::epsilon())
    {
      throw std::runtime_error("VehicleModel3D returned a non-finite or degenerate state");
    }

    const double qx = orientation.x() / quaternion_norm;
    const double qy = orientation.y() / quaternion_norm;
    const double qz = orientation.z() / quaternion_norm;
    const double qw = orientation.w() / quaternion_norm;
    rpp_localization::StateVector state(k_state_size);
    state(rpp_localization::StateMemberX) = position.x();
    state(rpp_localization::StateMemberY) = position.y();
    state(rpp_localization::StateMemberZ) = position.z();
    state(rpp_localization::StateMemberRoll) = std::atan2(
      2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    state(rpp_localization::StateMemberPitch) = std::asin(std::clamp(
      2.0 * (qw * qy - qz * qx), -1.0, 1.0));
    state(rpp_localization::StateMemberYaw) = std::atan2(
      2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    state(rpp_localization::StateMemberVx) = linear.x();
    state(rpp_localization::StateMemberVy) = linear.y();
    state(rpp_localization::StateMemberVz) = linear.z();
    state(rpp_localization::StateMemberVroll) = angular.x();
    state(rpp_localization::StateMemberVpitch) = angular.y();
    state(rpp_localization::StateMemberVyaw) = angular.z();

    if (delta == 0)
    {
      state.segment<3>(rpp_localization::StateMemberAx) =
        previous_state.segment<3>(rpp_localization::StateMemberAx);
    }
    else
    {
      const double delta_seconds = rpp_localization::nanoseconds_to_seconds(delta);
      state.segment<3>(rpp_localization::StateMemberAx) =
        (state.segment<3>(rpp_localization::StateMemberVx) -
        previous_state.segment<3>(rpp_localization::StateMemberVx)) / delta_seconds;
    }
    return state;
  }

  template<typename Control>
  rpp_localization::StateVector propagate_state(
    const rpp_localization::StateVector& state,
    const Control& control,
    const rpp_localization::TimestampNs reference_time,
    const rpp_localization::DurationNs delta) const
  {
    auto vehicle_state = to_vehicle_state(state);
    auto commands = build_commands(control);
    const auto propagated = vehicle_model_->step(
      std::move(vehicle_state), std::move(commands),
      rpp_localization::nanoseconds_to_seconds(reference_time),
      rpp_localization::nanoseconds_to_seconds(delta));
    return from_vehicle_state(propagated, state, delta);
  }

  template<typename Control>
  rpp_localization::CovarianceMatrix numerical_transition(
    const rpp_localization::StateVector& state,
    const rpp_localization::StateVector& propagated,
    const Control& control,
    const rpp_localization::TimestampNs reference_time,
    const rpp_localization::DurationNs delta) const
  {
    rpp_localization::CovarianceMatrix transition(k_state_size, k_state_size);
    transition.setZero();
    for (std::size_t column = 0; column < k_state_size; ++column)
    {
      auto perturbed = state;
      const double perturbation = jacobian_epsilon_ * std::max(
        1.0, std::abs(state(static_cast<Eigen::Index>(column))));
      perturbed(static_cast<Eigen::Index>(column)) += perturbation;
      const auto perturbed_output =
        propagate_state(perturbed, control, reference_time, delta);
      for (std::size_t row = 0; row < k_state_size; ++row)
      {
        double difference = perturbed_output(static_cast<Eigen::Index>(row)) -
          propagated(static_cast<Eigen::Index>(row));
        if (row == rpp_localization::StateMemberRoll ||
          row == rpp_localization::StateMemberPitch ||
          row == rpp_localization::StateMemberYaw)
        {
          difference = std::atan2(std::sin(difference), std::cos(difference));
        }
        transition(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          difference / perturbation;
      }
    }
    return transition;
  }

  static void write_output(
    LocalizationModelPredictOutput15& output,
    const rpp_localization::StateVector& state,
    const rpp_localization::CovarianceMatrix& covariance)
  {
    auto state_values = output.state().values();
    state_values.resize(k_state_size);
    auto covariance_values = output.covariance().values();
    covariance_values.resize(k_covariance_size);
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      state_values[row] = state(static_cast<Eigen::Index>(row));
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        covariance_values[row * k_state_size + column] =
          covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
      }
    }
  }

  static void write_zero_output(LocalizationModelPredictOutput15& output)
  {
    write_output(output,
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

  std::shared_ptr<more_dynamics::VehicleModel3D> vehicle_model_;
  std::vector<std::int64_t> command_sizes_;
  std::vector<std::int64_t> control_command_indices_;
  std::vector<std::int64_t> control_data_indices_;
  std::vector<double> control_scales_;
  std::vector<double> control_offsets_;
  rpp_localization::CovarianceMatrix process_noise_covariance_;
  double jacobian_epsilon_{1e-6};
  bool ready_{false};
};
