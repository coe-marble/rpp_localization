#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/InertialModel.hpp>
#include <rpp_plugin_types/rpp_localization/InvariantFilter.hpp>

#include <rpp_localization/inekf/inertial_payload.hpp>
#include <rpp_localization/inekf/invariant_ekf.hpp>

#include <kj/exception.h>

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace rpp_localization::plugins::detail
{

// Lets the core filter predict through an InertialModel component.
class InertialModelAdapter final : public InEKF::InertialPredictionModel
{
public:
  explicit InertialModelAdapter(std::shared_ptr<InertialModel> model)
  : model_(std::move(model))
  {
    if (!model_)
    {
      throw std::invalid_argument("InertialModel component is required");
    }
  }

  void predict(
    InEKF::InertialLieState& state,
    const InEKF::InertialPredictionInput& input) override
  {
    InertialModel::InertialModelPredictInput payload;
    write_inertial_state(payload.state(), state);
    write_imu(payload.imu(), input.imu);
    payload.referenceTimeNs() = input.reference_time;
    payload.deltaNs() = input.delta;

    const auto output = model_->predict(std::move(payload));
    const auto status = output.status();
    if (status.code() != k_inertial_ok)
    {
      throw InertialPayloadError(status.code(), status.message());
    }
    state = read_inertial_state(output.state());
  }

private:
  std::shared_ptr<InertialModel> model_;
};

}  // namespace rpp_localization::plugins::detail

// Right-invariant extended Kalman filter as an InvariantFilter. The filter
// equations are in rpp_localization::InEKF::InvariantEkf; this class owns
// the model adapter, payload validation, the estimate time and the handover.
class Inekf final : public rpp_localization::InvariantFilter
{
public:
  RPP_COMPONENTS(
    {"model", "rpp_localization::InertialModel"}
  )

  static constexpr std::uint16_t k_position = 0;
  static constexpr std::uint16_t k_body_velocity = 1;
  static constexpr std::uint16_t k_attitude = 2;

  Inekf() = default;
  ~Inekf() override = default;

  rpp::CallbackResult on_configure(const rpp::ComponentContext& context) override
  {
    context_ = &context;
    initialize_with_model(
      context.get_subcomponent_context("model").get_instance<rpp_localization::InertialModel>());
    return {};
  }

  rpp::CallbackResult on_cleanup() override
  {
    reset();
    model_.reset();
    context_ = nullptr;
    return {};
  }

  rpp::CallbackResult on_activate(const std::optional<InertialEstimate::Const>& state) override
  {
    if (!context_ || !model_)
    {
      return rpp::CallbackResult::failure("InvariantFilter must be configured before activation");
    }

    const auto& model_context = context_->get_subcomponent_context("model");
    auto result = model_context.activate();
    if (!result.success)
    {
      return result;
    }
    reset();
    if (!state)
    {
      return {};
    }

    const auto restored = restore_estimate(*state);
    if (restored.code != rpp_localization::plugins::detail::k_inertial_ok)
    {
      reset();
      model_context.deactivate();
      return rpp::CallbackResult::failure(
        "InvariantFilter rejected the handover estimate: " + restored.message);
    }
    return {};
  }

  rpp::DeactivationResult<InertialEstimate::Const> on_deactivate() override
  {
    std::optional<InertialEstimate::Const> handover;
    if (filter_)
    {
      InertialEstimate estimate;
      write_estimate(estimate);
      handover.emplace(std::move(estimate));
    }
    reset();
    if (!context_)
    {
      return {rpp::CallbackResult::failure(
          "InvariantFilter must be configured before deactivation"), std::nullopt};
    }
    auto result = context_->get_subcomponent_context("model").deactivate().result;
    if (!result.success)
    {
      return {std::move(result), std::nullopt};
    }
    return {std::move(result), std::move(handover)};
  }

  void reset() override
  {
    filter_.reset();
    reference_time_ = 0;
  }

  LocalizationStatus::Const initialize(InertialEstimate::Const initial_estimate) override
  {
    LocalizationStatus status;
    if (!model_)
    {
      set_status(
        status, rpp_localization::plugins::detail::k_inertial_not_initialized,
        "InertialModel is not initialized");
      return status;
    }
    const auto restored = restore_estimate(initial_estimate);
    set_status(status, restored.code, restored.message);
    return status;
  }

  LocalizationStatus::Const reset(InertialEstimate::Const initial_estimate) override
  {
    reset();
    return initialize(std::move(initial_estimate));
  }

  InvariantFilterResult::Const predict(InvariantPredictInput::Const input) override
  {
    namespace detail = rpp_localization::plugins::detail;
    InvariantFilterResult result;
    if (!require_ready(result))
    {
      return result;
    }
    if (input.deltaNs() < 0 || input.referenceTimeNs() - input.deltaNs() != reference_time_)
    {
      finish(
        result, detail::k_inertial_invalid_time,
        "referenceTimeNs must advance by exactly deltaNs", false);
      return result;
    }

    const auto previous = filter_->state();
    const auto previous_rate = filter_->angular_velocity();
    try
    {
      rpp_localization::InEKF::InertialPredictionInput step;
      step.reference_time = input.referenceTimeNs();
      step.delta = input.deltaNs();
      step.imu = detail::read_imu(input.imu());
      filter_->predict(step);
      reference_time_ = input.referenceTimeNs();
      finish(result, detail::k_inertial_ok, "", true);
      return result;
    }
    catch (const detail::InertialPayloadError& error)
    {
      rollback(previous, previous_rate);
      finish(result, error.code(), error.what(), false);
    }
    catch (const kj::Exception& error)
    {
      // A model in another process reports its failure this way.
      rollback(previous, previous_rate);
      finish(result, detail::k_inertial_model_failure, error.getDescription().cStr(), false);
    }
    catch (const std::exception& error)
    {
      rollback(previous, previous_rate);
      finish(result, detail::k_inertial_model_failure, error.what(), false);
    }
    return result;
  }

  InvariantFilterResult::Const correct(InvariantMeasurement::Const measurement) override
  {
    namespace detail = rpp_localization::plugins::detail;
    InvariantFilterResult result;
    if (!require_ready(result))
    {
      return result;
    }
    if (measurement.referenceTimeNs() > reference_time_)
    {
      finish(
        result, detail::k_inertial_invalid_time,
        "measurement follows the current estimate; predict first", false);
      return result;
    }
    const double threshold = measurement.mahalanobisThreshold();
    if (std::isnan(threshold) || threshold <= 0.0)
    {
      finish(
        result, detail::k_inertial_invalid_payload,
        "mahalanobisThreshold must be positive", false);
      return result;
    }

    const auto previous = filter_->state();
    try
    {
      const Eigen::Matrix3d covariance =
        detail::read_matrix3(measurement.covariance(), "covariance");
      bool accepted = false;
      switch (measurement.kind())
      {
        case k_position:
          accepted = filter_->correct_position(
            detail::read_vector3(measurement.values(), "position values"), covariance,
            detail::read_vector3(measurement.leverArm(), "leverArm"), threshold);
          break;
        case k_body_velocity:
          accepted = filter_->correct_body_velocity(
            detail::read_vector3(measurement.values(), "bodyVelocity values"), covariance,
            detail::read_vector3(measurement.leverArm(), "leverArm"), threshold);
          break;
        case k_attitude:
          accepted = filter_->correct_attitude(
            detail::read_matrix3(measurement.values(), "attitude values"), covariance,
            threshold);
          break;
        default:
          finish(result, detail::k_inertial_invalid_payload, "unknown measurement kind", false);
          return result;
      }
      if (!filter_->state()().allFinite() || !filter_->state().cov().allFinite())
      {
        filter_->set_state(previous);
        finish(
          result, detail::k_inertial_numerical_failure,
          "correction produced an invalid estimate", false);
        return result;
      }
      finish(result, detail::k_inertial_ok, "", accepted);
    }
    catch (const detail::InertialPayloadError& error)
    {
      filter_->set_state(previous);
      finish(result, error.code(), error.what(), false);
    }
    catch (const std::exception& error)
    {
      filter_->set_state(previous);
      finish(result, detail::k_inertial_invalid_payload, error.what(), false);
    }
    return result;
  }

  InvariantFilterResult::Const getEstimate() override
  {
    InvariantFilterResult result;
    if (require_ready(result))
    {
      finish(result, rpp_localization::plugins::detail::k_inertial_ok, "", true);
    }
    return result;
  }

protected:
  // Separate from ComponentContext lookup so tests can inject an in-process model.
  void initialize_with_model(std::shared_ptr<rpp_localization::InertialModel> model)
  {
    reset();
    model_ = std::make_unique<rpp_localization::plugins::detail::InertialModelAdapter>(
      std::move(model));
  }

private:
  struct RestoreResult
  {
    std::uint16_t code;
    std::string message;
  };

  // Validates the whole estimate before replacing the running filter.
  RestoreResult restore_estimate(const InertialEstimate::Const& estimate)
  {
    namespace detail = rpp_localization::plugins::detail;
    try
    {
      auto filter = std::make_unique<rpp_localization::InEKF::InvariantEkf>(*model_);
      filter->set_state(detail::read_inertial_state(estimate.state()));
      if (estimate.angularVelocity().size() != 0)
      {
        filter->set_angular_velocity(
          detail::read_vector3(estimate.angularVelocity(), "angularVelocity"));
      }
      filter_ = std::move(filter);
      reference_time_ = estimate.referenceTimeNs();
      return {detail::k_inertial_ok, ""};
    }
    catch (const detail::InertialPayloadError& error)
    {
      return {error.code(), error.what()};
    }
    catch (const std::exception& error)
    {
      return {detail::k_inertial_invalid_state, error.what()};
    }
  }

  void rollback(
    const rpp_localization::InEKF::InertialLieState& state,
    const Eigen::Vector3d& angular_velocity)
  {
    filter_->set_state(state);
    filter_->set_angular_velocity(angular_velocity);
  }

  bool require_ready(InvariantFilterResult& result)
  {
    if (filter_)
    {
      return true;
    }
    auto status = result.status();
    status.code() = rpp_localization::plugins::detail::k_inertial_not_initialized;
    status.message() = "filter is not initialized";
    result.accepted() = false;
    return false;
  }

  template<typename EstimateBuilder>
  void write_estimate(EstimateBuilder&& estimate) const
  {
    namespace detail = rpp_localization::plugins::detail;
    detail::write_inertial_state(estimate.state(), filter_->state());
    detail::write_values(estimate.angularVelocity(), filter_->angular_velocity());
    estimate.referenceTimeNs() = reference_time_;
  }

  void finish(
    InvariantFilterResult& result,
    const std::uint16_t code,
    std::string message,
    const bool accepted)
  {
    write_estimate(result.estimate());
    auto status = result.status();
    status.code() = code;
    status.message() = std::move(message);
    result.accepted() = accepted;
  }

  static void set_status(LocalizationStatus& status, const std::uint16_t code, std::string message)
  {
    status.code() = code;
    status.message() = std::move(message);
  }

  std::unique_ptr<rpp_localization::plugins::detail::InertialModelAdapter> model_;
  std::unique_ptr<rpp_localization::InEKF::InvariantEkf> filter_;
  const rpp::ComponentContext* context_{nullptr};
  rpp_localization::TimestampNs reference_time_{0};
};
