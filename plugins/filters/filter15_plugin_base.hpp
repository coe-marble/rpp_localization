#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_localization/LocalizationFilter15.hpp>

#include <rpp_localization/core/filter_base.hpp>
#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/core/model_base.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav_model15_adapter.hpp"

// Shared RPP lifecycle adapter. Derived plugins choose Ekf or Ukf; this class
// owns the model adapter, runtime filter, payload validation, and replay state.
class Filter15PluginBase : public rpp_localization::LocalizationFilter15
{
public:
  static constexpr std::size_t k_state_size = rpp_localization::STATE_SIZE;
  static constexpr std::size_t k_control_size = rpp_localization::TWIST_SIZE;
  static constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

  Filter15PluginBase() = default;
  ~Filter15PluginBase() override = default;

  void initialize(const rpp::ComponentContext& context) override
  {
    initialize_with_model(context.get_component<rpp_localization::NavModel15>("model"));
  }

  void reset() override
  {
    filter_.reset();
    filter_ready_ = false;
    reference_time_ = 0;
    initial_reference_time_ = 0;
    history_.clear();
    if (model_runtime_)
    {
      model_runtime_->reset_runtime();
    }
  }

  LocalizationStatus::Const initialize(Estimate15::Const initial_estimate) override
  {
    LocalizationStatus status;
    if (!component_ready_ || !model_runtime_)
    {
      set_status(status, k_not_initialized, "NavModel15 component is not initialized");
      return status;
    }

    rpp_localization::StateVector state(k_state_size);
    rpp_localization::CovarianceMatrix covariance(k_state_size, k_state_size);
    const auto validation = read_estimate(initial_estimate, state, covariance);
    if (validation.code != k_ok)
    {
      set_status(status, validation.code, validation.message);
      return status;
    }

    model_runtime_->set_state(state);
    model_runtime_->set_state_covariance(covariance);
    try
    {
      filter_ = create_filter(*model_runtime_);
    }
    catch (const std::exception& error)
    {
      set_status(status, k_model_failure, error.what());
      return status;
    }
    if (!filter_)
    {
      set_status(status, k_model_failure, "filter factory returned no runtime filter");
      return status;
    }

    initial_state_ = std::move(state);
    initial_covariance_ = std::move(covariance);
    reference_time_ = initial_estimate.referenceTimeNs();
    initial_reference_time_ = reference_time_;
    history_.clear();
    filter_ready_ = true;
    set_status(status, k_ok, "");
    return status;
  }

  LocalizationFilterResult15::Const predict(LocalizationPredictInput15::Const input) override
  {
    LocalizationFilterResult15 result;
    write_zero_result(result);
    if (!filter_ready_ || !filter_ || !model_runtime_)
    {
      set_result_status(result, k_not_initialized, "filter is not initialized");
      return result;
    }
    const auto elapsed = static_cast<std::uint64_t>(input.referenceTimeNs()) -
      static_cast<std::uint64_t>(reference_time_);
    if (input.deltaNs() < 0 || input.referenceTimeNs() < reference_time_ ||
      static_cast<std::uint64_t>(input.deltaNs()) != elapsed)
    {
      write_current_estimate(result);
      set_result_status(result, k_invalid_time,
        "referenceTimeNs must advance by exactly deltaNs");
      return result;
    }

    const auto input_control = input.control();
    if (!has_valid_control(input_control))
    {
      write_current_estimate(result);
      set_result_status(result, k_invalid_payload,
        "present control must contain six enabled flags and six finite enabled values");
      return result;
    }

    const PredictionEvent prediction{
      input.referenceTimeNs(), input.deltaNs(), read_control(input_control)};
    const auto previous_state = model_runtime_->get_state();
    const auto previous_covariance = model_runtime_->get_state_covariance();
    const auto previous_control = model_runtime_->get_control_input();
    try
    {
      apply_prediction(prediction);
    }
    catch (const rpp_localization::plugins::detail::ModelPredictionError& error)
    {
      restore_runtime(previous_state, previous_covariance, previous_control);
      write_current_estimate(result);
      set_result_status(result, error.code(), error.what());
      return result;
    }
    catch (const std::exception& error)
    {
      restore_runtime(previous_state, previous_covariance, previous_control);
      write_current_estimate(result);
      set_result_status(result, k_model_failure, error.what());
      return result;
    }

    if (!has_valid_state_and_covariance())
    {
      restore_runtime(previous_state, previous_covariance, previous_control);
      write_current_estimate(result);
      set_result_status(result, k_numerical_failure, "prediction produced an invalid estimate");
      return result;
    }

    history_.push_back(HistoryEvent::prediction(prediction));
    reference_time_ = prediction.reference_time;
    write_current_estimate(result);
    set_result_status(result, k_ok, "");
    return result;
  }

  LocalizationFilterResult15::Const correct(Measurement15::Const measurement) override
  {
    LocalizationFilterResult15 result;
    write_zero_result(result);
    if (!filter_ready_ || !filter_ || !model_runtime_)
    {
      set_result_status(result, k_not_initialized, "filter is not initialized");
      return result;
    }

    rpp_localization::Measurement core_measurement;
    const auto validation = read_measurement(measurement, core_measurement);
    if (validation.code != k_ok)
    {
      write_current_estimate(result);
      set_result_status(result, validation.code, validation.message);
      return result;
    }
    if (core_measurement.time_ > reference_time_)
    {
      write_current_estimate(result);
      set_result_status(result, k_invalid_time, "measurement follows the current estimate; predict first");
      return result;
    }
    if (core_measurement.time_ < initial_reference_time_)
    {
      write_current_estimate(result);
      set_result_status(result, k_stale_measurement,
        "measurement precedes the retained initialization state");
      return result;
    }

    if (core_measurement.time_ < reference_time_)
    {
      if (!replay_stale_measurement(core_measurement))
      {
        write_current_estimate(result);
        set_result_status(result, k_stale_measurement,
          "measurement could not be replayed through the retained prediction history");
        return result;
      }
    }
    else if (!apply_current_measurement(core_measurement))
    {
      write_current_estimate(result);
      set_result_status(result, k_numerical_failure, "correction produced an invalid estimate");
      return result;
    }

    write_current_estimate(result);
    set_result_status(result, k_ok, "");
    return result;
  }

  NavModelDescription15::Const describeModel() override
  {
    if (!model_component_)
    {
      throw std::logic_error("NavModel15 component is not initialized");
    }
    return model_component_->describe();
  }

  LocalizationFilterResult15::Const getEstimate() override
  {
    LocalizationFilterResult15 result;
    write_zero_result(result);
    if (!filter_ready_ || !model_runtime_)
    {
      set_result_status(result, k_not_initialized, "filter is not initialized");
      return result;
    }

    write_current_estimate(result);
    set_result_status(result, k_ok, "");
    return result;
  }

  LocalizationStatus::Const reset(Estimate15::Const initial_estimate) override
  {
    reset();
    return initialize(std::move(initial_estimate));
  }

protected:
  // Separate from ComponentContext lookup so boundary tests can inject an
  // in-process NavModel15 implementation.
  void initialize_with_model(std::shared_ptr<rpp_localization::NavModel15> model)
  {
    filter_.reset();
    model_runtime_.reset();
    model_component_ = std::move(model);
    model_runtime_ = std::make_unique<rpp_localization::plugins::detail::NavModel15Adapter>(
      model_component_);
    component_ready_ = true;
    filter_ready_ = false;
    reference_time_ = 0;
    initial_reference_time_ = 0;
    history_.clear();
  }

  virtual std::unique_ptr<rpp_localization::FilterBase> create_filter(
    rpp_localization::ModelBase& model) = 0;

private:
  struct ValidationResult
  {
    std::uint16_t code;
    std::string message;
  };

  struct PredictionEvent
  {
    rpp_localization::TimestampNs reference_time;
    rpp_localization::DurationNs delta;
    rpp_localization::plugins::detail::ModelControl control;
  };

  struct HistoryEvent
  {
    enum class Kind
    {
      k_prediction,
      k_measurement
    };

    HistoryEvent(
      const Kind event_kind,
      const rpp_localization::TimestampNs event_time,
      PredictionEvent prediction,
      rpp_localization::Measurement measurement)
    : kind(event_kind),
      time(event_time),
      prediction_input(std::move(prediction)),
      measurement_input(std::move(measurement))
    {
    }

    static HistoryEvent prediction(PredictionEvent input)
    {
      return {Kind::k_prediction, input.reference_time, std::move(input), {}};
    }

    static HistoryEvent measurement(rpp_localization::Measurement input)
    {
      return {Kind::k_measurement, input.time_, {}, std::move(input)};
    }

    Kind kind;
    rpp_localization::TimestampNs time;
    PredictionEvent prediction_input;
    rpp_localization::Measurement measurement_input;
  };

  static constexpr std::uint16_t k_ok = 0;
  static constexpr std::uint16_t k_invalid_payload = 1;
  static constexpr std::uint16_t k_invalid_time = 2;
  static constexpr std::uint16_t k_invalid_state = 3;
  static constexpr std::uint16_t k_invalid_covariance = 4;
  static constexpr std::uint16_t k_not_initialized = 5;
  static constexpr std::uint16_t k_stale_measurement = 6;
  static constexpr std::uint16_t k_model_failure = 7;
  static constexpr std::uint16_t k_numerical_failure = 9;

  template<typename Values>
  static bool has_finite_values(const Values& values, const std::size_t expected_size)
  {
    return rpp_localization::plugins::detail::NavModel15Adapter::has_finite_values(
      values, expected_size);
  }

  template<typename Values>
  static bool has_valid_state_covariance(const Values& values)
  {
    return rpp_localization::plugins::detail::NavModel15Adapter::has_valid_state_covariance(
      values);
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

  template<typename Control>
  static rpp_localization::plugins::detail::ModelControl read_control(const Control& control)
  {
    rpp_localization::plugins::detail::ModelControl output;
    output.present = control.present();
    output.stamp = control.stampNs();
    if (!output.present)
    {
      return output;
    }

    const auto values = control.values();
    const auto enabled = control.enabled();
    for (std::size_t index = 0; index < k_control_size; ++index)
    {
      output.values[index] = values[index];
      output.enabled[index] = enabled[index];
    }
    return output;
  }

  static bool has_valid_mahalanobis_threshold(const double threshold)
  {
    return threshold >= 0.0 &&
      (std::isfinite(threshold) || threshold == std::numeric_limits<double>::infinity());
  }

  static ValidationResult read_estimate(
    const Estimate15::Const& input,
    rpp_localization::StateVector& state,
    rpp_localization::CovarianceMatrix& covariance)
  {
    const auto state_values = input.state().values();
    const auto covariance_values = input.covariance().values();
    if (!has_finite_values(state_values, k_state_size))
    {
      return {k_invalid_state, "state must contain 15 finite values"};
    }
    if (!has_valid_state_covariance(covariance_values))
    {
      return {k_invalid_covariance,
        "covariance must contain 225 finite values with non-negative diagonal"};
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
    return {k_ok, ""};
  }

  static ValidationResult read_measurement(
    const Measurement15::Const& input,
    rpp_localization::Measurement& measurement)
  {
    const auto state_values = input.state().values();
    const auto covariance_values = input.covariance().values();
    const auto update_mask = input.updateMask();
    if (!has_finite_values(state_values, k_state_size))
    {
      return {k_invalid_state, "measurement state must contain 15 finite values"};
    }
    if (!has_finite_values(covariance_values, k_covariance_size))
    {
      return {k_invalid_covariance, "measurement covariance must contain 225 finite values"};
    }
    if (update_mask.size() != k_state_size)
    {
      return {k_invalid_payload, "measurement updateMask must contain 15 values"};
    }
    if (!has_valid_mahalanobis_threshold(input.mahalanobisThreshold()))
    {
      return {k_invalid_payload,
        "mahalanobisThreshold must be non-negative or positive infinity"};
    }

    measurement.time_ = input.referenceTimeNs();
    measurement.mahalanobis_thresh_ = input.mahalanobisThreshold();
    measurement.topic_name_ = input.sourceName();
    measurement.update_vector_.assign(k_state_size, false);
    measurement.measurement_.resize(k_state_size);
    measurement.covariance_.resize(k_state_size, k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      measurement.update_vector_[index] = update_mask[index];
      measurement.measurement_(static_cast<Eigen::Index>(index)) = state_values[index];
    }
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        measurement.covariance_(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          covariance_values[row * k_state_size + column];
      }
    }
    return {k_ok, ""};
  }

  bool apply_current_measurement(const rpp_localization::Measurement& measurement)
  {
    const auto previous_state = model_runtime_->get_state();
    const auto previous_covariance = model_runtime_->get_state_covariance();
    try
    {
      filter_->correct(measurement);
    }
    catch (const std::exception&)
    {
      model_runtime_->set_state(previous_state);
      model_runtime_->set_state_covariance(previous_covariance);
      return false;
    }
    if (!has_valid_state_and_covariance())
    {
      model_runtime_->set_state(previous_state);
      model_runtime_->set_state_covariance(previous_covariance);
      return false;
    }

    history_.push_back(HistoryEvent::measurement(measurement));
    return true;
  }

  void apply_prediction(const PredictionEvent& prediction)
  {
    model_runtime_->set_control(prediction.control);
    filter_->predict(prediction.reference_time, prediction.delta);
  }

  bool replay_stale_measurement(const rpp_localization::Measurement& measurement)
  {
    const auto insertion = std::upper_bound(
      history_.begin(), history_.end(), measurement.time_,
      [](const rpp_localization::TimestampNs timestamp, const HistoryEvent& event)
      {
        return timestamp < event.time;
      });
    const auto insertion_index = static_cast<std::size_t>(
      std::distance(history_.begin(), insertion));
    history_.insert(insertion, HistoryEvent::measurement(measurement));

    const auto previous_state = model_runtime_->get_state();
    const auto previous_covariance = model_runtime_->get_state_covariance();
    const auto previous_control = model_runtime_->get_control_input();
    const auto previous_reference_time = reference_time_;
    auto previous_filter = std::move(filter_);

    try
    {
      model_runtime_->set_state(initial_state_);
      model_runtime_->set_state_covariance(initial_covariance_);
      model_runtime_->set_control(rpp_localization::plugins::detail::ModelControl{});
      auto replay_filter = create_filter(*model_runtime_);
      if (!replay_filter)
      {
        throw std::runtime_error("filter factory returned no runtime filter during replay");
      }

      reference_time_ = initial_reference_time_;
      for (std::size_t index = 0; index < history_.size(); ++index)
      {
        const auto& event = history_[index];
        if (event.kind == HistoryEvent::Kind::k_prediction)
        {
          if (event.time < reference_time_)
          {
            throw std::runtime_error("prediction history is not chronological");
          }
          model_runtime_->set_control(event.prediction_input.control);
          replay_filter->predict(event.time, event.time - reference_time_);
          reference_time_ = event.time;
        }
        else
        {
          if (event.time < reference_time_)
          {
            throw std::runtime_error("measurement history is not chronological");
          }
          if (event.time > reference_time_)
          {
            const auto prediction = find_covering_prediction(index, event.time);
            if (prediction == nullptr)
            {
              throw std::runtime_error("no prediction interval covers the stale measurement");
            }
            model_runtime_->set_control(prediction->control);
            replay_filter->predict(event.time, event.time - reference_time_);
            reference_time_ = event.time;
          }
          replay_filter->correct(event.measurement_input);
        }

        if (!has_valid_state_and_covariance())
        {
          throw std::runtime_error("replay produced an invalid estimate");
        }
      }

      filter_ = std::move(replay_filter);
      return true;
    }
    catch (const std::exception&)
    {
      history_.erase(history_.begin() + static_cast<std::ptrdiff_t>(insertion_index));
      restore_runtime(previous_state, previous_covariance, previous_control);
      reference_time_ = previous_reference_time;
      filter_ = std::move(previous_filter);
      return false;
    }
  }

  const PredictionEvent* find_covering_prediction(
    const std::size_t from_index,
    const rpp_localization::TimestampNs timestamp) const
  {
    for (std::size_t index = from_index; index < history_.size(); ++index)
    {
      const auto& event = history_[index];
      if (event.kind == HistoryEvent::Kind::k_prediction && event.time >= timestamp)
      {
        return &event.prediction_input;
      }
    }
    return nullptr;
  }

  [[nodiscard]] bool has_valid_state_and_covariance() const
  {
    const auto& state = model_runtime_->get_state();
    const auto& covariance = model_runtime_->get_state_covariance();
    if (state.size() != static_cast<Eigen::Index>(k_state_size) ||
      covariance.rows() != static_cast<Eigen::Index>(k_state_size) ||
      covariance.cols() != static_cast<Eigen::Index>(k_state_size) ||
      !state.allFinite() || !covariance.allFinite())
    {
      return false;
    }

    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      if (covariance(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(index)) < 0.0)
      {
        return false;
      }
    }
    return true;
  }

  void restore_runtime(
    const rpp_localization::StateVector& state,
    const rpp_localization::CovarianceMatrix& covariance,
    const rpp_localization::plugins::detail::ModelControl& control)
  {
    model_runtime_->set_state(state);
    model_runtime_->set_state_covariance(covariance);
    model_runtime_->set_control(control);
  }

  static void write_estimate(
    Estimate15 estimate,
    const rpp_localization::StateVector& state,
    const rpp_localization::CovarianceMatrix& covariance,
    const rpp_localization::TimestampNs reference_time)
  {
    auto state_values = estimate.state().values();
    state_values.resize(k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      state_values[index] = state(static_cast<Eigen::Index>(index));
    }

    auto covariance_values = estimate.covariance().values();
    covariance_values.resize(k_covariance_size);
    for (std::size_t row = 0; row < k_state_size; ++row)
    {
      for (std::size_t column = 0; column < k_state_size; ++column)
      {
        covariance_values[row * k_state_size + column] =
          covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
      }
    }
    estimate.referenceTimeNs() = reference_time;
  }

  static void set_status(LocalizationStatus& status, const std::uint16_t code, std::string message)
  {
    status.code() = code;
    status.message() = std::move(message);
  }

  static void set_result_status(
    LocalizationFilterResult15& result,
    const std::uint16_t code,
    std::string message)
  {
    auto status = result.status();
    set_status(status, code, std::move(message));
  }

  static void write_zero_result(LocalizationFilterResult15& result)
  {
    write_estimate(
      result.estimate(),
      rpp_localization::StateVector::Zero(k_state_size),
      rpp_localization::CovarianceMatrix::Zero(k_state_size, k_state_size),
      0);
  }

  void write_current_estimate(LocalizationFilterResult15& result) const
  {
    write_estimate(
      result.estimate(),
      model_runtime_->get_state(),
      model_runtime_->get_state_covariance(),
      reference_time_);
  }

  std::shared_ptr<rpp_localization::NavModel15> model_component_;
  std::unique_ptr<rpp_localization::plugins::detail::NavModel15Adapter> model_runtime_;
  std::unique_ptr<rpp_localization::FilterBase> filter_;
  rpp_localization::StateVector initial_state_;
  rpp_localization::CovarianceMatrix initial_covariance_;
  std::vector<HistoryEvent> history_;
  rpp_localization::TimestampNs reference_time_{0};
  rpp_localization::TimestampNs initial_reference_time_{0};
  bool component_ready_{false};
  bool filter_ready_{false};
};
