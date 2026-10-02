#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/core/validation.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"

namespace rpp_localization
{
namespace
{

class RecordingModel final : public ModelBase
{
public:
  explicit RecordingModel(const int state_dimension = 2)
  : ModelBase(state_dimension)
  {
  }

  void predict(
    StateVector& state,
    CovarianceMatrix& covariance,
    const TimestampNs reference_time,
    const DurationNs delta) override
  {
    last_reference_time = reference_time;
    last_delta = delta;
    state.array() += nanoseconds_to_seconds(delta);
    covariance.diagonal().array() += 1.0;
  }

  TimestampNs last_reference_time{};
  DurationNs last_delta{};
};

class RecordingFilter final : public FilterBase
{
public:
  RecordingFilter()
  : FilterBase(1)
  {
  }

  void correct(const Measurement& measurement) override
  {
    corrected_measurement_time = measurement.time_;
  }

  void predict(const TimestampNs reference_time, const DurationNs delta) override
  {
    predicted_reference_time = reference_time;
    predicted_delta = delta;
  }

  TimestampNs corrected_measurement_time{};
  TimestampNs predicted_reference_time{};
  DurationNs predicted_delta{};
};

}  // namespace

TEST(CoreCompatibilityTest, normalizes_covariance_like_the_legacy_filters)
{
  const auto negative = validation::normalize_measurement_covariance(-4.0);
  EXPECT_DOUBLE_EQ(negative.value, 4.0);
  EXPECT_EQ(negative.status, validation::CovarianceStatus::kNegative);

  const auto negative_near_zero =
    validation::normalize_measurement_covariance(-0.5e-9);
  EXPECT_DOUBLE_EQ(
    negative_near_zero.value,
    validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(
    negative_near_zero.status,
    validation::CovarianceStatus::kNegativeAndNearZero);

  const auto near_zero = validation::normalize_measurement_covariance(0.0);
  EXPECT_DOUBLE_EQ(near_zero.value, validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(near_zero.status, validation::CovarianceStatus::kNearZero);

  const auto minimum = validation::normalize_measurement_covariance(
    validation::kMinimumMeasurementCovariance);
  EXPECT_DOUBLE_EQ(minimum.value, validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(minimum.status, validation::CovarianceStatus::kValid);

  const auto nan = validation::normalize_measurement_covariance(
    std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(std::isnan(nan.value));
  EXPECT_EQ(nan.status, validation::CovarianceStatus::kValid);
}

TEST(CoreCompatibilityTest, classifies_measurement_times_like_the_legacy_filter)
{
  EXPECT_EQ(
    validation::classify_measurement_delta(-1),
    validation::MeasurementTimeStatus::kStale);
  EXPECT_EQ(
    validation::classify_measurement_delta(0),
    validation::MeasurementTimeStatus::kCurrent);
  EXPECT_EQ(
    validation::classify_measurement_delta(1),
    validation::MeasurementTimeStatus::kForward);
}

TEST(CoreCompatibilityTest, dispatches_model_prediction_through_the_core_clock)
{
  RecordingModel model;
  ModelBase& model_base = model;

  model_base.predict(42, 1'500'000'000);

  EXPECT_EQ(model.last_reference_time, 42);
  EXPECT_EQ(model.last_delta, 1'500'000'000);
  EXPECT_DOUBLE_EQ(model.get_state()(0), 1.5);
  EXPECT_DOUBLE_EQ(model.get_state()(1), 1.5);
  EXPECT_DOUBLE_EQ(model.get_state_covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(model.get_state_covariance()(1, 1), 1.0);

  StateVector state = StateVector::Zero(2);
  model_base.predict(state, 84, 500'000'000);

  EXPECT_EQ(model.last_reference_time, 84);
  EXPECT_EQ(model.last_delta, 500'000'000);
  EXPECT_DOUBLE_EQ(state(0), 0.5);
  EXPECT_DOUBLE_EQ(state(1), 0.5);
}

TEST(CoreCompatibilityTest, dispatches_filter_operations)
{
  RecordingFilter filter;
  FilterBase& filter_base = filter;
  Measurement measurement;
  measurement.time_ = 123;

  filter_base.correct(measurement);
  filter_base.predict(456, 789);

  EXPECT_EQ(filter.corrected_measurement_time, 123);
  EXPECT_EQ(filter.predicted_reference_time, 456);
  EXPECT_EQ(filter.predicted_delta, 789);
}

TEST(CoreCompatibilityTest, ekf_uses_an_injected_runtime_model)
{
  RecordingModel model(STATE_SIZE);
  Ekf filter(model);

  filter.predict(1'000, 250'000'000);

  EXPECT_EQ(model.last_reference_time, 1'000);
  EXPECT_EQ(model.last_delta, 250'000'000);
  EXPECT_DOUBLE_EQ(model.get_state()(0), 0.25);
  EXPECT_DOUBLE_EQ(model.get_state()(1), 0.25);
}

}  // namespace rpp_localization
