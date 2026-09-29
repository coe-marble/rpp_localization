#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/core/validation.hpp"

namespace rpp_localization
{
namespace
{

class RecordingModel final : public ModelBase
{
public:
  RecordingModel()
  : ModelBase(2)
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
    state.array() += nanosecondsToSeconds(delta);
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

TEST(CoreCompatibilityTest, NormalizesCovarianceLikeTheLegacyFilters)
{
  const auto negative = validation::normalizeMeasurementCovariance(-4.0);
  EXPECT_DOUBLE_EQ(negative.value, 4.0);
  EXPECT_EQ(negative.status, validation::CovarianceStatus::kNegative);

  const auto negative_near_zero =
    validation::normalizeMeasurementCovariance(-0.5e-9);
  EXPECT_DOUBLE_EQ(
    negative_near_zero.value,
    validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(
    negative_near_zero.status,
    validation::CovarianceStatus::kNegativeAndNearZero);

  const auto near_zero = validation::normalizeMeasurementCovariance(0.0);
  EXPECT_DOUBLE_EQ(near_zero.value, validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(near_zero.status, validation::CovarianceStatus::kNearZero);

  const auto minimum = validation::normalizeMeasurementCovariance(
    validation::kMinimumMeasurementCovariance);
  EXPECT_DOUBLE_EQ(minimum.value, validation::kMinimumMeasurementCovariance);
  EXPECT_EQ(minimum.status, validation::CovarianceStatus::kValid);

  const auto nan = validation::normalizeMeasurementCovariance(
    std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(std::isnan(nan.value));
  EXPECT_EQ(nan.status, validation::CovarianceStatus::kValid);
}

TEST(CoreCompatibilityTest, ClassifiesMeasurementTimesLikeTheLegacyFilter)
{
  EXPECT_EQ(
    validation::classifyMeasurementDelta(-1),
    validation::MeasurementTimeStatus::kStale);
  EXPECT_EQ(
    validation::classifyMeasurementDelta(0),
    validation::MeasurementTimeStatus::kCurrent);
  EXPECT_EQ(
    validation::classifyMeasurementDelta(1),
    validation::MeasurementTimeStatus::kForward);
}

TEST(CoreCompatibilityTest, DispatchesModelPredictionThroughTheCoreClock)
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

TEST(CoreCompatibilityTest, DispatchesFilterRuntimeOperations)
{
  RecordingFilter filter;
  RuntimeFilter& runtime_filter = filter;
  Measurement measurement;
  measurement.time_ = 123;

  runtime_filter.correct(measurement);
  runtime_filter.predict(456, 789);

  EXPECT_EQ(filter.corrected_measurement_time, 123);
  EXPECT_EQ(filter.predicted_reference_time, 456);
  EXPECT_EQ(filter.predicted_delta, 789);
}

}  // namespace rpp_localization
