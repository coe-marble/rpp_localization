#include <gtest/gtest.h>

#include <rpp_localization/filters/extended_kalman_filter.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "filter15_plugin_base.hpp"
#include "nav_model15_adapter.hpp"

namespace
{

constexpr std::size_t k_state_size = rpp_localization::STATE_SIZE;
constexpr std::size_t k_control_size = rpp_localization::TWIST_SIZE;
constexpr std::size_t k_covariance_size = k_state_size * k_state_size;

void write_state(
  rpp_localization::NavModel15::LocalizationModelPredictOutput15& output,
  const double offset)
{
  auto values = output.state().values();
  values.resize(k_state_size);
  for (std::size_t index = 0; index < k_state_size; ++index)
  {
    values[index] = offset + static_cast<double>(index);
  }
}

void write_covariance(
  rpp_localization::NavModel15::LocalizationModelPredictOutput15& output,
  const double diagonal)
{
  auto values = output.covariance().values();
  values.resize(k_covariance_size);
  for (std::size_t row = 0; row < k_state_size; ++row)
  {
    for (std::size_t column = 0; column < k_state_size; ++column)
    {
      values[row * k_state_size + column] = row == column ? diagonal : 0.0;
    }
  }
}

class FakeNavModel15 final : public rpp_localization::NavModel15
{
public:
  enum class OutputMode
  {
    k_valid,
    k_model_failure,
    k_invalid_covariance
  };

  void initialize(const rpp::ComponentContext&) override {}

  LocalizationModelPredictOutput15::Const predict(LocalizationModelPredictInput15::Const input) override
  {
    reference_time_ = input.referenceTimeNs();
    delta_ = input.deltaNs();
    const auto input_control = input.control();
    control_present_ = input_control.present();
    control_stamp_ = input_control.stampNs();
    const auto values = input_control.values();
    const auto enabled = input_control.enabled();
    control_values_.resize(values.size());
    control_enabled_.resize(enabled.size());
    for (std::size_t index = 0; index < values.size(); ++index)
    {
      control_values_[index] = values[index];
      control_enabled_[index] = enabled[index];
    }

    LocalizationModelPredictOutput15 output;
    if (mode_ == OutputMode::k_model_failure)
    {
      output.status().code() = 7;
      output.status().message() = "fake model failure";
      return output;
    }

    write_state(output, 1.0);
    write_covariance(output, mode_ == OutputMode::k_invalid_covariance ? -1.0 : 2.0);
    output.status().code() = 0;
    return output;
  }

  OutputMode mode_{OutputMode::k_valid};
  rpp_localization::TimestampNs reference_time_{0};
  rpp_localization::DurationNs delta_{0};
  bool control_present_{false};
  rpp_localization::TimestampNs control_stamp_{0};
  std::vector<double> control_values_;
  std::vector<bool> control_enabled_;
};

class BoundaryEkf15 final : public Filter15PluginBase
{
public:
  void initialize_with(std::shared_ptr<rpp_localization::NavModel15> model)
  {
    initialize_with_model(std::move(model));
  }

protected:
  std::unique_ptr<rpp_localization::FilterBase> create_filter(
    rpp_localization::ModelBase& model) override
  {
    return std::make_unique<rpp_localization::Ekf>(model);
  }
};

rpp_localization::LocalizationFilter15::Estimate15 make_estimate(
  const rpp_localization::TimestampNs reference_time)
{
  rpp_localization::LocalizationFilter15::Estimate15 estimate;
  auto state = estimate.state().values();
  state.resize(k_state_size);
  auto covariance = estimate.covariance().values();
  covariance.resize(k_covariance_size);
  for (std::size_t row = 0; row < k_state_size; ++row)
  {
    state[row] = 0.0;
    for (std::size_t column = 0; column < k_state_size; ++column)
    {
      covariance[row * k_state_size + column] = row == column ? 1.0 : 0.0;
    }
  }
  estimate.referenceTimeNs() = reference_time;
  return estimate;
}

rpp_localization::LocalizationFilter15::Measurement15 make_measurement(
  const rpp_localization::TimestampNs reference_time,
  const double x,
  const double covariance_x)
{
  rpp_localization::LocalizationFilter15::Measurement15 measurement;
  auto state = measurement.state().values();
  state.resize(k_state_size);
  auto covariance = measurement.covariance().values();
  covariance.resize(k_covariance_size);
  auto update_mask = measurement.updateMask();
  update_mask.resize(k_state_size);
  for (std::size_t row = 0; row < k_state_size; ++row)
  {
    state[row] = row == 0 ? x : 0.0;
    update_mask[row] = row == 0;
    for (std::size_t column = 0; column < k_state_size; ++column)
    {
      covariance[row * k_state_size + column] = row == column ? 1.0 : 0.0;
    }
  }
  covariance[0] = covariance_x;
  measurement.referenceTimeNs() = reference_time;
  measurement.mahalanobisThreshold() = std::numeric_limits<double>::infinity();
  measurement.sourceName() = "boundary-test";
  return measurement;
}

TEST(NavModel15Adapter, ForwardsCanonicalStateCovarianceControlAndTime)
{
  auto model = std::make_shared<FakeNavModel15>();
  rpp_localization::plugins::detail::NavModel15Adapter adapter(model);
  rpp_localization::StateVector state(k_state_size);
  rpp_localization::CovarianceMatrix covariance =
    rpp_localization::CovarianceMatrix::Identity(k_state_size, k_state_size);
  for (std::size_t index = 0; index < k_state_size; ++index)
  {
    state[static_cast<Eigen::Index>(index)] = static_cast<double>(index);
  }

  rpp_schema::rpp_localization::Control6 control;
  control.present() = true;
  control.stampNs() = 19;
  auto control_values = control.values();
  auto control_enabled = control.enabled();
  control_values.resize(k_control_size);
  control_enabled.resize(k_control_size);
  for (std::size_t index = 0; index < k_control_size; ++index)
  {
    control_values[index] = static_cast<double>(index) + 0.5;
    control_enabled[index] = index % 2 == 0;
  }

  adapter.set_control(control);
  adapter.predict(state, covariance, 41, 17);

  EXPECT_EQ(model->reference_time_, 41);
  EXPECT_EQ(model->delta_, 17);
  EXPECT_TRUE(model->control_present_);
  EXPECT_EQ(model->control_stamp_, 19);
  EXPECT_EQ(model->control_values_.size(), k_control_size);
  EXPECT_EQ(model->control_enabled_.size(), k_control_size);
  EXPECT_DOUBLE_EQ(state[0], 1.0);
  EXPECT_DOUBLE_EQ(state[14], 15.0);
  EXPECT_DOUBLE_EQ(covariance(0, 0), 2.0);
}

TEST(NavModel15Adapter, RejectsModelStatusAndInvalidCovariance)
{
  auto model = std::make_shared<FakeNavModel15>();
  rpp_localization::plugins::detail::NavModel15Adapter adapter(model);
  rpp_localization::StateVector state = rpp_localization::StateVector::Zero(k_state_size);
  rpp_localization::CovarianceMatrix covariance =
    rpp_localization::CovarianceMatrix::Identity(k_state_size, k_state_size);

  model->mode_ = FakeNavModel15::OutputMode::k_model_failure;
  try
  {
    adapter.predict(state, covariance, 0, 1);
    FAIL() << "expected ModelPredictionError";
  }
  catch (const rpp_localization::plugins::detail::ModelPredictionError& error)
  {
    EXPECT_EQ(error.code(), 7);
  }

  model->mode_ = FakeNavModel15::OutputMode::k_invalid_covariance;
  try
  {
    adapter.predict(state, covariance, 0, 1);
    FAIL() << "expected ModelPredictionError";
  }
  catch (const rpp_localization::plugins::detail::ModelPredictionError& error)
  {
    EXPECT_EQ(error.code(), 9);
  }
}

TEST(LocalizationFilter15PluginBoundary, PreservesStaleTimeAndCovarianceNormalization)
{
  auto model = std::make_shared<FakeNavModel15>();
  BoundaryEkf15 filter;
  filter.initialize_with(model);

  auto initial = make_estimate(100);
  const auto initialized = filter.initialize(std::move(initial));
  ASSERT_EQ(initialized.code(), 0);

  rpp_localization::LocalizationFilter15::LocalizationPredictInput15 prediction;
  prediction.referenceTimeNs() = 110;
  prediction.deltaNs() = 10;
  prediction.control().present() = false;
  const auto predicted = filter.predict(std::move(prediction));
  ASSERT_EQ(predicted.status().code(), 0);
  ASSERT_EQ(predicted.estimate().referenceTimeNs(), 110);

  auto stale = make_measurement(109, 2.0, 1.0);
  const auto stale_result = filter.correct(std::move(stale));
  EXPECT_EQ(stale_result.status().code(), 0);
  EXPECT_EQ(stale_result.estimate().referenceTimeNs(), 110);

  auto future = make_measurement(111, 2.0, 1.0);
  const auto future_result = filter.correct(std::move(future));
  EXPECT_EQ(future_result.status().code(), 2);

  auto current = make_measurement(110, 2.0, -0.25);
  const auto corrected = filter.correct(std::move(current));
  EXPECT_EQ(corrected.status().code(), 0);
  EXPECT_GT(corrected.estimate().state().values()[0], 1.0);
  EXPECT_LT(corrected.estimate().state().values()[0], 2.0);
}

}  // namespace
