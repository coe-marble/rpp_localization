#include <cmath>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "gtest/gtest.h"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"

namespace rpp_localization
{
namespace
{

void initialize_model(ConstantAccelerationModel& model)
{
  model.initialize_runtime();
}

Measurement position_measurement(const double covariance)
{
  Measurement measurement;
  measurement.measurement_ = MeasurementVector::Zero(STATE_SIZE);
  measurement.measurement_(StateMemberX) = 1.0;
  measurement.covariance_ = CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE);
  measurement.covariance_(StateMemberX, StateMemberX) = covariance;
  measurement.update_vector_ = std::vector<bool>(STATE_SIZE, false);
  measurement.update_vector_[StateMemberX] = true;
  return measurement;
}

}  // namespace

TEST(ConstantAccelerationModelParity, PreservesStateAndCovariancePrediction)
{
  ConstantAccelerationModel model(STATE_SIZE);
  initialize_model(model);

  CovarianceMatrix process_noise = CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE);
  process_noise(StateMemberX, StateMemberX) = 0.25;
  model.set_process_noise_covariance(process_noise);

  StateVector state = StateVector::Zero(STATE_SIZE);
  state(StateMemberVx) = 2.0;
  state(StateMemberAx) = 1.0;
  CovarianceMatrix covariance = CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE);

  model.predict(state, covariance, 1'000'000'000, 1'000'000'000);

  EXPECT_DOUBLE_EQ(state(StateMemberX), 2.5);
  EXPECT_DOUBLE_EQ(state(StateMemberVx), 3.0);
  EXPECT_DOUBLE_EQ(covariance(StateMemberX, StateMemberX), 0.25);
}

TEST(ConstantAccelerationModelParity, PreservesControlAndTimeoutHandling)
{
  ConstantAccelerationModel model(STATE_SIZE);
  initialize_model(model);
  model.set_process_noise_covariance(CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE));

  std::vector<bool> update_vector(TWIST_SIZE, false);
  update_vector[ControlMemberVx] = true;
  std::vector<double> limits(TWIST_SIZE, 0.0);
  limits[ControlMemberVx] = 1.0;
  std::vector<double> gains(TWIST_SIZE, 0.0);
  gains[ControlMemberVx] = 1.0;
  model.set_control_params(
    update_vector,
    static_cast<DurationNs>(1'000'000'000),
    limits,
    gains,
    limits,
    gains);

  ControlCommand control;
  control.stamp = 1'000'000'000;
  control.control = ControlVector::Zero(TWIST_SIZE);
  control.control(ControlMemberVx) = 2.0;
  model.set_control(control);

  StateVector state = StateVector::Zero(STATE_SIZE);
  model.set_state(state);
  CovarianceMatrix covariance = CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE);
  model.predict(state, covariance, 1'100'000'000, 1'000'000'000);

  EXPECT_DOUBLE_EQ(state(StateMemberX), 0.5);
  EXPECT_DOUBLE_EQ(state(StateMemberVx), 1.0);
  EXPECT_DOUBLE_EQ(state(StateMemberAx), 1.0);

  state.setZero();
  covariance.setZero();
  model.set_state(state);
  model.predict(state, covariance, 3'000'000'000, 1'000'000'000);

  EXPECT_DOUBLE_EQ(state(StateMemberX), 0.0);
  EXPECT_DOUBLE_EQ(state(StateMemberVx), 0.0);
  EXPECT_DOUBLE_EQ(state(StateMemberAx), 0.0);
}

TEST(ConstantAccelerationModelParity, PreservesNearPitchSingularityPrediction)
{
  ConstantAccelerationModel model(STATE_SIZE);
  initialize_model(model);
  model.set_computes_covariance(false);

  StateVector state = StateVector::Zero(STATE_SIZE);
  const double pitch = PI / 2.0 - 1e-6;
  const double delta_seconds = 0.1;
  state(StateMemberPitch) = pitch;
  state(StateMemberVyaw) = 1.0;
  CovarianceMatrix covariance = CovarianceMatrix::Zero(STATE_SIZE, STATE_SIZE);

  model.predict(
    state,
    covariance,
    1'000'000'000,
    static_cast<DurationNs>(delta_seconds * 1e9));

  const double expected_yaw = angles::normalize_angle(delta_seconds / std::cos(pitch));
  EXPECT_TRUE(std::isfinite(state(StateMemberYaw)));
  EXPECT_NEAR(state(StateMemberYaw), expected_yaw, 1e-9);
}

TEST(ConstantAccelerationModelParity, NormalizesCorrectionCovarianceLikeTheLegacyEkf)
{
  ConstantAccelerationModel model(STATE_SIZE);
  initialize_model(model);
  Ekf filter(model);

  model.set_state(StateVector::Zero(STATE_SIZE));
  model.set_state_covariance(CovarianceMatrix::Identity(STATE_SIZE, STATE_SIZE));
  filter.correct(position_measurement(-4.0));

  EXPECT_DOUBLE_EQ(model.get_state()(StateMemberX), 0.2);
  EXPECT_DOUBLE_EQ(model.get_state_covariance()(StateMemberX, StateMemberX), 0.8);

  model.set_state(StateVector::Zero(STATE_SIZE));
  model.set_state_covariance(CovarianceMatrix::Identity(STATE_SIZE, STATE_SIZE));
  filter.correct(position_measurement(-0.5e-9));

  EXPECT_NEAR(model.get_state()(StateMemberX), 1.0, 1e-8);
  EXPECT_NEAR(
    model.get_state_covariance()(StateMemberX, StateMemberX),
    1e-9,
    1e-12);
}

}  // namespace rpp_localization
