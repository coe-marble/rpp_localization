#include <cmath>
#include <vector>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/filters/unscented_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"

namespace rpp_localization
{
namespace
{

constexpr double k_pi = 3.14159265358979323846;

Eigen::MatrixXd process_noise()
{
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  covariance.diagonal().setConstant(0.01);
  return covariance;
}

// Sigma points one standard deviation wide, so angle differences are not
// hidden by the default 0.001 spread.
struct UkfUnderTest
{
  UkfUnderTest(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance)
  : model(STATE_SIZE),
    filter(model)
  {
    model.initialize_runtime();
    model.set_process_noise_covariance(process_noise());
    filter.initialize_runtime(process_noise(), false, 1.0, 0.0, 2.0);
    model.set_state(state);
    model.set_state_covariance(covariance);
  }

  ConstantAccelerationModel model;
  Ukf filter;
};

struct EkfUnderTest
{
  EkfUnderTest(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance)
  : model(STATE_SIZE),
    filter(model)
  {
    model.initialize_runtime();
    model.set_process_noise_covariance(process_noise());
    model.set_state(state);
    model.set_state_covariance(covariance);
  }

  ConstantAccelerationModel model;
  Ekf filter;
};

Measurement measure(
  const std::vector<StateMembers>& members,
  const std::vector<double>& values,
  const double variance)
{
  Measurement measurement;
  measurement.topic_name_ = "test";
  measurement.update_vector_.assign(STATE_SIZE, false);
  measurement.measurement_ = Eigen::VectorXd::Zero(STATE_SIZE);
  measurement.covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  for (std::size_t index = 0; index < members.size(); ++index) {
    measurement.update_vector_[members[index]] = true;
    measurement.measurement_(members[index]) = values[index];
    measurement.covariance_(members[index], members[index]) = variance;
  }
  return measurement;
}

Eigen::MatrixXd diagonal_covariance(const double variance)
{
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  covariance.diagonal().setConstant(variance);
  return covariance;
}

void expect_symmetric_positive_definite(const Eigen::MatrixXd& covariance)
{
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_LT((covariance - covariance.transpose()).cwiseAbs().maxCoeff(), 1e-9);
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(UkfTest, correction_of_a_linear_measurement_matches_the_ekf)
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberX) = 2.0;
  state(StateMemberYaw) = 0.3;
  state(StateMemberVx) = 1.0;
  const Eigen::MatrixXd covariance = diagonal_covariance(0.5);
  UkfUnderTest ukf(state, covariance);
  EkfUnderTest ekf(state, covariance);
  const Measurement measurement = measure(
    {StateMemberX, StateMemberY, StateMemberVx}, {2.4, -0.2, 1.3}, 0.1);

  ukf.filter.correct(measurement);
  ekf.filter.correct(measurement);

  EXPECT_LT((ukf.model.get_state() - ekf.model.get_state()).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT(
    (ukf.model.get_state_covariance() - ekf.model.get_state_covariance())
    .cwiseAbs().maxCoeff(), 1e-9);
  expect_symmetric_positive_definite(ukf.model.get_state_covariance());
}

// A yaw-only measurement must correct a position that is correlated with yaw
// exactly as a Kalman update does. The position difference of a sigma point
// exceeds pi here, so wrapping it as if it were an angle corrupts the gain.
TEST(UkfTest, yaw_measurement_corrects_correlated_position_like_the_ekf)
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberYaw) = 0.2;
  Eigen::MatrixXd covariance = diagonal_covariance(0.05);
  covariance(StateMemberX, StateMemberX) = 4.0;
  covariance(StateMemberX, StateMemberYaw) = 0.3;
  covariance(StateMemberYaw, StateMemberX) = 0.3;
  UkfUnderTest ukf(state, covariance);
  EkfUnderTest ekf(state, covariance);
  const Measurement measurement = measure({StateMemberYaw}, {0.5}, 0.01);

  ukf.filter.correct(measurement);
  ekf.filter.correct(measurement);

  EXPECT_NEAR(
    ukf.model.get_state()(StateMemberX), ekf.model.get_state()(StateMemberX), 1e-9);
  EXPECT_NEAR(
    ukf.model.get_state()(StateMemberYaw), ekf.model.get_state()(StateMemberYaw), 1e-9);
  EXPECT_LT(
    (ukf.model.get_state_covariance() - ekf.model.get_state_covariance())
    .cwiseAbs().maxCoeff(), 1e-9);
}

// After a prediction the model wraps each sigma point, so near +-pi some of
// them sit on the other side of the branch cut from the mean.
TEST(UkfTest, yaw_correction_is_continuous_across_the_branch_cut)
{
  const double yaw_offset = 0.05;
  const auto corrected_yaw_error = [&](const double yaw) {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
      state(StateMemberYaw) = yaw;
      UkfUnderTest ukf(state, diagonal_covariance(0.05));
      ukf.filter.predict(0, 100'000'000);
      ukf.filter.correct(measure({StateMemberYaw}, {yaw + yaw_offset}, 0.01));
      expect_symmetric_positive_definite(ukf.model.get_state_covariance());
      return std::remainder(ukf.model.get_state()(StateMemberYaw) - yaw, 2.0 * k_pi);
    };

  const double reference = corrected_yaw_error(0.3);

  EXPECT_GT(reference, 0.0);
  EXPECT_LT(reference, yaw_offset);
  EXPECT_NEAR(corrected_yaw_error(k_pi - 0.01), reference, 1e-6);
  EXPECT_NEAR(corrected_yaw_error(-k_pi + 0.01), reference, 1e-6);
}

TEST(UkfTest, prediction_of_a_stationary_state_matches_the_ekf)
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberX) = 1.0;
  state(StateMemberYaw) = 0.4;
  const Eigen::MatrixXd covariance = diagonal_covariance(0.02);
  UkfUnderTest ukf(state, covariance);
  EkfUnderTest ekf(state, covariance);

  ukf.filter.predict(0, 100'000'000);
  ekf.filter.predict(0, 100'000'000);

  EXPECT_LT((ukf.model.get_state() - ekf.model.get_state()).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LT(
    (ukf.model.get_state_covariance() - ekf.model.get_state_covariance())
    .cwiseAbs().maxCoeff(), 1e-3);
  expect_symmetric_positive_definite(ukf.model.get_state_covariance());
}

TEST(UkfTest, prediction_moves_the_position_along_the_heading)
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberYaw) = k_pi / 2.0;
  state(StateMemberVx) = 2.0;
  UkfUnderTest ukf(state, diagonal_covariance(1e-6));

  ukf.filter.predict(0, 500'000'000);

  EXPECT_NEAR(ukf.model.get_state()(StateMemberX), 0.0, 1e-4);
  EXPECT_NEAR(ukf.model.get_state()(StateMemberY), 1.0, 1e-4);
}

TEST(UkfTest, prediction_keeps_an_angle_that_nothing_measures)
{
  // The default sigma spread, which makes the central weight about -1e6.
  ConstantAccelerationModel model(STATE_SIZE);
  Ukf filter(model);
  model.initialize_runtime();
  model.set_process_noise_covariance(process_noise());
  filter.initialize_runtime(process_noise(), false, 0.001, 0.0, 2.0);
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberRoll) = -3.1;
  Eigen::MatrixXd covariance = diagonal_covariance(0.01);
  covariance(StateMemberRoll, StateMemberRoll) = 40.0;
  model.set_state(state);
  model.set_state_covariance(covariance);

  filter.predict(0, 33'000'000);

  EXPECT_NEAR(model.get_state()(StateMemberRoll), -3.1, 1e-2);
  EXPECT_GT(model.get_state_covariance()(StateMemberRoll, StateMemberRoll), 39.0);
  expect_symmetric_positive_definite(model.get_state_covariance());
}

TEST(UkfTest, correction_leaves_states_without_uncertainty_untouched)
{
  // The estimate right after start-up: only the measured states have a
  // variance, and the covariance is singular.
  ConstantAccelerationModel model(STATE_SIZE);
  Ukf filter(model);
  model.initialize_runtime();
  model.set_process_noise_covariance(process_noise());
  filter.initialize_runtime(process_noise(), false, 0.001, 0.0, 2.0);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  covariance(StateMemberYaw, StateMemberYaw) = 0.0076;
  model.set_state(Eigen::VectorXd::Zero(STATE_SIZE));
  model.set_state_covariance(covariance);

  filter.correct(measure({StateMemberVyaw}, {0.1}, 0.0019));

  EXPECT_TRUE(model.get_state().allFinite());
  EXPECT_GE(model.get_state_covariance().diagonal().minCoeff(), 0.0);
  EXPECT_DOUBLE_EQ(model.get_state()(StateMemberVyaw), 0.0);
}

TEST(UkfTest, prediction_survives_a_slightly_indefinite_covariance)
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(STATE_SIZE);
  state(StateMemberVx) = 1.0;
  // What rounding in the covariance update leaves behind on a state that
  // nothing measures: a small negative eigenvalue.
  Eigen::MatrixXd covariance = diagonal_covariance(0.01);
  covariance(StateMemberYaw, StateMemberYaw) = -1e-9;
  UkfUnderTest ukf(state, covariance);

  ukf.filter.predict(0, 100'000'000);

  EXPECT_TRUE(ukf.model.get_state().allFinite());
  EXPECT_TRUE(ukf.model.get_state_covariance().allFinite());
  EXPECT_NEAR(ukf.model.get_state()(StateMemberX), 0.1, 1e-3);
}

}  // namespace
}  // namespace rpp_localization
