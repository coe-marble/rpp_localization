#include <cmath>
#include <random>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "rpp_localization/inekf/inertial_process.hpp"
#include "rpp_localization/inekf/invariant_ekf.hpp"
#include "rpp_localization/inekf/so3.hpp"

namespace rpp_localization::InEKF
{
namespace
{

constexpr double k_gravity = 9.80665;
constexpr DurationNs k_imu_period = 10'000'000;  // 100 Hz

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Covariance = Eigen::Matrix<double, 15, 15>;

Eigen::Matrix3d yaw_rotation(const double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

InertialLieState make_state(
  const Eigen::Matrix3d& rotation,
  const Eigen::Vector3d& velocity,
  const Eigen::Vector3d& position,
  const Covariance& covariance = Covariance::Zero(),
  const Vector6& biases = Vector6::Zero())
{
  InertialLieState::MatrixState matrix = InertialLieState::MatrixState::Identity();
  matrix.block<3, 3>(0, 0) = rotation;
  matrix.block<3, 1>(0, 3) = velocity;
  matrix.block<3, 1>(0, 4) = position;
  return InertialLieState(matrix, covariance, biases);
}

/// Covariance of a state whose world position and heading are uncertain
/// independently of each other, written in invariant coordinates.
Covariance world_centred_covariance(
  const Eigen::Vector3d& position, const double rotation_variance, const double position_variance)
{
  Covariance covariance = Covariance::Identity() * 1e-10;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() * rotation_variance;
  const Eigen::Matrix3d lever = SO3<>::wedge(position);
  covariance.block<3, 3>(0, 0) = rotation;
  covariance.block<3, 3>(k_inertial_position, 0) = lever * rotation;
  covariance.block<3, 3>(0, k_inertial_position) = rotation * lever.transpose();
  covariance.block<3, 3>(k_inertial_position, k_inertial_position) =
    lever * rotation * lever.transpose() + Eigen::Matrix3d::Identity() * position_variance;
  return covariance;
}

InertialPredictionInput imu_sample(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel)
{
  InertialPredictionInput input;
  input.delta = k_imu_period;
  input.imu << gyro, accel;
  return input;
}

double yaw_of(const InertialLieState& state)
{
  const Eigen::Matrix3d rotation = state.R()();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

void expect_symmetric_positive_definite(const Eigen::MatrixXd& covariance)
{
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_LT((covariance - covariance.transpose()).cwiseAbs().maxCoeff(), 1e-9);
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(InertialProcessTest, level_imu_at_rest_stays_at_rest)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 2.0, 3.0));

  for (int step = 0; step < 500; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, k_gravity)));
  }

  EXPECT_LT((state[0]).norm(), 1e-9);
  EXPECT_LT((state[1] - Eigen::Vector3d(1.0, 2.0, 3.0)).norm(), 1e-9);
  EXPECT_LT((state.R()() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST(InertialProcessTest, constant_specific_force_accelerates_along_the_heading)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    yaw_rotation(M_PI / 2.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  for (int step = 0; step < 200; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, k_gravity)));
  }

  EXPECT_LT((state[0] - Eigen::Vector3d(0.0, 2.0, 0.0)).norm(), 1e-9);
  EXPECT_LT((state[1] - Eigen::Vector3d(0.0, 2.0, 0.0)).norm(), 1e-9);
}

TEST(InertialProcessTest, gyroscope_bias_is_removed_from_the_rate)
{
  InertialProcess process(k_gravity);
  Vector6 biases = Vector6::Zero();
  biases(2) = 0.1;
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Covariance::Zero(), biases);

  for (int step = 0; step < 200; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d(0.0, 0.0, 0.6), Eigen::Vector3d(0.0, 0.0, k_gravity)));
  }

  EXPECT_NEAR(yaw_of(state), 0.5 * 2.0, 1e-9);
}

TEST(InertialProcessTest, accelerometer_bias_is_removed_from_the_specific_force)
{
  InertialProcess process(k_gravity);
  Vector6 biases = Vector6::Zero();
  biases(3) = 0.3;
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Covariance::Zero(), biases);

  for (int step = 0; step < 100; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, k_gravity)));
  }

  EXPECT_LT(state[0].norm(), 1e-9);
}

TEST(InertialProcessTest, covariance_grows_and_stays_positive_definite)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Covariance::Identity() * 1e-4);
  const double initial_trace = state.cov().trace();

  for (int step = 0; step < 300; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, k_gravity)));
  }

  EXPECT_GT(state.cov().trace(), initial_trace);
  expect_symmetric_positive_definite(state.cov());
}

TEST(InertialProcessTest, rotation_stays_orthonormal_over_a_long_run)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // Ten minutes at 200 Hz while tumbling about all three axes.
  auto input = imu_sample(Eigen::Vector3d(0.7, -0.4, 1.1), Eigen::Vector3d(0.0, 0.0, k_gravity));
  input.delta = 5'000'000;
  for (int step = 0; step < 120000; ++step) {
    process.predict(state, input);
  }

  const Eigen::Matrix3d rotation = state.R()();
  EXPECT_LT((rotation * rotation.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_NEAR(rotation.determinant(), 1.0, 1e-12);
}

TEST(InertialProcessTest, zero_duration_leaves_the_state_untouched)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
    Covariance::Identity());
  auto input = imu_sample(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(5.0, 5.0, 5.0));
  input.delta = 0;

  process.predict(state, input);

  EXPECT_EQ(state[0], Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_EQ(state.cov(), Covariance::Identity());
}

TEST(InertialProcessTest, rejects_a_backward_step)
{
  InertialProcess process(k_gravity);
  InertialLieState state;
  auto input = imu_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  input.delta = -1;

  EXPECT_THROW(process.predict(state, input), std::invalid_argument);
}

TEST(InvariantEkfTest, position_measurement_pulls_the_position)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Covariance::Identity()));

  ASSERT_TRUE(
    filter.correct_position(Eigen::Vector3d(2.0, -1.0, 0.5), Eigen::Matrix3d::Identity()));

  // Equal prior and measurement variance: the estimate moves half way.
  EXPECT_LT((filter.state()[1] - Eigen::Vector3d(1.0, -0.5, 0.25)).norm(), 1e-6);
  expect_symmetric_positive_definite(filter.state().cov());
}

TEST(InvariantEkfTest, position_measurement_accounts_for_the_lever_arm)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  const Eigen::Vector3d position(5.0, 3.0, 0.0);
  const Eigen::Vector3d lever_arm(2.0, 0.0, 1.0);
  filter.set_state(
    make_state(
      yaw_rotation(M_PI / 2.0), Eigen::Vector3d::Zero(), position,
      Covariance::Identity() * 0.01));

  ASSERT_TRUE(
    filter.correct_position(
      position + yaw_rotation(M_PI / 2.0) * lever_arm,
      Eigen::Matrix3d::Identity() * 0.01, lever_arm));

  EXPECT_LT((filter.state()[1] - position).norm(), 1e-9);
}

TEST(InvariantEkfTest, body_velocity_measurement_corrects_the_world_velocity)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      yaw_rotation(M_PI / 2.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Covariance::Identity()));

  ASSERT_TRUE(
    filter.correct_body_velocity(
      Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Matrix3d::Identity() * 1e-6));

  EXPECT_LT((filter.state()[0] - Eigen::Vector3d(0.0, 2.0, 0.0)).norm(), 1e-4);
}

TEST(InvariantEkfTest, body_velocity_measurement_removes_the_lever_arm_velocity)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Covariance::Identity() * 1e-6));
  auto input = imu_sample(Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d(0.0, 0.0, k_gravity));
  input.delta = 1;
  filter.predict(input);

  // A sensor 2 m forward on a vessel yawing at 0.5 rad/s also moves sideways.
  ASSERT_TRUE(
    filter.correct_body_velocity(
      Eigen::Vector3d(1.0, 1.0, 0.0), Eigen::Matrix3d::Identity() * 1e-6,
      Eigen::Vector3d(2.0, 0.0, 0.0)));

  EXPECT_LT((filter.state()[0] - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 1e-6);
}

TEST(InvariantEkfTest, attitude_measurement_rotates_towards_the_measurement)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Covariance::Identity() * 0.01));

  ASSERT_TRUE(filter.correct_attitude(yaw_rotation(0.2), Eigen::Matrix3d::Identity() * 0.01));

  EXPECT_NEAR(yaw_of(filter.state()), 0.1, 1e-6);
}

TEST(RotationExponentialTest, small_angle_matches_the_rotation)
{
  // Angles on both sides of the switch to the series expansion.
  for (const double angle : {1e-7, 5e-5, 9.9e-5, 1.1e-4, 1e-2}) {
    const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
    const Eigen::Matrix3d expected = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    const Eigen::Vector3d tangent = angle * axis;
    EXPECT_LT((SO3<>::exp(tangent)() - expected).cwiseAbs().maxCoeff(), 1e-12) << angle;
  }
}

TEST(RotationLogarithmTest, inverts_the_exponential_at_every_angle)
{
  const Eigen::Vector3d axis = Eigen::Vector3d(-0.6, 0.2, 0.7).normalized();
  for (const double angle : {0.0, 1e-9, 1e-5, 9.9e-5, 1.1e-4, 0.1, 1.5, 3.0, M_PI - 1e-6}) {
    const Eigen::Vector3d tangent = angle * axis;
    const Eigen::Vector3d recovered = SO3<>::log(SO3<>::exp(tangent));
    EXPECT_LT((recovered - tangent).norm(), 1e-8) << angle;
  }
}

TEST(RotationLogarithmTest, stays_finite_when_rounding_moves_the_trace_past_three)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation.diagonal().array() += 4e-16;
  const Eigen::Vector3d tangent = SO3<>::log(SO3<>(rotation));
  EXPECT_TRUE(tangent.allFinite());
  EXPECT_LT(tangent.norm(), 1e-9);
}

TEST(InertialGroupTest, logarithm_inverts_the_exponential)
{
  for (const double scale : {1e-7, 5e-5, 2e-4, 0.3, 2.5}) {
    InertialLieState::TangentVector tangent = InertialLieState::TangentVector::Zero();
    tangent.head<9>() << 0.3, -0.5, 0.8, 1.0, -2.0, 0.5, 30.0, -12.0, 4.0;
    tangent.head<3>() = scale * tangent.head<3>().normalized();
    const InertialLieState::TangentVector recovered =
      InertialLieState::log(InertialLieState::exp(tangent));
    EXPECT_LT((recovered - tangent).norm(), 1e-7) << scale;
  }
}

TEST(InertialGroupTest, inverse_composes_to_the_identity)
{
  const auto state = make_state(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.5, -0.8).normalized()).toRotationMatrix(),
    Eigen::Vector3d(1.0, -2.0, 0.3), Eigen::Vector3d(340.0, 290.0, -5.0));
  const InertialLieState::MatrixState product = state() * state.inverse()();
  EXPECT_LT((product - InertialLieState::MatrixState::Identity()).cwiseAbs().maxCoeff(), 1e-10);
}

TEST(InertialGroupTest, adjoint_moves_a_perturbation_across_the_state)
{
  // X exp(xi) X^-1 = exp(Ad_X xi), which the noise propagation relies on.
  const auto state = make_state(
    Eigen::AngleAxisd(-1.1, Eigen::Vector3d(0.4, -0.3, 0.9).normalized()).toRotationMatrix(),
    Eigen::Vector3d(0.6, 0.2, -0.1), Eigen::Vector3d(25.0, -40.0, 3.0));
  InertialLieState::TangentVector tangent = InertialLieState::TangentVector::Zero();
  tangent.head<9>() << 1e-3, -2e-3, 1.5e-3, 2e-3, 1e-3, -1e-3, -3e-3, 2e-3, 1e-3;

  const InertialLieState::MatrixState left =
    state() * InertialLieState::exp(tangent)() * state.inverse()();
  const InertialLieState::TangentVector moved = InertialLieState::Ad(state) * tangent;
  const InertialLieState::MatrixState right = InertialLieState::exp(moved)();

  EXPECT_LT((left - right).cwiseAbs().maxCoeff(), 1e-10);
}

TEST(InvariantEkfTest, attitude_measurement_equal_to_the_estimate_is_accepted)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.7, 0.7).normalized()).toRotationMatrix();
  filter.set_state(
    make_state(rotation, Eigen::Vector3d::Zero(), Eigen::Vector3d(10.0, 5.0, 0.0),
      Covariance::Identity() * 0.01));

  ASSERT_TRUE(filter.correct_attitude(rotation, Eigen::Matrix3d::Identity() * 1e-4));
  EXPECT_LT((filter.state().R()() - rotation).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(InvariantEkfTest, attitude_measurement_half_a_turn_away_is_followed)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Covariance::Identity() * 1.0));

  for (int step = 0; step < 20; ++step) {
    ASSERT_TRUE(filter.correct_attitude(yaw_rotation(3.1), Eigen::Matrix3d::Identity() * 1e-4));
  }
  EXPECT_NEAR(yaw_of(filter.state()), 3.1, 1e-3);
}

TEST(InertialProcessTest, slow_rotation_is_integrated_in_full)
{
  InertialProcess process(k_gravity);
  auto state = make_state(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  // 0.005 rad/s gives steps of 5e-5 rad, below the series switch.
  const double rate = 0.005;
  for (int step = 0; step < 2000; ++step) {
    process.predict(
      state, imu_sample(Eigen::Vector3d(0.0, 0.0, rate), Eigen::Vector3d(0.0, 0.0, k_gravity)));
  }

  EXPECT_NEAR(yaw_of(state), rate * 2000 * 0.01, 1e-9);
}

TEST(InvariantEkfTest, small_attitude_corrections_far_from_the_origin_keep_the_position)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  const Eigen::Vector3d position(340.0, 290.0, -0.5);
  // The position is known in the world and the heading is not, which in
  // invariant coordinates is a rotation error about the vehicle.
  Covariance covariance = Covariance::Identity() * 1e-8;
  const Eigen::Matrix3d rotation_covariance = Eigen::Matrix3d::Identity() * 1e-4;
  const Eigen::Matrix3d lever = SO3<>::wedge(position);
  covariance.block<3, 3>(0, 0) = rotation_covariance;
  covariance.block<3, 3>(k_inertial_position, 0) = lever * rotation_covariance;
  covariance.block<3, 3>(0, k_inertial_position) = rotation_covariance * lever.transpose();
  covariance.block<3, 3>(k_inertial_position, k_inertial_position) +=
    lever * rotation_covariance * lever.transpose();
  filter.set_state(
    make_state(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), position, covariance));

  for (int step = 1; step <= 200; ++step) {
    ASSERT_TRUE(
      filter.correct_attitude(yaw_rotation(2e-5 * step), Eigen::Matrix3d::Identity() * 1e-6));
  }

  EXPECT_GT(yaw_of(filter.state()), 1e-3);
  EXPECT_LT((filter.state()[1] - position).norm(), 1e-3);
}

TEST(InvariantEkfTest, depth_measurement_moves_only_the_height)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  const Eigen::Vector3d position(340.0, 290.0, -1.0);
  filter.set_state(
    make_state(
      yaw_rotation(0.7), Eigen::Vector3d(0.4, 0.1, 0.0), position,
      world_centred_covariance(position, 1e-4, 0.25)));

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance.diagonal() << k_unmeasured_axis_variance, k_unmeasured_axis_variance, 0.01;
  // The horizontal values are far off and must not matter.
  ASSERT_TRUE(filter.correct_position(Eigen::Vector3d(-5000.0, 9000.0, -3.0), covariance));

  const Eigen::Vector3d corrected = filter.state()[1];
  EXPECT_NEAR(corrected.x(), position.x(), 1e-6);
  EXPECT_NEAR(corrected.y(), position.y(), 1e-6);
  EXPECT_NEAR(corrected.z(), -1.0 + (-2.0) * 0.25 / 0.26, 1e-3);
  EXPECT_LT((filter.state().R()() - yaw_rotation(0.7)).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(InvariantEkfTest, horizontal_fix_leaves_the_height)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  const Eigen::Vector3d position(-120.0, 60.0, -4.0);
  filter.set_state(
    make_state(
      yaw_rotation(-1.2), Eigen::Vector3d::Zero(), position,
      world_centred_covariance(position, 1e-4, 1.0)));

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  covariance(2, 2) = k_unmeasured_axis_variance;
  ASSERT_TRUE(filter.correct_position(Eigen::Vector3d(-118.0, 61.0, 777.0), covariance));

  const Eigen::Vector3d corrected = filter.state()[1];
  EXPECT_NEAR(corrected.x(), -119.0, 1e-3);
  EXPECT_NEAR(corrected.y(), 60.5, 1e-3);
  EXPECT_NEAR(corrected.z(), position.z(), 1e-6);
}

TEST(InvariantEkfTest, mahalanobis_gate_rejects_an_outlier)
{
  InertialProcess process(k_gravity);
  InvariantEkf filter(process);
  filter.set_state(
    make_state(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Covariance::Identity() * 0.01));

  EXPECT_FALSE(
    filter.correct_position(
      Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Matrix3d::Identity() * 0.01,
      Eigen::Vector3d::Zero(), 3.0));
  EXPECT_LT(filter.state()[1].norm(), 1e-12);
}

// A vessel circling at constant speed, observed with a biased, noisy IMU and
// noisy position, body-velocity and attitude measurements. The filter starts
// away from the truth and has to find the state and the observable biases.
TEST(InvariantEkfTest, converges_on_a_circle_and_estimates_the_biases)
{
  const double speed = 2.0;
  const double yaw_rate = 0.2;
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.015);
  const Eigen::Vector3d accel_bias(0.05, -0.03, 0.02);
  const double delta = nanoseconds_to_seconds(k_imu_period);

  std::mt19937_64 generator(7);
  std::normal_distribution<double> normal(0.0, 1.0);
  const auto noise = [&](const double standard_deviation) {
      return Eigen::Vector3d(normal(generator), normal(generator), normal(generator)) *
             standard_deviation;
    };

  InertialProcess process(k_gravity);
  InertialProcessNoise process_noise;
  process_noise.gyro.setConstant(2e-3);
  process_noise.accel.setConstant(2e-2);
  process.set_noise(process_noise);
  InvariantEkf filter(process);

  Covariance initial_covariance = Covariance::Zero();
  initial_covariance.diagonal() <<
    0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 4.0, 4.0, 4.0,
    1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2;
  filter.set_state(
    make_state(
      yaw_rotation(0.2), Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, -1.5, 0.5),
      initial_covariance));

  Eigen::Vector3d true_position = Eigen::Vector3d::Zero();
  double true_yaw = 0.0;
  for (int step = 1; step <= 12000; ++step) {
    const double time = step * delta;
    true_yaw = yaw_rate * time;
    true_position = speed / yaw_rate *
      Eigen::Vector3d(std::sin(true_yaw), 1.0 - std::cos(true_yaw), 0.0);
    const Eigen::Matrix3d true_rotation = yaw_rotation(true_yaw);
    const Eigen::Vector3d true_body_velocity(speed, 0.0, 0.0);

    // Mid-step readings of a body turning at constant speed.
    filter.predict(
      imu_sample(
        Eigen::Vector3d(0.0, 0.0, yaw_rate) + gyro_bias + noise(2e-3),
        Eigen::Vector3d(0.0, speed * yaw_rate, k_gravity) + accel_bias + noise(2e-2)));

    if (step % 10 == 0) {
      filter.correct_attitude(
        true_rotation * SO3<>::exp(noise(0.01))(), Eigen::Matrix3d::Identity() * 1e-4);
    }
    if (step % 20 == 0) {
      filter.correct_position(true_position + noise(0.3), Eigen::Matrix3d::Identity() * 0.09);
      filter.correct_body_velocity(
        true_body_velocity + noise(0.02), Eigen::Matrix3d::Identity() * 4e-4);
    }
  }

  const auto& estimate = filter.state();
  EXPECT_LT((estimate[1] - true_position).norm(), 0.3);
  EXPECT_LT(
    std::abs(std::remainder(yaw_of(estimate) - true_yaw, 2.0 * M_PI)), 0.01);
  EXPECT_LT(
    (estimate.R()().transpose() * estimate[0] - Eigen::Vector3d(speed, 0.0, 0.0)).norm(), 0.05);
  EXPECT_LT((estimate.aug().head<3>() - gyro_bias).norm(), 5e-3);
  EXPECT_LT((estimate.aug().tail<3>() - accel_bias).norm(), 2e-2);
  expect_symmetric_positive_definite(estimate.cov());
  expect_symmetric_positive_definite(filter.pose_covariance());
}

}  // namespace
}  // namespace rpp_localization::InEKF
