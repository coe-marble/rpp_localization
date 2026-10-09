#include "rpp_localization/inekf/inertial_process.hpp"

#include <cmath>
#include <stdexcept>

#include "rpp_localization/inekf/so3.hpp"

namespace rpp_localization::InEKF
{

InertialProcess::InertialProcess(const double gravity)
: gravity_(0.0, 0.0, -gravity)
{
  if (!std::isfinite(gravity) || gravity < 0.0) {
    throw std::invalid_argument("gravity cannot be negative");
  }
  set_noise(InertialProcessNoise{});
}

void InertialProcess::set_noise(const InertialProcessNoise& noise)
{
  Eigen::Matrix<double, 15, 1> standard_deviation;
  standard_deviation <<
    noise.gyro, noise.accel, Eigen::Vector3d::Zero(), noise.gyro_bias, noise.accel_bias;
  if (!standard_deviation.allFinite() || (standard_deviation.array() < 0.0).any()) {
    throw std::invalid_argument("inertial process noise must be finite and non-negative");
  }
  noise_ = standard_deviation.array().square().matrix().asDiagonal();
}

void InertialProcess::predict(
  InertialLieState& state,
  const InertialPredictionInput& input)
{
  if (input.delta < 0) {
    throw std::invalid_argument("inertial prediction cannot step backwards");
  }
  if (!input.imu.allFinite()) {
    throw std::invalid_argument("IMU sample must be finite");
  }
  const double delta = nanoseconds_to_seconds(input.delta);
  if (delta == 0.0) {
    return;
  }

  // The covariance is propagated about the state before the step.
  const Covariance transition_matrix = transition(state, delta);
  const Covariance adjoint = InertialLieState::Ad(state);
  const Covariance propagated =
    transition_matrix *
    (state.cov() + adjoint * noise_ * adjoint.transpose() * delta) *
    transition_matrix.transpose();

  const Eigen::Vector3d angular_velocity = input.imu.head<3>() - state.aug().head<3>();
  const Eigen::Vector3d specific_force = input.imu.tail<3>() - state.aug().tail<3>();
  const Eigen::Matrix3d rotation = state.R()();
  const Eigen::Vector3d velocity = state[0];
  const Eigen::Vector3d position = state[1];
  const Eigen::Vector3d acceleration = rotation * specific_force + gravity_;

  InertialLieState::MatrixState next = InertialLieState::MatrixState::Identity();
  next.block<3, 3>(0, 0) = rotation * SO3<>::exp(angular_velocity * delta)();
  next.block<3, 1>(0, 3) = velocity + acceleration * delta;
  next.block<3, 1>(0, 4) = position + velocity * delta + 0.5 * acceleration * delta * delta;

  state.setMat(next);
  state.setCov(0.5 * (propagated + propagated.transpose()));
}

InertialProcess::Covariance InertialProcess::transition(
  const InertialLieState& state,
  const double delta) const
{
  // Right-invariant error dynamics. The group part does not depend on the
  // state; only the coupling to the biases does.
  const Eigen::Matrix3d rotation = state.R()();
  Covariance dynamics = Covariance::Zero();
  dynamics.block<3, 3>(k_inertial_velocity, k_inertial_rotation) = SO3<>::wedge(gravity_);
  dynamics.block<3, 3>(k_inertial_position, k_inertial_velocity) = Eigen::Matrix3d::Identity();
  dynamics.block<3, 3>(k_inertial_rotation, k_inertial_gyro_bias) = -rotation;
  dynamics.block<3, 3>(k_inertial_velocity, k_inertial_gyro_bias) =
    -SO3<>::wedge(state[0]) * rotation;
  dynamics.block<3, 3>(k_inertial_position, k_inertial_gyro_bias) =
    -SO3<>::wedge(state[1]) * rotation;
  dynamics.block<3, 3>(k_inertial_velocity, k_inertial_accel_bias) = -rotation;

  const Covariance step = dynamics * delta;
  return Covariance::Identity() + step + step * step / 2.0 + step * step * step / 6.0;
}

}  // namespace rpp_localization::InEKF
