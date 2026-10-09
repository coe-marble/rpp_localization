#include "rpp_localization/inekf/invariant_ekf.hpp"

#include <cmath>
#include <stdexcept>

#include "rpp_localization/inekf/so3.hpp"

namespace rpp_localization::InEKF
{
namespace
{

void require_covariance(const Eigen::Matrix3d& covariance)
{
  if (!covariance.allFinite() || (covariance.diagonal().array() < 0.0).any()) {
    throw std::invalid_argument("measurement covariance must be finite and non-negative");
  }
}

}  // namespace

InvariantEkf::InvariantEkf(InertialPredictionModel& model)
: model_(model)
{
}

void InvariantEkf::set_state(const InertialLieState& state)
{
  if (!state().allFinite() || !state.cov().allFinite() || !state.aug().allFinite()) {
    throw std::invalid_argument("inertial state must be finite");
  }
  state_ = state;
}

void InvariantEkf::predict(const InertialPredictionInput& input)
{
  angular_velocity_ = input.imu.head<3>() - state_.aug().head<3>();
  model_.predict(state_, input);
}

bool InvariantEkf::correct_position(
  const Eigen::Vector3d& position,
  const Eigen::Matrix3d& covariance,
  const Eigen::Vector3d& lever_arm,
  const double mahalanobis_threshold)
{
  require_covariance(covariance);
  const Eigen::Vector3d predicted = state_[1] + state_.R()() * lever_arm;

  Jacobian jacobian = Jacobian::Zero();
  jacobian.block<3, 3>(0, k_inertial_rotation) = -SO3<>::wedge(predicted);
  jacobian.block<3, 3>(0, k_inertial_position) = Eigen::Matrix3d::Identity();
  return correct(position - predicted, jacobian, covariance, mahalanobis_threshold);
}

bool InvariantEkf::correct_body_velocity(
  const Eigen::Vector3d& velocity,
  const Eigen::Matrix3d& covariance,
  const Eigen::Vector3d& lever_arm,
  const double mahalanobis_threshold)
{
  require_covariance(covariance);
  // Rotated into the world frame, the innovation is the velocity error
  // itself, whatever the attitude error is.
  const Eigen::Matrix3d rotation = state_.R()();
  const Eigen::Vector3d at_origin = velocity - angular_velocity_.cross(lever_arm);

  Jacobian jacobian = Jacobian::Zero();
  jacobian.block<3, 3>(0, k_inertial_velocity) = Eigen::Matrix3d::Identity();
  return correct(
    rotation * at_origin - state_[0], jacobian,
    rotation * covariance * rotation.transpose(), mahalanobis_threshold);
}

bool InvariantEkf::correct_attitude(
  const Eigen::Matrix3d& rotation,
  const Eigen::Matrix3d& covariance,
  const double mahalanobis_threshold)
{
  require_covariance(covariance);
  const Eigen::Matrix3d estimate = state_.R()();
  const Eigen::Matrix3d error = rotation * estimate.transpose();

  Jacobian jacobian = Jacobian::Zero();
  jacobian.block<3, 3>(0, k_inertial_rotation) = Eigen::Matrix3d::Identity();
  return correct(
    SO3<>::log(SO3<>(error)), jacobian,
    estimate * covariance * estimate.transpose(), mahalanobis_threshold);
}

bool InvariantEkf::correct(
  const Eigen::Vector3d& innovation,
  const Jacobian& jacobian,
  const Eigen::Matrix3d& covariance,
  const double mahalanobis_threshold)
{
  if (!innovation.allFinite()) {
    throw std::invalid_argument("measurement must be finite");
  }
  const Covariance prior = state_.cov();
  // A measurement reported without noise would make the gain singular.
  const Eigen::Matrix3d noise = covariance + 1e-9 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d innovation_covariance =
    jacobian * prior * jacobian.transpose() + noise;
  const Eigen::Matrix3d innovation_information = innovation_covariance.inverse();

  const double squared_distance = innovation.dot(innovation_information * innovation);
  if (squared_distance > mahalanobis_threshold * mahalanobis_threshold) {
    return false;
  }

  const Eigen::Matrix<double, 15, 3> gain =
    prior * jacobian.transpose() * innovation_information;
  const Eigen::Matrix<double, 15, 1> correction = gain * innovation;

  InertialLieState::TangentVector group_correction = InertialLieState::TangentVector::Zero();
  group_correction.head<9>() = correction.head<9>();
  state_.setMat(InertialLieState::exp(group_correction)() * state_());
  state_.setAug(state_.aug() + correction.tail<6>());

  const Covariance residual = Covariance::Identity() - gain * jacobian;
  const Covariance posterior =
    residual * prior * residual.transpose() + gain * noise * gain.transpose();
  state_.setCov(0.5 * (posterior + posterior.transpose()));
  return true;
}

Eigen::Matrix<double, 6, 6> InvariantEkf::pose_covariance() const
{
  // World position error is the position part of the invariant error minus
  // the lever effect of the rotation error on the position itself.
  Eigen::Matrix<double, 6, 15> map = Eigen::Matrix<double, 6, 15>::Zero();
  map.block<3, 3>(0, k_inertial_position) = Eigen::Matrix3d::Identity();
  map.block<3, 3>(0, k_inertial_rotation) = -SO3<>::wedge(state_[1]);
  map.block<3, 3>(3, k_inertial_rotation) = Eigen::Matrix3d::Identity();
  return map * state_.cov() * map.transpose();
}

Eigen::Matrix3d InvariantEkf::body_velocity_covariance() const
{
  const Eigen::Matrix3d rotation = state_.R()();
  return rotation.transpose() *
         state_.cov().block<3, 3>(k_inertial_velocity, k_inertial_velocity) * rotation;
}

}  // namespace rpp_localization::InEKF
