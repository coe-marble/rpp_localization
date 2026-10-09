#ifndef RPP_LOCALIZATION__INEKF__INVARIANT_EKF_HPP_
#define RPP_LOCALIZATION__INEKF__INVARIANT_EKF_HPP_

#include <limits>

#include "Eigen/Dense"
#include "rpp_localization/inekf/inertial_prediction_model.hpp"

namespace rpp_localization::InEKF
{

/// Right-invariant extended Kalman filter on an inertial navigation state.
///
/// The estimation error is defined by true = exp(error) * estimate. An IMU
/// sample drives each prediction; position, body-velocity and attitude
/// measurements correct the estimate.
class InvariantEkf
{
public:
  using Covariance = Eigen::Matrix<double, 15, 15>;

  explicit InvariantEkf(InertialPredictionModel& model);

  void set_state(const InertialLieState& state);
  [[nodiscard]] const InertialLieState& state() const noexcept {return state_;}

  /// Angular velocity of the last prediction with the gyroscope bias removed.
  [[nodiscard]] const Eigen::Vector3d& angular_velocity() const noexcept
  {
    return angular_velocity_;
  }

  void predict(const InertialPredictionInput& input);

  /// World position of a sensor mounted at `lever_arm` in the body frame.
  /// Returns false when the Mahalanobis gate rejects the measurement.
  bool correct_position(
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& covariance,
    const Eigen::Vector3d& lever_arm = Eigen::Vector3d::Zero(),
    double mahalanobis_threshold = std::numeric_limits<double>::infinity());

  /// Body-frame velocity of a sensor mounted at `lever_arm`.
  bool correct_body_velocity(
    const Eigen::Vector3d& velocity,
    const Eigen::Matrix3d& covariance,
    const Eigen::Vector3d& lever_arm = Eigen::Vector3d::Zero(),
    double mahalanobis_threshold = std::numeric_limits<double>::infinity());

  /// Rotation from body to world; `covariance` is its body-frame error.
  bool correct_attitude(
    const Eigen::Matrix3d& rotation,
    const Eigen::Matrix3d& covariance,
    double mahalanobis_threshold = std::numeric_limits<double>::infinity());

  /// Covariance of world position and world-frame rotation error, 6x6.
  [[nodiscard]] Eigen::Matrix<double, 6, 6> pose_covariance() const;

  /// Covariance of the body-frame linear velocity.
  [[nodiscard]] Eigen::Matrix3d body_velocity_covariance() const;

private:
  using Jacobian = Eigen::Matrix<double, 3, 15>;

  bool correct(
    const Eigen::Vector3d& innovation,
    const Jacobian& jacobian,
    const Eigen::Matrix3d& covariance,
    double mahalanobis_threshold);

  InertialPredictionModel& model_;
  InertialLieState state_;
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
};

}  // namespace rpp_localization::InEKF

#endif  // RPP_LOCALIZATION__INEKF__INVARIANT_EKF_HPP_
