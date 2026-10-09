#ifndef RPP_LOCALIZATION__INEKF__INERTIAL_PROCESS_HPP_
#define RPP_LOCALIZATION__INEKF__INERTIAL_PROCESS_HPP_

#include "Eigen/Dense"
#include "rpp_localization/inekf/inertial_prediction_model.hpp"

namespace rpp_localization::InEKF
{

/// Continuous-time noise of the IMU and of its bias random walks, as
/// standard deviations per square root of a second.
struct InertialProcessNoise
{
  Eigen::Vector3d gyro = Eigen::Vector3d::Constant(1e-3);
  Eigen::Vector3d accel = Eigen::Vector3d::Constant(1e-2);
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Constant(1e-5);
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Constant(1e-4);
};

/// Strapdown inertial propagation of a right-invariant error state.
///
/// The world z axis points up, so gravity is (0, 0, -gravity) and a level
/// IMU at rest reads +gravity on its z axis.
class InertialProcess final : public InertialPredictionModel
{
public:
  using Covariance = Eigen::Matrix<double, 15, 15>;

  explicit InertialProcess(double gravity = 9.80665);

  void set_noise(const InertialProcessNoise& noise);

  void predict(
    InertialLieState& state,
    const InertialPredictionInput& input) override;

private:
  Covariance transition(const InertialLieState& state, double delta) const;

  Eigen::Vector3d gravity_;
  Covariance noise_ = Covariance::Zero();
};

}  // namespace rpp_localization::InEKF

#endif  // RPP_LOCALIZATION__INEKF__INERTIAL_PROCESS_HPP_
