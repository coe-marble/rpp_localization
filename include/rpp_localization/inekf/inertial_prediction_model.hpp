#ifndef RPP_LOCALIZATION__INEKF__INERTIAL_PREDICTION_MODEL_HPP_
#define RPP_LOCALIZATION__INEKF__INERTIAL_PREDICTION_MODEL_HPP_

#include "Eigen/Dense"
#include "rpp_localization/core/types.hpp"
#include "rpp_localization/inekf/se3.hpp"

namespace rpp_localization::InEKF
{

/// Extended pose of an inertial navigation state with IMU biases.
///
/// The group element holds the rotation from body to world, the velocity
/// and the position in the world frame. The augmented part holds the
/// gyroscope bias followed by the accelerometer bias, both in the body
/// frame. The 15x15 covariance is on the tangent space, ordered rotation,
/// velocity, position, gyroscope bias, accelerometer bias.
using InertialLieState = SE3<2, 6>;

inline constexpr int k_inertial_rotation = 0;
inline constexpr int k_inertial_velocity = 3;
inline constexpr int k_inertial_position = 6;
inline constexpr int k_inertial_gyro_bias = 9;
inline constexpr int k_inertial_accel_bias = 12;
inline constexpr int k_inertial_tangent_size = 15;

/// One IMU sample that drives a prediction over `delta`.
struct InertialPredictionInput
{
  TimestampNs reference_time{};
  DurationNs delta{};
  /// Angular velocity followed by specific force, both in the body frame.
  Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero();
};

/// Propagates an inertial state and its covariance with an IMU sample.
class InertialPredictionModel
{
public:
  virtual ~InertialPredictionModel() = default;

  virtual void predict(
    InertialLieState& state,
    const InertialPredictionInput& input) = 0;
};

}  // namespace rpp_localization::InEKF

#endif  // RPP_LOCALIZATION__INEKF__INERTIAL_PREDICTION_MODEL_HPP_
