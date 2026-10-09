#ifndef RPP_LOCALIZATION__INEKF__INERTIAL_PAYLOAD_HPP_
#define RPP_LOCALIZATION__INEKF__INERTIAL_PAYLOAD_HPP_

#include <rpp_localization/inekf/inertial_prediction_model.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

// Conversion between the InertialState payload and the core Lie state,
// shared by the inertial model and invariant filter plugins.
namespace rpp_localization::plugins::detail
{

inline constexpr std::uint16_t k_inertial_ok = 0;
inline constexpr std::uint16_t k_inertial_invalid_payload = 1;
inline constexpr std::uint16_t k_inertial_invalid_time = 2;
inline constexpr std::uint16_t k_inertial_invalid_state = 3;
inline constexpr std::uint16_t k_inertial_invalid_covariance = 4;
inline constexpr std::uint16_t k_inertial_not_initialized = 5;
inline constexpr std::uint16_t k_inertial_model_failure = 7;
inline constexpr std::uint16_t k_inertial_numerical_failure = 9;

class InertialPayloadError final : public std::runtime_error
{
public:
  InertialPayloadError(const std::uint16_t code, const std::string& message)
  : std::runtime_error(message),
    code_(code)
  {
  }

  [[nodiscard]] std::uint16_t code() const noexcept {return code_;}

private:
  std::uint16_t code_;
};

template<typename Values>
bool has_finite_values(const Values& values, const std::size_t expected_size)
{
  if (values.size() != expected_size) {
    return false;
  }
  for (std::size_t index = 0; index < expected_size; ++index) {
    if (!std::isfinite(values[index])) {
      return false;
    }
  }
  return true;
}

template<typename Values>
Eigen::Vector3d read_vector3(const Values& values, const std::string& name)
{
  if (!has_finite_values(values, 3)) {
    throw InertialPayloadError(
            k_inertial_invalid_payload, name + " must contain 3 finite values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

template<typename Values>
Eigen::Matrix3d read_matrix3(const Values& values, const std::string& name)
{
  if (!has_finite_values(values, 9)) {
    throw InertialPayloadError(
            k_inertial_invalid_payload, name + " must contain 9 finite values");
  }
  Eigen::Matrix3d matrix;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      matrix(row, column) = values[3 * row + column];
    }
  }
  return matrix;
}

template<typename Values, typename Vector>
void write_values(Values values, const Vector& vector)
{
  values.resize(static_cast<std::size_t>(vector.size()));
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    values[static_cast<std::size_t>(index)] = vector(index);
  }
}

template<typename Values, typename Matrix>
void write_row_major(Values values, const Matrix& matrix)
{
  values.resize(static_cast<std::size_t>(matrix.rows() * matrix.cols()));
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      values[static_cast<std::size_t>(row * matrix.cols() + column)] = matrix(row, column);
    }
  }
}

/// Reads a right-invariant state; the left convention is not implemented.
template<typename State>
InEKF::InertialLieState read_inertial_state(const State& state)
{
  if (!state.rightInvariant()) {
    throw InertialPayloadError(
            k_inertial_invalid_state, "only right-invariant inertial states are supported");
  }
  const Eigen::Matrix3d rotation = read_matrix3(state.rotation(), "rotation");
  if ((rotation * rotation.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
    rotation.determinant() < 0.0)
  {
    throw InertialPayloadError(k_inertial_invalid_state, "rotation must be a rotation matrix");
  }

  InEKF::InertialLieState::MatrixState matrix = InEKF::InertialLieState::MatrixState::Identity();
  matrix.block<3, 3>(0, 0) = rotation;
  matrix.block<3, 1>(0, 3) = read_vector3(state.velocity(), "velocity");
  matrix.block<3, 1>(0, 4) = read_vector3(state.position(), "position");

  Eigen::Matrix<double, 6, 1> biases;
  biases << read_vector3(state.gyroBias(), "gyroBias"), read_vector3(state.accelBias(), "accelBias");

  const auto covariance_values = state.covariance();
  if (!has_finite_values(covariance_values, 225)) {
    throw InertialPayloadError(
            k_inertial_invalid_covariance, "covariance must contain 225 finite values");
  }
  InEKF::InertialLieState::MatrixCov covariance;
  for (std::size_t row = 0; row < 15; ++row) {
    for (std::size_t column = 0; column < 15; ++column) {
      covariance(row, column) = covariance_values[15 * row + column];
    }
    if (covariance(row, row) < 0.0) {
      throw InertialPayloadError(
              k_inertial_invalid_covariance, "covariance must have a non-negative diagonal");
    }
  }
  return InEKF::InertialLieState(matrix, covariance, biases);
}

template<typename State>
void write_inertial_state(State state, const InEKF::InertialLieState& value)
{
  write_row_major(state.rotation(), value.R()());
  write_values(state.velocity(), value[0]);
  write_values(state.position(), value[1]);
  write_values(state.gyroBias(), Eigen::Vector3d(value.aug().template head<3>()));
  write_values(state.accelBias(), Eigen::Vector3d(value.aug().template tail<3>()));
  write_row_major(state.covariance(), value.cov());
  state.rightInvariant() = true;
}

template<typename Imu>
Eigen::Matrix<double, 6, 1> read_imu(const Imu& imu)
{
  Eigen::Matrix<double, 6, 1> sample;
  sample << read_vector3(imu.angularVelocity(), "angularVelocity"),
    read_vector3(imu.specificForce(), "specificForce");
  return sample;
}

template<typename Imu>
void write_imu(Imu imu, const Eigen::Matrix<double, 6, 1>& sample)
{
  write_values(imu.angularVelocity(), Eigen::Vector3d(sample.head<3>()));
  write_values(imu.specificForce(), Eigen::Vector3d(sample.tail<3>()));
}

}  // namespace rpp_localization::plugins::detail

#endif  // RPP_LOCALIZATION__INEKF__INERTIAL_PAYLOAD_HPP_
