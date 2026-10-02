#pragma once

#include <Eigen/Dense>

#include <cstdint>

namespace rpp_localization
{

using StateVector = Eigen::VectorXd;
using CovarianceMatrix = Eigen::MatrixXd;
using MeasurementVector = Eigen::VectorXd;
using ControlVector = Eigen::VectorXd;

// All values share a caller-supplied epoch and clock domain.
using TimestampNs = std::int64_t;
using DurationNs = std::int64_t;

[[nodiscard]] constexpr double nanoseconds_to_seconds(const TimestampNs nanoseconds) noexcept
{
  return static_cast<double>(nanoseconds) * 1e-9;
}

}  // namespace rpp_localization
