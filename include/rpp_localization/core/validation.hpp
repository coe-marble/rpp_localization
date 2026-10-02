#pragma once

#include <cmath>
#include <cstdint>

#include "rpp_localization/core/types.hpp"

namespace rpp_localization::validation
{

inline constexpr double kMinimumMeasurementCovariance = 1e-9;

enum class CovarianceStatus : std::uint8_t
{
  kValid,
  kNegative,
  kNearZero,
  kNegativeAndNearZero,
};

struct NormalizedCovariance
{
  double value;
  CovarianceStatus status;
};

[[nodiscard]] inline NormalizedCovariance normalize_measurement_covariance(
  double covariance)
{
  const bool was_negative = covariance < 0.0;
  if (was_negative)
  {
    covariance = std::fabs(covariance);
  }

  const bool was_near_zero = covariance < kMinimumMeasurementCovariance;
  if (was_near_zero)
  {
    covariance = kMinimumMeasurementCovariance;
  }

  const auto status = was_negative ?
    (was_near_zero ? CovarianceStatus::kNegativeAndNearZero : CovarianceStatus::kNegative) :
    (was_near_zero ? CovarianceStatus::kNearZero : CovarianceStatus::kValid);

  return {covariance, status};
}

[[nodiscard]] constexpr bool has_negative_covariance(const CovarianceStatus status)
{
  return status == CovarianceStatus::kNegative ||
         status == CovarianceStatus::kNegativeAndNearZero;
}

[[nodiscard]] constexpr bool has_near_zero_covariance(const CovarianceStatus status)
{
  return status == CovarianceStatus::kNearZero ||
         status == CovarianceStatus::kNegativeAndNearZero;
}

enum class MeasurementTimeStatus : std::uint8_t
{
  kStale,
  kCurrent,
  kForward,
};

[[nodiscard]] inline MeasurementTimeStatus classify_measurement_delta(
  const DurationNs delta)
{
  if (delta > 0)
  {
    return MeasurementTimeStatus::kForward;
  }
  if (delta < 0)
  {
    return MeasurementTimeStatus::kStale;
  }
  return MeasurementTimeStatus::kCurrent;
}

}  // namespace rpp_localization::validation
