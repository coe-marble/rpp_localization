#pragma once

#include <algorithm>
#include <cmath>

namespace rpp_localization::navigation_helpers
{
// Preserve sensor variance normalization before submitting a measurement.
inline double normalize_measurement_variance(double variance)
{
  return std::max(std::abs(variance), 1e-9);
}
}  // namespace rpp_localization::navigation_helpers
