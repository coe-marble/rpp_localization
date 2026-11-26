#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace rpp_localization::estimation
{
// One sample per replay attempt: saved history records after the rollback point.
// Welford's update needs constant storage and never allocates. Serialize cold.
struct ReplayStatistics
{
  std::uint64_t count{0}, states_max{0};
  double states_mean{0.0}, states_m2{0.0};

  void observe(std::uint64_t states) noexcept
  {
    ++count;
    states_max = std::max(states_max, states);
    const double value = static_cast<double>(states);
    const double delta = value - states_mean;
    states_mean += delta / static_cast<double>(count);
    states_m2 += delta * (value - states_mean);
  }

  double states_stddev() const noexcept
  {
    return count ? std::sqrt(std::max(0.0, states_m2 / static_cast<double>(count))) : 0.0;
  }

  void merge(const ReplayStatistics& other) noexcept
  {
    if (!other.count) {return;}
    if (!count) {*this = other; return;}
    const auto combined = count + other.count;
    const double delta = other.states_mean - states_mean;
    states_m2 += other.states_m2 + delta * delta * static_cast<double>(count) *
      static_cast<double>(other.count) / static_cast<double>(combined);
    states_mean += delta * static_cast<double>(other.count) / static_cast<double>(combined);
    states_max = std::max(states_max, other.states_max); count = combined;
  }

  std::string json() const
  {
    std::ostringstream out;
    out << "{\"count\":" << count << ",\"states_mean\":" << states_mean
        << ",\"states_stddev\":" << states_stddev() << ",\"states_max\":" << states_max << '}';
    return out.str();
  }
};
}  // namespace rpp_localization::estimation
