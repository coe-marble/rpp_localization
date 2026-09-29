#pragma once

#include "rpp_localization/core/measurement.hpp"

namespace rpp_localization
{

class RuntimeFilter
{
public:
  virtual ~RuntimeFilter() = default;

  virtual void correct(const Measurement& measurement) = 0;
  virtual void predict(TimestampNs reference_time, DurationNs delta) = 0;
};

}  // namespace rpp_localization
