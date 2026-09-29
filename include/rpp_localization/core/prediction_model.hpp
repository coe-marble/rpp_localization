#pragma once

#include "rpp_localization/core/types.hpp"

namespace rpp_localization
{

class PredictionModel
{
public:
  virtual ~PredictionModel() = default;

  virtual void predict(
    StateVector& state,
    CovarianceMatrix& state_covariance,
    TimestampNs reference_time,
    DurationNs delta) = 0;
};

}  // namespace rpp_localization
