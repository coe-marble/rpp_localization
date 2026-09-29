#pragma once

#include "rpp_localization/core/types.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>


namespace rpp_localization
{

// Compatibility representation of the legacy queued measurement. Timestamps use
// the ROS-independent core nanosecond clock.
struct ControlCommand
{
  TimestampNs stamp;
  ControlVector control;
};

struct Measurement
{
  Measurement()
  : time_(0),
    mahalanobis_thresh_(std::numeric_limits<double>::max()),
    topic_name_(""),
    latest_control_()
  {
  }

  TimestampNs time_;
  double mahalanobis_thresh_;
  std::string topic_name_;
  std::vector<bool> update_vector_;
  ControlCommand latest_control_;
  MeasurementVector measurement_;
  CovarianceMatrix covariance_;

  // Earlier measurements have greater priority.
  bool operator()(
    const std::shared_ptr<Measurement>& first,
    const std::shared_ptr<Measurement>& second)
  {
    return (*this)(*first, *second);
  }

  bool operator()(const Measurement& first, const Measurement& second)
  {
    return first.time_ > second.time_;
  }
};

using MeasurementPtr = std::shared_ptr<Measurement>;

}  // namespace rpp_localization
