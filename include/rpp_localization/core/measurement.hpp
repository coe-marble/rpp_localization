#pragma once

#include <Eigen/Dense>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

namespace rpp_localization
{

// Compatibility representation of the legacy queued measurement. This remains
// ROS-time based until the later ROS-free core extraction.
struct ControlCommand
{
  rclcpp::Time stamp;
  Eigen::VectorXd control;
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

  rclcpp::Time time_;
  double mahalanobis_thresh_;
  std::string topic_name_;
  std::vector<bool> update_vector_;
  ControlCommand latest_control_;
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

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
