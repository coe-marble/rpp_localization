/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-FileCopyrightText: (c) 2019, Steve Macenski
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_STATE_HPP_
#define RPP_LOCALIZATION__FILTER_STATE_HPP_

#include <memory>

#include "Eigen/Dense"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace rpp_localization
{

/**
 * @brief Structure used for storing and comparing filter states
 *
 * This structure is useful when higher-level classes need to remember filter
 * history. Measurement units are assumed to be in meters and radians. Times are
 * real-valued and measured in seconds.
 */
struct FilterState
{
  FilterState()
  : _state(), _estimate_error_covariance(), _latest_control(),
    _last_measurement_time(0.0), _latest_control_time(0)
  {
  }

  // The filter state vector
  Eigen::VectorXd _state;

  // The filter error covariance matrix
  Eigen::MatrixXd _estimate_error_covariance;

  // The most recent control vector
  Eigen::VectorXd _latest_control;

  // The time stamp of the most recent measurement for the filter
  rclcpp::Time _last_measurement_time;

  // The time stamp of the most recent control term
  rclcpp::Time _latest_control_time;

  // We want the queue to be sorted from latest to earliest timestamps.
  bool operator()(const FilterState & a, const FilterState & b)
  {
    return a._last_measurement_time < b._last_measurement_time;
  }
};
using FilterStatePtr = std::shared_ptr<FilterState>;

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_STATE_HPP_
