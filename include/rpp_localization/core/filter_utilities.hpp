/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_UTILITIES_HPP_
#define RPP_LOCALIZATION__FILTER_UTILITIES_HPP_

#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

#define FB_DEBUG(msg) \
  if (get_debug()) { \
    *_debug_stream << msg; \
  }

// Handy methods for debug output
std::ostream & operator<<(std::ostream & os, const Eigen::MatrixXd & mat);
std::ostream & operator<<(std::ostream & os, const Eigen::VectorXd & vec);
std::ostream & operator<<(std::ostream & os, const std::vector<size_t> & vec);
std::ostream & operator<<(std::ostream & os, const std::vector<int> & vec);

namespace rpp_localization
{
namespace filter_utilities
{

/**
 * @brief Utility method for appending tf2 prefixes cleanly
 * @param[in] tf_prefix - the tf2 prefix to append
 * @param[in, out] frame_id - the resulting frame_id value
 */
inline void appendPrefix(const std::string & tf_prefix, std::string & frame_id)
{
  size_t frame_id_prefix_index = 0u;
  size_t tf_prefix_index = 0u;

  // Strip all leading slashes for tf2 compliance
  if (!frame_id.empty() && frame_id.at(0u) == '/') {
    frame_id_prefix_index = 1u;
  }

  if (!tf_prefix.empty() && tf_prefix.at(0u) == '/') {
    tf_prefix_index = 1u;
  }

  // If we do have a tf prefix, then put a slash in between
  if (!tf_prefix.empty()) {
    frame_id = tf_prefix.substr(tf_prefix_index) + "/" + frame_id.substr(frame_id_prefix_index);
  }
}

inline double nanosecToSec(const rcl_time_point_value_t nanoseconds)
{
  return static_cast<double>(nanoseconds) * 1e-9;
}

inline int secToNanosec(const double seconds)
{
  return static_cast<int>(seconds * 1e9);
}

inline double toSec(const rclcpp::Duration & duration)
{
  return nanosecToSec(duration.nanoseconds());
}

inline double toSec(const rclcpp::Time & time)
{
  return nanosecToSec(time.nanoseconds());
}

inline double toSec(const std_msgs::msg::Header::_stamp_type & stamp)
{
  return static_cast<double>(stamp.sec) + nanosecToSec(stamp.nanosec);
}

bool check_mahalanobis_threshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_covariance,
  double n_sigmas);

/**
* @brief Keeps the state Euler angles in the range [-pi, pi]
*/
void wrapStateAngles(Eigen::VectorXd& state);

}  // namespace filter_utilities
}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_UTILITIES_HPP_
