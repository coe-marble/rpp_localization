/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_UTILITIES_HPP_
#define RPP_LOCALIZATION__FILTER_UTILITIES_HPP_

#include <cmath>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "angles/angles.h"
#include "rpp_localization/core/filter_common.hpp"

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
inline void append_prefix(const std::string & tf_prefix, std::string & frame_id)
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

bool check_mahalanobis_threshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & inverse_innovation_covariance,
  double n_sigmas);

/**
* @brief Keeps the state Euler angles in the range [-pi, pi]
*/
void wrap_state_angles(Eigen::VectorXd& state);

}  // namespace filter_utilities
}  // namespace rpp_localization

inline std::ostream & operator<<(std::ostream & os, const Eigen::MatrixXd & mat)
{
  os << "[";

  int row_count = static_cast<int>(mat.rows());

  for (int row = 0; row < row_count; ++row) {
    if (row > 0) {
      os << " ";
    }

    for (int col = 0; col < mat.cols(); ++col) {
      os << std::setiosflags(std::ios::left) << std::setw(12) <<
        std::setprecision(5) << mat(row, col);
    }

    if (row < row_count - 1) {
      os << "\n";
    }
  }

  os << "]\n";

  return os;
}

inline std::ostream & operator<<(std::ostream & os, const Eigen::VectorXd & vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

inline std::ostream & operator<<(std::ostream & os, const std::vector<size_t> & vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

inline std::ostream & operator<<(std::ostream & os, const std::vector<int> & vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(3) <<
      (vec[dim] ? "t" : "f");
  }
  os << "]\n";

  return os;
}

namespace rpp_localization
{
namespace filter_utilities
{


inline bool check_mahalanobis_threshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & inverse_innovation_covariance,
  double n_sigmas)
{
  if (std::isnan(n_sigmas) || n_sigmas < 0.0 ||
    inverse_innovation_covariance.rows() != innovation.size() ||
    inverse_innovation_covariance.cols() != innovation.size())
  {
    return false;
  }
  const double squared_mahalanobis =
    innovation.dot(inverse_innovation_covariance * innovation);
  return std::isfinite(squared_mahalanobis) && squared_mahalanobis >= 0.0 &&
    std::sqrt(squared_mahalanobis) < n_sigmas;
}

inline void wrap_state_angles(Eigen::VectorXd& state)
{
  using namespace rpp_localization;
  state(StateMemberRoll) = angles::normalize_angle(state(StateMemberRoll));
  state(StateMemberPitch) = angles::normalize_angle(state(StateMemberPitch));
  state(StateMemberYaw) = angles::normalize_angle(state(StateMemberYaw));
}

}

}

#endif  // RPP_LOCALIZATION__FILTER_UTILITIES_HPP_
