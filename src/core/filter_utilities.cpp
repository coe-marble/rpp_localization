/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/core/filter_utilities.hpp"

#include <iomanip>
#include <vector>

#include "angles/angles.h"

#include "rpp_localization/core/filter_common.hpp"


std::ostream & operator<<(std::ostream & os, const Eigen::MatrixXd & mat)
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

std::ostream & operator<<(std::ostream & os, const Eigen::VectorXd & vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<size_t> & vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<int> & vec)
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


bool check_mahalanobis_threshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_covariance,
  double n_sigmas)
{
  return true;
  double squared_mahalanobis =
    innovation.dot(innovation_covariance * innovation);
  double threshold = n_sigmas * n_sigmas;

  if (squared_mahalanobis >= threshold)
  {
    return false;
  }
  return true;
}

void wrap_state_angles(Eigen::VectorXd& state)
{
  using namespace rpp_localization;
  state(StateMemberRoll) = angles::normalize_angle(state(StateMemberRoll));
  state(StateMemberPitch) = angles::normalize_angle(state(StateMemberPitch));
  state(StateMemberYaw) = angles::normalize_angle(state(StateMemberYaw));
}

}

}