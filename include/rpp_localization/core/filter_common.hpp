/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016  Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_COMMON_HPP_
#define RPP_LOCALIZATION__FILTER_COMMON_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rpp_localization/core/types.hpp"

namespace rpp_localization
{

/**
 * @brief Enumeration that defines the state vector
 */
enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

/**
 * @brief Enumeration that defines the control vector
 */
enum ControlMembers
{
  ControlMemberVx,
  ControlMemberVy,
  ControlMemberVz,
  ControlMemberVroll,
  ControlMemberVpitch,
  ControlMemberVyaw
};

/**
 * @brief Global constants that define our state
 * vector size and offsets to groups of values
 * within that state.
 */
const int STATE_SIZE = 15;
const int POSITION_OFFSET = StateMemberX;
const int ORIENTATION_OFFSET = StateMemberRoll;
const int POSITION_V_OFFSET = StateMemberVx;
const int ORIENTATION_V_OFFSET = StateMemberVroll;
const int POSITION_A_OFFSET = StateMemberAx;

/**
 * @brief Pose and twist messages each contain six variables
 */
const int POSE_SIZE = 6;
const int TWIST_SIZE = 6;
const int POSITION_SIZE = 3;
const int ORIENTATION_SIZE = 3;
const int LINEAR_VELOCITY_SIZE = 3;
const int ACCELERATION_SIZE = 3;
const int ANGULAR_VELOCITY_SIZE = 3;

/**
 * @brief Common constants
 */
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

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
    const std::shared_ptr<Measurement> & first,
    const std::shared_ptr<Measurement> & second)
  {
    return (*this)(*first, *second);
  }

  bool operator()(const Measurement & first, const Measurement & second)
  {
    return first.time_ > second.time_;
  }
};

using MeasurementPtr = std::shared_ptr<Measurement>;

/**
 * @brief Structure used for storing and comparing filter states.
 *
 * This structure is useful when higher-level classes need to remember filter
 * history. Measurement units are assumed to be in meters and radians.
 * Timestamps are integer nanoseconds in a caller-supplied clock domain.
 */
struct FilterState
{
  FilterState()
  : _state(), _estimate_error_covariance(), _latest_control(),
    _last_measurement_time(0), _latest_control_time(0)
  {
  }

  StateVector _state;
  CovarianceMatrix _estimate_error_covariance;
  ControlVector _latest_control;
  TimestampNs _last_measurement_time;
  TimestampNs _latest_control_time;

  // We want the queue to be sorted from latest to earliest timestamps.
  bool operator()(const FilterState & first, const FilterState & second)
  {
    return first._last_measurement_time < second._last_measurement_time;
  }
};

using FilterStatePtr = std::shared_ptr<FilterState>;

struct FilterResult
{
  std::vector<StateVector> states;
  std::vector<CovarianceMatrix> covariances;
  std::vector<double> timestamps;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_COMMON_HPP_
