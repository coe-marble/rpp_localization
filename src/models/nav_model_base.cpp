/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/models/nav_model_base.hpp"

#include <algorithm>
#include <ostream>
#include <vector>

#include "angles/angles.h"
#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"
#include "rpp_localization/core/measurement.hpp"

namespace rpp_localization
{
NavModelBase::NavModelBase(int state_dim)
: ModelBase(state_dim),
  _initialized(false),
  _use_dynamic_process_noise_covariance(false),
  _control_timeout(0, 0u),
  _debug_stream(nullptr),
  acceleration_gains_(TWIST_SIZE, 0.0),
  acceleration_limits_(TWIST_SIZE, 0.0),
  deceleration_gains_(TWIST_SIZE, 0.0),
  deceleration_limits_(TWIST_SIZE, 0.0),
  _control_update_vector(TWIST_SIZE, 0),
  control_acceleration_(TWIST_SIZE),
  _covariance_epsilon(STATE_SIZE, STATE_SIZE),
  dynamic_process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  _identity(STATE_SIZE, STATE_SIZE),
  process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  _debug(false)
{
  reset();
}

NavModelBase::~NavModelBase() {}


void NavModelBase::init(std::shared_ptr<rclcpp::Node> node)
{
  _node = node;
  load_params();
}


void NavModelBase::reset()
{
  _initialized = false;

  control_acceleration_.setZero();

  // We need the identity for the update equations
  _identity.setIdentity();

  // Set the epsilon matrix to be a matrix with small values on the diagonal
  // It is used to maintain the positive-definite property of the covariance
  _covariance_epsilon.setIdentity();
  _covariance_epsilon *= 0.001;


  // These can be overridden via the launch parameters,
  // but we need default values.
  process_noise_covariance_.setZero();
  process_noise_covariance_(StateMemberX, StateMemberX) = 0.05;
  process_noise_covariance_(StateMemberY, StateMemberY) = 0.05;
  process_noise_covariance_(StateMemberZ, StateMemberZ) = 0.06;
  process_noise_covariance_(StateMemberRoll, StateMemberRoll) = 0.03;
  process_noise_covariance_(StateMemberPitch, StateMemberPitch) = 0.03;
  process_noise_covariance_(StateMemberYaw, StateMemberYaw) = 0.06;
  process_noise_covariance_(StateMemberVx, StateMemberVx) = 0.025;
  process_noise_covariance_(StateMemberVy, StateMemberVy) = 0.025;
  process_noise_covariance_(StateMemberVz, StateMemberVz) = 0.04;
  process_noise_covariance_(StateMemberVroll, StateMemberVroll) = 0.01;
  process_noise_covariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
  process_noise_covariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
  process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.01;
  process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.01;
  process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.015;

  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

void NavModelBase::compute_dynamic_process_noise_covariance(
  const Eigen::VectorXd & state, Eigen::MatrixXd& covariance)
{
  // A more principled approach would be to get the current velocity from the
  // state, make a diagonal matrix from it, and then rotate it to be in the
  // world frame (i.e., the same frame as the pose data). We could then use this
  // rotated velocity matrix to scale the process noise covariance for the pose
  // variables as rotatedVelocityMatrix * poseCovariance *
  // rotatedVelocityMatrix' However, this presents trouble for robots that may
  // incur rotational error as a result of linear motion (and vice-versa).
  // Instead, we create a diagonal matrix whose diagonal values are the vector
  // norm of the state's velocity. We use that to scale the process noise
  // covariance.
  Eigen::MatrixXd velocity_matrix(TWIST_SIZE, TWIST_SIZE);
  velocity_matrix.setIdentity();
  velocity_matrix.diagonal() *=
    state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

  covariance.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET, POSITION_OFFSET) =
    velocity_matrix *
    process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET,
    POSITION_OFFSET) *
    velocity_matrix.transpose();
}

bool NavModelBase::get_initialized_status() {return _initialized;}

void NavModelBase::set_control_params(
  const std::vector<bool> & update_vector,
  const rclcpp::Duration & control_timeout,
  const std::vector<double> & acceleration_limits,
  const std::vector<double> & acceleration_gains,
  const std::vector<double> & deceleration_limits,
  const std::vector<double> & deceleration_gains)
{
  _use_control = true;
  _control_update_vector = update_vector;
  _control_timeout = control_timeout;
  acceleration_limits_ = acceleration_limits;
  acceleration_gains_ = acceleration_gains;
  deceleration_limits_ = deceleration_limits;
  deceleration_gains_ = deceleration_gains;
}


bool NavModelBase::get_debug() {return _debug;}

void NavModelBase::set_debug(const bool debug, std::ostream * out_stream)
{
  if (debug) {
    if (out_stream != NULL) {
      _debug_stream = out_stream;
      _debug = true;
    } else {
      _debug = false;
    }
  } else {
    _debug = false;
  }
}

const Eigen::MatrixXd & NavModelBase::get_process_noise_covariance()
{
  return process_noise_covariance_;
}

void NavModelBase::set_process_noise_covariance(
  const Eigen::MatrixXd & process_noise_covariance)
{
  process_noise_covariance_ = process_noise_covariance;
  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

void NavModelBase::validate_delta(rclcpp::Duration & /*delta*/)
{
  // TODO(someone): Need to verify this condition B'Coz
  // rclcpp::Duration::from_seconds(100000.0) value is 0.00010000000000000000479
  // This handles issues with ROS time when use_sim_time is on and we're playing
  // from bags.
  /* if (delta > rclcpp::Duration::from_seconds(100000.0))
  {
    MB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to
  0.01\n");

    delta = rclcpp::Duration::from_seconds(0.01);
  } */
}

void NavModelBase::prepareControl(
  const rclcpp::Time & reference_time,
  const double)
{
  control_acceleration_.setZero();

  if (_use_control) {
    bool timed_out =
      (reference_time - _control.stamp >= _control_timeout);

    if (timed_out) {
      MB_DEBUG(
        "Control timed out. Reference time was " <<
          reference_time.nanoseconds() << ", latest control time was " <<
          _control.stamp.nanoseconds() << ", control timeout was " <<
          _control_timeout.nanoseconds() << "\n");
    }

    for (size_t controlInd = 0; controlInd < TWIST_SIZE; ++controlInd) {
      if (_control_update_vector[controlInd]) {
        control_acceleration_(controlInd) = computeControlAcceleration(
          _state(controlInd + POSITION_V_OFFSET),
          (timed_out ? 0.0 : _control.control(controlInd)),
          acceleration_limits_[controlInd], acceleration_gains_[controlInd],
          deceleration_limits_[controlInd], deceleration_gains_[controlInd]);
      }
    }
  }
}

inline double NavModelBase::computeControlAcceleration(
  const double state,
  const double control,
  const double acceleration_limit,
  const double acceleration_gain,
  const double deceleration_limit,
  const double deceleration_gain)
{
  MB_DEBUG("---------- FilterBase::computeControlAcceleration ----------\n");

  const double error = control - state;
  const bool same_sign = (::fabs(error) <= ::fabs(control) + 0.01);
  const double set_point = (same_sign ? control : 0.0);
  const bool decelerating = ::fabs(set_point) < ::fabs(state);
  double limit = acceleration_limit;
  double gain = acceleration_gain;

  if (decelerating) {
    limit = deceleration_limit;
    gain = deceleration_gain;
  }

  const double final_accel = std::min(std::max(gain * error, -limit), limit);

  MB_DEBUG(
    "Control value: " <<
      control << "\n" <<
      "State value: " << state << "\n" <<
      "Error: " << error << "\n" <<
      "Same sign: " << (same_sign ? "true" : "false") << "\n" <<
      "Set point: " << set_point << "\n" <<
      "Decelerating: " << (decelerating ? "true" : "false") << "\n" <<
      "Limit: " << limit << "\n" <<
      "Gain: " << gain << "\n" <<
      "Final is " << final_accel << "\n");

  return final_accel;
}


const std::vector<bool>&
NavModelBase::get_control_update_vector()
{
  return _control_update_vector;
}


void NavModelBase::load_params()
{
  if (!_node->has_parameter("use_control"))
  {
    _node->declare_parameter("use_control", false);
  }
  _node->get_parameter("use_control", _use_control);

  auto timeout = 0.0;
  if (!_node->has_parameter("control_timeout"))
  {
    _node->declare_parameter("control_timeout", timeout);
  }
  _node->get_parameter("control_timeout", timeout);
  _control_timeout = rclcpp::Duration(std::chrono::nanoseconds((long)(timeout * 1e9)));


  if (!_node->has_parameter("dynamic_process_noise_covariance"))
  {
    _node->declare_parameter("dynamic_process_noise_covariance", false);
  }
  _node->get_parameter("dynamic_process_noise_covariance", _use_dynamic_process_noise_covariance);

  if (_use_control) {
    _node->declare_parameter("control_config", rclcpp::PARAMETER_BOOL_ARRAY);
    if (_node->get_parameter("control_config", _control_update_vector)) {
      if (_control_update_vector.size() != TWIST_SIZE) {
        RCLCPP_ERROR_STREAM(
          _node->get_logger(),
          "Control configuration must be of size " <<
            TWIST_SIZE << ". Provided config was of size " << _control_update_vector.size() <<
            ". No control term will be used.");
        _use_control = false;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        _node->get_logger(),
        "use_control is set to true, but control_config is missing. No control term will be "
        "used.");
      _control_update_vector.resize(TWIST_SIZE, 0);
      _use_control = false;
    }

    _node->declare_parameter("acceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (_node->get_parameter("acceleration_limits", acceleration_limits_)) {
      if (acceleration_limits_.size() != TWIST_SIZE) {
        RCLCPP_ERROR_STREAM(
          _node->get_logger(),
          "Acceleration configuration must be of size " << TWIST_SIZE <<
            ". Provided config was of size " << acceleration_limits_.size() <<
            ". No control term will be used.");
        _use_control = false;
      }
    } else {
      RCLCPP_ERROR_STREAM(
        _node->get_logger(),
        "use_control is set to true, but acceleration_limits is missing. Will use default "
        "values.");
      acceleration_limits_.resize(TWIST_SIZE, 1.0);
    }

    _node->declare_parameter("acceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (_node->get_parameter("acceleration_gains", acceleration_gains_)) {
      const int size = acceleration_gains_.size();
      if (size != TWIST_SIZE) {
        RCLCPP_ERROR_STREAM(
          _node->get_logger(),
          "Acceleration gain configuration must be of size " << TWIST_SIZE << ". Provided config "
            "was of size " << size << ". All gains will be assumed to be 1.");
        std::fill_n(
          acceleration_gains_.begin(), std::min(size, TWIST_SIZE),
          1.0);
        acceleration_gains_.resize(TWIST_SIZE, 1.0);
      }
    }

    _node->declare_parameter("deceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (_node->get_parameter("deceleration_limits", deceleration_limits_)) {
      if (deceleration_limits_.size() != TWIST_SIZE) {
        RCLCPP_ERROR_STREAM(
          _node->get_logger(),
          "Deceleration configuration must be of size " << TWIST_SIZE << ". Provided config was "
            "of size " << deceleration_limits_.size() << ". No control term will be used.");
        _use_control = false;
      }
    } else {
      RCLCPP_WARN_STREAM(
        _node->get_logger(),
        "use_control is set to true, but no deceleration_limits specified. Will use acceleration "
        "limits.");
      deceleration_limits_ = acceleration_limits_;
    }

    _node->declare_parameter("deceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (_node->get_parameter("deceleration_gains", deceleration_gains_)) {
      const int size = deceleration_gains_.size();
      if (size != TWIST_SIZE) {
        RCLCPP_ERROR_STREAM(
          _node->get_logger(),
          "Deceleration gain configuration must be of size " << TWIST_SIZE << ". Provided config "
            "was of size " << size << ". All gains will be assumed to be 1.");
        std::fill_n(
          deceleration_gains_.begin(), std::min(size, TWIST_SIZE),
          1.0);
        deceleration_gains_.resize(TWIST_SIZE, 1.0);
      }
    } else {
      RCLCPP_WARN_STREAM(
        _node->get_logger(),
        "use_control is set to true, but no deceleration_gains specified. Will use acceleration "
        "gains.");
      deceleration_gains_ = acceleration_gains_;
    }
  }
  else {
    acceleration_limits_.resize(TWIST_SIZE, 1.0);
    acceleration_gains_.resize(TWIST_SIZE, 1.0);
    deceleration_limits_.resize(TWIST_SIZE, 1.0);
    deceleration_gains_.resize(TWIST_SIZE, 1.0);
  }

  std::vector<double> initial_state;
  _node->declare_parameter("initial_state", rclcpp::PARAMETER_DOUBLE_ARRAY);
  if (_node->get_parameter("initial_state", initial_state)) {
    if (initial_state.size() != STATE_SIZE) {
      RCLCPP_ERROR_STREAM(
        _node->get_logger(),
        "Initial state must be of size " << STATE_SIZE << ". Provided config was of size " <<
          initial_state.size() << ". The initial state will be ignored.");
    }
    else
    {
      Eigen::Map<Eigen::VectorXd> eigen_state(initial_state.data(),
        initial_state.size());
      _state = eigen_state;
    }
  }

  // Now that we've checked if IMU linear acceleration is being used, we can
  // determine our final control parameters
  if (_use_control && std::accumulate(
      _control_update_vector.begin(),
      _control_update_vector.end(), 0) == 0)
  {
    RCLCPP_ERROR_STREAM(
      _node->get_logger(),
      "use_control is set to true, but control_config has only false values. No control term will "
      "be used.");
    _use_control = false;
  }

  ros_filter_utilities::load_covariance_parameter(*_node, "process_noise_covariance", process_noise_covariance_);
  MB_DEBUG("Process noise covariance is:\n" << process_noise_covariance_ << "\n");
}

}  // namespace rpp_localization
