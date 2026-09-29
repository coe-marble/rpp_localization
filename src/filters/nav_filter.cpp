/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/filters/nav_filter.hpp"

#include <algorithm>
#include <ostream>
#include <vector>

#include "angles/angles.h"
#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"
#include "rpp_localization/filters/ekf.hpp"
#include "rpp_localization/filters/ukf.hpp"
#include "rpp_localization/models/constant_acc_model.hpp"
#include "rpp_localization/inekf/inekf.hpp"
#include "rpp_localization/inekf/inertial_process.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/validation.hpp"
#include "rpp_localization/ros/time.hpp"



namespace rpp_localization
{

template<class T>
NavFilter<T>::NavFilter()
: _initialized(false),
  _last_measurement_time(0),
  _sensor_timeout(0, 0u),
  _debug_stream(nullptr),
  _debug(false),
  _filter(STATE_SIZE)
{
  reset();
}

template<class T>
void NavFilter<T>::init(rclcpp::Node& node)
{
  load_params(node);
  _filter.init(node);
}

template<class T>
void NavFilter<T>::reset()
{
  _initialized = false;

  // Set the estimate error covariance. We want our measurements
  // to be accepted rapidly when the filter starts, so we should
  // initialize the state's covariance with large values.
  auto& estimate_error_covariance = _filter.get_model()->get_state_covariance_unsafe();
  estimate_error_covariance.setIdentity();
  estimate_error_covariance *= 1e-9;

  // Set the epsilon matrix to be a matrix with small values on the diagonal
  // It is used to maintain the positive-definite property of the covariance
  _covariance_epsilon.setIdentity();
  _covariance_epsilon *= 0.001;

  // Assume 30Hz from sensor data (configurable)
  _sensor_timeout = rclcpp::Duration::from_seconds(0.033333333);

  // Initialize our last update and measurement times
  _last_measurement_time = 0;

}


template<class T>
bool NavFilter<T>::get_debug() {return _debug;}

template<class T>
bool NavFilter<T>::get_initialized_status() {return _initialized;}


template<class T>
T& NavFilter<T>::get_filter() { return _filter;}

template<class T>
TimestampNs NavFilter<T>::get_last_measurement_time()
{
  return _last_measurement_time;
}

template<class T>
const rclcpp::Duration& NavFilter<T>::get_sensor_timeout()
{
  return _sensor_timeout;
}

template<class T>
const Eigen::MatrixXd & NavFilter<T>::get_estimate_error_covariance()
{
  return _filter.get_model()->get_state_covariance();
}

template<class T>
void NavFilter<T>::correct(const Measurement & measurement)
{
  _filter.correct(measurement);
}

template<class T>
void NavFilter<T>::predict(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  static_cast<RuntimeFilter&>(_filter).predict(reference_time, delta);
}

template<class T>
bool NavFilter<T>::use_control()
{
  return _filter.get_model()->use_control();
}

template<class T>
const ControlCommand & NavFilter<T>::get_control()
{
  if (_filter.get_model() == nullptr)
    throw std::runtime_error("Model does not use control");
  return _filter.get_model()->get_control();
}

template<class T>
void NavFilter<T>::set_control(const ControlCommand& control)
{
  if (_filter.get_model() == nullptr)
    throw std::runtime_error("Model does not use control");
  _filter.get_model()->set_control(control);
}

template<class T>
const std::vector<bool>& NavFilter<T>::get_control_update_vector()
{
  if (_filter.get_model() == nullptr)
    throw std::runtime_error("Model does not use control");
  // nav filter, if it uses mode, has nav model base for sure
  return dynamic_cast<NavModelBase*>(_filter.get_model())->get_control_update_vector();
}

template<class T>
const Eigen::VectorXd& NavFilter<T>::get_state()
{
  return _filter.get_model()->get_state();
}

template<class T>
void NavFilter<T>::process_measurement(const Measurement & measurement)
{

  Eigen::VectorXd& state = _filter.get_model()->get_state_unsafe();
  FB_DEBUG(
    "------ FilterBase::process_measurement (" << measurement.topic_name_ <<
      ") ------\n");

  DurationNs delta = 0;
  auto measurement_time_status = validation::MeasurementTimeStatus::kCurrent;

  // If we've had a previous reading, then go through the predict/update
  // cycle. Otherwise, set our state and covariance to whatever we get
  // from this measurement.
  if (_initialized) {
    // Determine how much time has passed since our last measurement
    delta = measurement.time_ - _last_measurement_time;
    measurement_time_status = validation::classifyMeasurementDelta(delta);

    FB_DEBUG(
      "Filter is already initialized. Carrying out predict/correct loop...\n"
      "Measurement time is " <<
        std::setprecision(20) << measurement.time_ <<
        ", last measurement time is " << _last_measurement_time <<
        ", delta is " << delta << "\n");

    // Only want to carry out a prediction if it's
    // forward in time. Otherwise, just correct.
    if (measurement_time_status == validation::MeasurementTimeStatus::kForward) {
      rclcpp::Duration ros_delta = ros::toRosDuration(delta);
      validate_delta(ros_delta);
      predict(measurement.time_, ros_delta.nanoseconds());

      // Return this to the user
    }

    correct(measurement);
  } else {
    FB_DEBUG("First measurement. Initializing filter.\n");

    // Initialize the filter, but only with the values we're using
    size_t measurement_length = measurement.update_vector_.size();
    for (size_t i = 0; i < measurement_length; ++i) {
      state[i] = (measurement.update_vector_[i] ? measurement.measurement_[i] :
        state[i]);
    }

    // Same for covariance
    auto& estimate_error_covariance = _filter.get_model()->get_state_covariance_unsafe();
    for (size_t i = 0; i < measurement_length; ++i) {
      for (size_t j = 0; j < measurement_length; ++j) {
        estimate_error_covariance(i, j) =
          (measurement.update_vector_[i]
            && measurement.update_vector_[j]
            && abs(measurement.covariance_(i, j)) > 1e-9 ?
          measurement.covariance_(i, j) :
          estimate_error_covariance(i, j));
      }
    }

    _initialized = true;
  }

  if (measurement_time_status != validation::MeasurementTimeStatus::kStale) {
    // Update the last measurement and update time.
    // The measurement time is based on the time stamp of the
    // measurement, whereas the update time is based on this
    // node's current ROS time. The update time is used to
    // determine if we have a sensor timeout, whereas the
    // measurement time is used to calculate time deltas for
    // prediction and correction.
    _last_measurement_time = measurement.time_;
  }

  FB_DEBUG(
    "------ /FilterBase::process_measurement (" << measurement.topic_name_ <<
      ") ------\n");
}

template<class T>
void NavFilter<T>::set_debug(const bool debug, std::ostream * out_stream)
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

template<class T>
void NavFilter<T>::set_last_measurement_time(
  const TimestampNs last_measurement_time)
{
  _last_measurement_time = last_measurement_time;
}

template<class T>
void NavFilter<T>::set_sensor_timeout(const rclcpp::Duration & sensor_timeout)
{
  _sensor_timeout = sensor_timeout;
}

template<class T>
void NavFilter<T>::set_state(const Eigen::VectorXd & state)
{
  _filter.get_model()->set_state(state);
}

template<class T>
void NavFilter<T>::validate_delta(rclcpp::Duration & /*delta*/)
{
  // TODO(someone): Need to verify this condition B'Coz
  // rclcpp::Duration::from_seconds(100000.0) value is 0.00010000000000000000479
  // This handles issues with ROS time when use_sim_time is on and we're playing
  // from bags.
  /* if (delta > rclcpp::Duration::from_seconds(100000.0))
  {
    FB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to
  0.01\n");

    delta = rclcpp::Duration::from_seconds(0.01);
  } */
}


template<class T>
void NavFilter<T>::set_estimate_error_covariance(
  const Eigen::MatrixXd & estimate_error_covariance)
{
  _filter.get_model()->set_state_covariance(estimate_error_covariance);
}



template<class T>
void NavFilter<T>::load_params(rclcpp::Node& node)
{

  if (!node.has_parameter("use_control"))
  {
    node.declare_parameter("use_control", false);
  }
  node.get_parameter("use_control", _use_control);


  Eigen::MatrixXd estimate_error_covariance(STATE_SIZE, STATE_SIZE);
  ros_filter_utilities::load_covariance_parameter(node, "initial_estimate_covariance", estimate_error_covariance);
  set_estimate_error_covariance(estimate_error_covariance);
  FB_DEBUG("Initial estimate covariance is:\n" << estimate_error_covariance << "\n");
}


}  // namespace rpp_localization

template class rpp_localization::NavFilter<rpp_localization::Ekf<rpp_localization::ConstantAccelerationModel>>;
template class rpp_localization::NavFilter<rpp_localization::Ukf<rpp_localization::ConstantAccelerationModel>>;
template class rpp_localization::NavFilter<rpp_localization::InEkf<rpp_localization::InEKF::InertialProcess>>;
