/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTER_BASE_HPP_
#define RPP_LOCALIZATION__FILTER_BASE_HPP_

#include <ostream>
#include <vector>
#include <stdexcept>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/measurement.hpp"

namespace rpp_localization
{

template<class T>
class NavFilter
{

public:

  NavFilter();

  void reset();

  void init(std::shared_ptr<rclcpp::Node> node);

  void correct(const Measurement & measurement);

  void predict(
    const rclcpp::Time & reference_time,
    const rclcpp::Duration & delta);

  void process_measurement(const Measurement & measurement);


  bool get_debug();

  bool use_control();
  const ControlCommand& get_control();
  void set_control(const ControlCommand & control);

  const std::vector<bool>& get_control_update_vector();

  bool get_initialized_status();
  T& get_filter();
  void load_params();
  const Eigen::VectorXd& get_state();
  const Eigen::MatrixXd & get_estimate_error_covariance();


  const rclcpp::Time& get_last_measurement_time();
  const rclcpp::Duration& get_sensor_timeout();


  void set_debug(const bool debug, std::ostream * out_stream = nullptr);
  void set_last_measurement_time(const rclcpp::Time & last_measurement_time);
  void set_sensor_timeout(const rclcpp::Duration & sensor_timeout);
  void set_state(const Eigen::VectorXd & state);
  void set_estimate_error_covariance(const Eigen::MatrixXd & estimate_error_covariance);

  void validate_delta(rclcpp::Duration & /*delta*/);


private:
  /**
   * @brief Whether or not the filter is in debug mode
   */
  bool _debug;
  bool _initialized;
  bool _use_control;
  T _filter;

  rclcpp::Time _last_measurement_time;
  rclcpp::Duration _sensor_timeout;
  std::ostream* _debug_stream;
  std::shared_ptr<rclcpp::Node> _node;

  /**
  * @brief Covariance matrices can be incredibly unstable. We can add a small
  * value to it at each iteration to help maintain its positive-definite
  * property.
  */
  Eigen::MatrixXd _covariance_epsilon;

};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTER_BASE_HPP_
