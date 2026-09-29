/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__EKF_HPP_
#define RPP_LOCALIZATION__EKF_HPP_

#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/filter_utilities.hpp"

namespace rpp_localization
{

/**
 * @brief Extended Kalman filter class
 *
 * Implementation of an extended Kalman filter (EKF). This class derives from
 * FilterBase and overrides the predict() and correct() methods in keeping with
 * the discrete time EKF algorithm.
 */

template<class T> // model type
class Ekf : public FilterBase
{
  public:

    Ekf(int state_space_dim);
    ~Ekf();


    void init(rclcpp::Node& node);

    void correct(const Measurement& measurement) override;

    void predict(TimestampNs reference_time, DurationNs delta) override;

    bool get_debug();
    void set_debug(const bool debug, std::ostream * out_stream);

  private:
    bool _debug;
    std::ostream* _debug_stream;
    Eigen::MatrixXd _identity;
    T _model;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__EKF_HPP_
