/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__ROS_FILTER_TYPES_HPP_
#define RPP_LOCALIZATION__ROS_FILTER_TYPES_HPP_

#include "rpp_localization/filters/ekf.hpp"
#include "rpp_localization/ros/ros_filter.hpp"
#include "rpp_localization/filters/ukf.hpp"
#include "rpp_localization/models/constant_acc_model.hpp"
#include "rpp_localization/inekf/inekf.hpp"
#include "rpp_localization/inekf/inertial_process.hpp"

namespace rpp_localization
{
    using RosUkf = RosFilter<Ukf<ConstantAccelerationModel>>;
    using RosEkf = RosFilter<Ekf<ConstantAccelerationModel>>;
    using RosInEkf = RosFilter<InEkf<InEKF::InertialProcess>>;
}

#endif  // RPP_LOCALIZATION__ROS_FILTER_TYPES_HPP_
