/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_
#define RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_


#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/filter_common.hpp"
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

class Ekf : public FilterBase
{
  public:

    explicit Ekf(ModelBase& model);
    ~Ekf();


    void correct(const Measurement& measurement) override;

    void predict(TimestampNs reference_time, DurationNs delta) override;

  private:
    Eigen::MatrixXd _identity;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_
