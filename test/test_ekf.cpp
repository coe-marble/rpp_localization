/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/filters/ekf.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/ros/ros_filter.hpp"
#include "rpp_localization/ros/ros_filter_types.hpp"

using rpp_localization::Ekf;
using rpp_localization::RosEkf;
using rpp_localization::STATE_SIZE;

TEST(EkfTest, Measurements) {
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<rpp_localization::RosEkf> filter =
    std::make_shared<rpp_localization::RosEkf>(options);
  filter->init();

  // create the instance of the class and pass parameters
  Eigen::MatrixXd initialCovar(15, 15);

  initialCovar.setIdentity();
  initialCovar *= 0.5;

  filter->getFilter().set_estimate_error_covariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }
  std::vector<bool> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  rclcpp::Time time1(1000);
  filter->rpp_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time1);

  filter->rpp_localization::RosEkf::integrateMeasurements(rclcpp::Time(1001));

  EXPECT_EQ(filter->getFilter().get_state(), measurement);
  EXPECT_EQ(
    filter->getFilter().get_estimate_error_covariance(),
    measurementCovariance);

  filter->getFilter().set_estimate_error_covariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  rclcpp::Time time2(1002);

  filter->rpp_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement2, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time2);

  filter->rpp_localization::RosEkf::integrateMeasurements(rclcpp::Time(1003));

  measurement = measurement2.eval() - filter->getFilter().get_state();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
