/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <iostream>
#include <queue>
#include <string>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/filters/nav_filter.hpp"
#include "rpp_localization/models/nav_model_base.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"

#include "filters/nav_filter.cpp"

using rpp_localization::Measurement;
using rpp_localization::STATE_SIZE;

namespace rpp_localization
{


class TestModel : public NavModelBase
{
public:
  rclcpp::Time val;

  TestModel(int state_dim)
  : NavModelBase(state_dim),
    val(0) {}

  void init(std::shared_ptr<rclcpp::Node> node) override
  {
    NavModelBase::init(node);
  }

  void step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance,
      const rclcpp::Time & reference_time, const double dT) override
  {

  }
};

template<class T>
class FilterDerived : public FilterBase
{
public:
  rclcpp::Time val;
  T model;

  FilterDerived(int state_dim)
  : FilterBase(state_dim),
    model(state_dim),
    val(0, 1002)
    {
      set_model_base(model);
    }

  void init(std::shared_ptr<rclcpp::Node> node) override
  {
    _node = node;
  }

  void correct(const Measurement & measurement)
  {
    EXPECT_EQ(val, measurement.time_);
    EXPECT_EQ(measurement.topic_name_, "odomTest");

    EXPECT_EQ(measurement.update_vector_.size(), 10u);
    for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
      EXPECT_EQ(measurement.update_vector_[i], true);
    }
  }
  void predict(
    const rclcpp::Time & /*reference_time*/,
    const rclcpp::Duration & /*delta*/) {}
};

}  // namespace rpp_localization

template class rpp_localization::FilterDerived<rpp_localization::TestModel>;
template class rpp_localization::NavFilter<rpp_localization::FilterDerived<rpp_localization::TestModel>>;
typedef rpp_localization::NavFilter<rpp_localization::FilterDerived<rpp_localization::TestModel>> NavFilterDerived;

TEST(FilterBaseTest, MeasurementStruct) {
  Measurement meas1;
  Measurement meas2;

  EXPECT_EQ(meas1.topic_name_, std::string(""));
  EXPECT_EQ(meas1.time_, rclcpp::Time(0));
  EXPECT_EQ(meas2.time_, rclcpp::Time(0));

  // Comparison test is true if the first
  // argument is > the second, so should
  // be false if they're equal.
  EXPECT_EQ(meas1(meas1, meas2), false);
  EXPECT_EQ(meas2(meas2, meas1), false);

  builtin_interfaces::msg::Time msg1;
  msg1.sec = 0;
  msg1.nanosec = 100;

  builtin_interfaces::msg::Time msg2;
  msg2.sec = 0;
  msg2.nanosec = 200;

  meas1.time_ = msg1;
  meas2.time_ = msg2;

  EXPECT_EQ(meas1(meas1, meas2), false);
  EXPECT_EQ(meas1(meas2, meas1), true);
  EXPECT_EQ(meas2(meas1, meas2), false);
  EXPECT_EQ(meas2(meas2, meas1), true);
}

TEST(FilterBaseTest, DerivedFilterGetSet) {
  NavFilterDerived derived;
  auto node = rclcpp::Node::make_shared("test_filter_base");
  derived.init(node);

  // With the ostream argument as NULL,
  // the debug flag will remain false.
  derived.set_debug(true);

  EXPECT_FALSE(derived.get_debug());

  // Now set the stream and do it again
  std::stringstream os;
  derived.set_debug(true, &os);

  EXPECT_TRUE(derived.get_debug());

  // Simple get/set checks
  double timeout = 7.4;
  derived.set_sensor_timeout(rclcpp::Duration::from_seconds(timeout));
  EXPECT_EQ(derived.get_sensor_timeout(), rclcpp::Duration::from_seconds(timeout));

  double lastMeasTime = 3.83;
  derived.set_last_measurement_time(rclcpp::Time(lastMeasTime));
  EXPECT_EQ(derived.get_last_measurement_time(), rclcpp::Time(lastMeasTime));

  Eigen::MatrixXd pnCovar(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    for (size_t j = 0; j < STATE_SIZE; ++j) {
      pnCovar(i, j) = static_cast<double>(i * j);
    }
  }
  derived.get_filter().model.set_state_covariance(pnCovar);
  auto statecov = derived.get_filter().model.get_state_covariance();
  EXPECT_EQ(statecov, pnCovar);

  derived.get_filter().model.set_process_noise_covariance(pnCovar);
  pnCovar = derived.get_filter().model.get_process_noise_covariance();
  EXPECT_EQ(statecov, pnCovar);

  Eigen::VectorXd state(STATE_SIZE);
  state.setZero();
  derived.set_state(state);
  EXPECT_EQ(derived.get_state(), state);

  EXPECT_EQ(derived.get_initialized_status(), false);
}

TEST(FilterBaseTest, MeasurementProcessing) {
  NavFilterDerived derived;
  auto node = rclcpp::Node::make_shared("test_filter_base");
  derived.init(node);

  Measurement meas;

  Eigen::VectorXd measurement(STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = 0.1 * static_cast<double>(i);
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    for (size_t j = 0; j < STATE_SIZE; ++j) {
      measurementCovariance(i, j) = 0.1 * static_cast<double>(i * j);
    }
  }

  meas.topic_name_ = "odomTest";
  meas.measurement_ = measurement;
  meas.covariance_ = measurementCovariance;
  meas.update_vector_.resize(10, true);
  meas.time_ = rclcpp::Time(0, 1000);

  // The filter shouldn't be initializedyet
  EXPECT_FALSE(derived.get_filter().model.get_initialized_status());

  derived.process_measurement(meas);

  // Now it's initialized, and the entire filter state
  // should be equal to the first state
  EXPECT_TRUE(derived.get_initialized_status());
  EXPECT_EQ(derived.get_state().head(10), measurement.head(10));

  // Process a measurement and make sure it updates the
  // lastMeasurementTime variable
  meas.time_ = rclcpp::Time(0, 1002);
  derived.process_measurement(meas);
  EXPECT_EQ(derived.get_last_measurement_time(), meas.time_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
