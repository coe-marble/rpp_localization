/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/filters/estimator.hpp"

TEST(RLETest, StateBuffer)
{
  // Generate a few empty estimator states
  std::vector<rpp_localization::EstimatorState> states;

  for (int i = 0; i < 10; i++) {
    /*
     * t = i s;
     * x = i m;
     * vx = 1.0 m/s;
     */
    rpp_localization::EstimatorState state;
    state.time_stamp = i;
    state.state(rpp_localization::StateMemberX) = i;
    state.state(rpp_localization::StateMemberY) = 0;
    state.state(rpp_localization::StateMemberZ) = 0;

    state.state(rpp_localization::StateMemberRoll) = 0;
    state.state(rpp_localization::StateMemberPitch) = 0;
    state.state(rpp_localization::StateMemberYaw) = 0;

    state.state(rpp_localization::StateMemberVx) = 1;
    state.state(rpp_localization::StateMemberVy) = 0;
    state.state(rpp_localization::StateMemberVz) = 0;

    state.state(rpp_localization::StateMemberVroll) = 0;
    state.state(rpp_localization::StateMemberVpitch) = 0;
    state.state(rpp_localization::StateMemberVyaw) = 0;

    state.state(rpp_localization::StateMemberAx) = 0;
    state.state(rpp_localization::StateMemberAy) = 0;
    state.state(rpp_localization::StateMemberAz) = 0;
    states.push_back(state);
  }

  // Instantiate a robot localization estimator with a buffer capacity of 5
  unsigned int buffer_capacity = 5;
  Eigen::MatrixXd process_noise_covariance = Eigen::MatrixXd::Identity(
    rpp_localization::STATE_SIZE, rpp_localization::STATE_SIZE);
  rpp_localization::RppLocalizationEstimator estimator(buffer_capacity,
    rpp_localization::FilterTypes::EKF, process_noise_covariance);

  rpp_localization::EstimatorState state;

  // Add the states in chronological order
  for (int i = 0; i < 6; i++) {
    estimator.set_state(states[i]);

    // Check that the state is added correctly
    estimator.get_state(states[i].time_stamp, state);
    EXPECT_EQ(state.time_stamp, states[i].time_stamp);
  }

  // We filled the buffer with more states that it can hold, so its size should
  // now be equal to the capacity
  EXPECT_EQ(static_cast<unsigned int>(estimator.getSize()), buffer_capacity);

  // Clear the buffer and check if it's really empty afterwards
  estimator.clearBuffer();
  EXPECT_EQ(estimator.getSize(), 0u);

  // Add states at time 1 through 3 inclusive to the buffer (buffer is not yet
  // full)
  for (int i = 1; i < 4; i++) {
    estimator.set_state(states[i]);
  }

  // Now add a state at time 0, but let's change it a bit (set StateMemberY=12)
  // so that we can inspect if it is correctly added to the buffer.
  rpp_localization::EstimatorState state_2 = states[0];
  state_2.state(rpp_localization::StateMemberY) = 12;
  estimator.set_state(state_2);
  EXPECT_EQ(
    rpp_localization::EstimatorResults::Exact,
    estimator.get_state(states[0].time_stamp, state));

  // Check if the state is correctly added
  EXPECT_EQ(state.state, state_2.state);

  // Add some more states. State at t=0 should now be dropped, so we should get
  // the prediction, which means y=0
  for (int i = 5; i < 8; i++) {
    estimator.set_state(states[i]);
  }
  EXPECT_EQ(
    rpp_localization::EstimatorResults::ExtrapolationIntoPast,
    estimator.get_state(states[0].time_stamp, state));
  EXPECT_EQ(states[0].state, state.state);

  // Estimate a state that is not in the buffer, but can be determined by
  // interpolation. The predicted state vector should be equal to the designed
  // state at the requested time.
  EXPECT_EQ(
    rpp_localization::EstimatorResults::Interpolation,
    estimator.get_state(states[4].time_stamp, state));
  EXPECT_EQ(states[4].state, state.state);

  // Estimate a state that is not in the buffer, but can be determined by
  // extrapolation into the future. The predicted state vector should be equal
  // to the designed state at the requested time.
  EXPECT_EQ(
    rpp_localization::EstimatorResults::ExtrapolationIntoFuture,
    estimator.get_state(states[8].time_stamp, state));
  EXPECT_EQ(states[8].state, state.state);

  // Add missing state somewhere in the middle
  estimator.set_state(states[4]);

  // Overwrite state at t=3 (oldest state now in the buffer) and check if it's
  // correctly overwritten.
  state_2 = states[3];
  state_2.state(rpp_localization::StateMemberVy) = -1.0;
  estimator.set_state(state_2);
  EXPECT_EQ(
    rpp_localization::EstimatorResults::Exact,
    estimator.get_state(states[3].time_stamp, state));
  EXPECT_EQ(state_2.state, state.state);

  // Add state that came too late
  estimator.set_state(states[0]);

  // Check if get_state needed to do extrapolation into the past
  EXPECT_EQ(
    estimator.get_state(states[0].time_stamp, state),
    rpp_localization::EstimatorResults::ExtrapolationIntoPast);

  // Check state at t=0. This can only work correctly if the state at t=3 is
  // overwritten and the state at zero is not in the buffer.
  EXPECT_DOUBLE_EQ(3.0, state.state(rpp_localization::StateMemberY));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_rpp_localization_estimator");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
