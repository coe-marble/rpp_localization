/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS, Helmond
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/ros/listener.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<rpp_localization::RosRppLocalizationListener> g_listener;

TEST(LocalizationListenerTest, testGetStateOfBaseLink)
{
  rclcpp::spin_some(node);

  rclcpp::Time time2(1001, 0);

  Eigen::VectorXd state(rpp_localization::STATE_SIZE);
  Eigen::MatrixXd covariance(rpp_localization::STATE_SIZE, rpp_localization::STATE_SIZE);


  std::string base_frame("base_link");
  g_listener->get_state(time2, base_frame, state, covariance);

  EXPECT_DOUBLE_EQ(1.0, state(rpp_localization::StateMemberX));
  EXPECT_DOUBLE_EQ(0.0, state(rpp_localization::StateMemberY));
  EXPECT_DOUBLE_EQ(0.0, state(rpp_localization::StateMemberZ));

  EXPECT_FLOAT_EQ(M_PI / 4, state(rpp_localization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberPitch));
  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberYaw));

  EXPECT_DOUBLE_EQ(M_PI / 4.0, state(rpp_localization::StateMemberVroll));
  EXPECT_DOUBLE_EQ(0.0, state(rpp_localization::StateMemberVpitch));
  EXPECT_DOUBLE_EQ(0.0, state(rpp_localization::StateMemberVyaw));
}

TEST(LocalizationListenerTest, GetStateOfRelatedFrame)
{
  rclcpp::spin_some(node);

  Eigen::VectorXd state(rpp_localization::STATE_SIZE);
  Eigen::MatrixXd covariance(rpp_localization::STATE_SIZE, rpp_localization::STATE_SIZE);

  rclcpp::Time time1(1000, 0);
  rclcpp::Time time2(1001, 0);

  std::string sensor_frame("sensor");

  EXPECT_TRUE(g_listener->get_state(time1, sensor_frame, state, covariance) );

  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberX));
  EXPECT_FLOAT_EQ(1.0, state(rpp_localization::StateMemberY));
  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberZ));

  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberPitch));
  EXPECT_FLOAT_EQ(M_PI / 2, state(rpp_localization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(rpp_localization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(rpp_localization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4.0, state(rpp_localization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(rpp_localization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(rpp_localization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0.0, state(rpp_localization::StateMemberVyaw));

  EXPECT_TRUE(g_listener->get_state(time2, sensor_frame, state, covariance));

  EXPECT_FLOAT_EQ(1.0, state(rpp_localization::StateMemberX));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(rpp_localization::StateMemberY));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(rpp_localization::StateMemberZ));

  EXPECT_TRUE(1e-12 > state(rpp_localization::StateMemberRoll));
  EXPECT_TRUE(1e-12 > fabs(-M_PI / 4.0 - state(rpp_localization::StateMemberPitch)));
  EXPECT_FLOAT_EQ(M_PI / 2, state(rpp_localization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(rpp_localization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(rpp_localization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4, state(rpp_localization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(rpp_localization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(rpp_localization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0, state(rpp_localization::StateMemberVyaw));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("test_ros_rpp_localization_listener");

  g_listener = std::make_unique<rpp_localization::RosRppLocalizationListener>(node);

  ::testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  rclcpp::shutdown();
  node = nullptr;
  g_listener = nullptr;

  return res;
}
