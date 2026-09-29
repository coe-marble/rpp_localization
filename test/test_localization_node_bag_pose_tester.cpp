/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gtest/gtest.h"

nav_msgs::msg::Odometry filtered_;

using namespace std::chrono_literals;

void filterCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  filtered_ = *msg;
}

TEST(BagTest, PoseCheck) {
  auto node = rclcpp::Node::make_shared("localization_node_bag_pose_tester");

  // getting parameters value from yaml file using get_parameter() API
  double finalX = node->declare_parameter("final_x", 0.0);
  double finalY = node->declare_parameter("final_y", 0.0);
  double finalZ = node->declare_parameter("final_z", 0.0);
  double tolerance = node->declare_parameter("tolerance", 0.0);
  bool outputFinalPosition = node->declare_parameter("output_final_position", false);
  std::string finalPositionFile = node->declare_parameter(
    "output_location",
    std::string("test.txt"));

  auto filteredSub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", rclcpp::QoS(1), filterCallback);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rclcpp::Rate(3).sleep();
  }

  if (outputFinalPosition) {
    try {
      std::ofstream posOut;
      posOut.open(finalPositionFile.c_str(), std::ofstream::app);
      posOut << filtered_.pose.pose.position.x << " " <<
        filtered_.pose.pose.position.y << " " <<
        filtered_.pose.pose.position.z << std::endl;
      posOut.close();
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Unable to open output file.\n");
    }
  }

  double xDiff = filtered_.pose.pose.position.x - finalX;
  double yDiff = filtered_.pose.pose.position.y - finalY;
  double zDiff = filtered_.pose.pose.position.z - finalZ;

  std::cout << "xDiff =" << xDiff << std::endl;
  std::cout << "yDiff =" << yDiff << std::endl;
  std::cout << "zDiff =" << zDiff << std::endl;

  EXPECT_LT(::sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff), tolerance);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::Rate(0.5).sleep();

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
