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

#include "rpp_localization/filters/ekf.hpp"
#include "rpp_localization/models/constant_acc_model.hpp"
#include "rpp_localization/ros/rosbag_filter.hpp"
#include "ros/rosbag_filter.cpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace std::chrono_literals;

typedef rpp_localization::RosBagFilter<rpp_localization::Ekf<rpp_localization::ConstantAccelerationModel>> FilterEkf;

TEST(BagTest, PoseCheck) {

  const auto parameters_file =
    ament_index_cpp::get_package_share_directory("rpp_localization") +
    "/test/test_ekf_localization_node_bag1.yaml";
  rclcpp::NodeOptions options;
  options.arguments({
    "test_ekf_localization_node_bag1", "--ros-args", "--params-file", parameters_file});

  const auto file_path =
    ament_index_cpp::get_package_share_directory("rpp_localization") +
    "/test/test1/test1.db3";

  RosBagMsgProvider provider(file_path);
  auto node = std::make_shared<FilterEkf>(options, provider, 50);
  node->init();

  FilterResult result = node->filter();

  // getting parameters value from yaml file using get_parameter() API
  double finalX = node->declare_parameter("final_x", 0.0);
  double finalY = node->declare_parameter("final_y", 0.0);
  double finalZ = node->declare_parameter("final_z", 0.0);
  double tolerance = node->declare_parameter("tolerance", 0.0);
  bool outputFinalPosition = node->declare_parameter("output_final_position", false);
  std::string finalPositionFile = node->declare_parameter(
    "output_location",
    std::string("test.txt"));

  nav_msgs::msg::Odometry filtered;
  node->getFilteredOdometryMessage(&filtered);
  if (outputFinalPosition) {
    try {
      std::ofstream posOut;
      posOut.open(finalPositionFile.c_str(), std::ofstream::app);
      posOut << filtered.pose.pose.position.x << " " <<
        filtered.pose.pose.position.y << " " <<
        filtered.pose.pose.position.z << std::endl;
      posOut.close();
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Unable to open output file.\n");
    }
  }


  double xDiff = filtered.pose.pose.position.x - finalX;
  double yDiff = filtered.pose.pose.position.y - finalY;
  double zDiff = filtered.pose.pose.position.z - finalZ;

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
