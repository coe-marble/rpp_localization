/*
 * SPDX-FileCopyrightText: (c) 2018, Locus Robotics
 * SPDX-FileCopyrightText: (c) 2019, Steve Macenski
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/ros/ros_filter_types.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<rpp_localization::RosEkf> filter =
    std::make_shared<rpp_localization::RosEkf>(options);

  filter->init();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
