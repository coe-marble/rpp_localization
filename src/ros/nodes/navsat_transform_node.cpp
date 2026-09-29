/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/ros/navsat_transform.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto navsat_transform_node = std::make_shared<rpp_localization::NavSatTransform>(options);

  rclcpp::spin(navsat_transform_node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
