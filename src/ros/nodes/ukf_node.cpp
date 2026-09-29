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
  options.arguments({"ukf_filter_node"});
  std::shared_ptr<rpp_localization::RosUkf> filter =
    std::make_shared<rpp_localization::RosUkf>(options);
  double alpha = filter->declare_parameter("alpha", 0.001);
  double kappa = filter->declare_parameter("kappa", 0.0);
  double beta = filter->declare_parameter("beta", 2.0);
  filter->getFilter().get_filter().setConstants(alpha, kappa, beta);
  filter->init();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
