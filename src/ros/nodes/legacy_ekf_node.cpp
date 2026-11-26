#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/ros/ros_filter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<rpp_localization::RosFilter> filter =
    std::make_shared<rpp_localization::RosFilter>(options, "ekf");

  filter->init();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
