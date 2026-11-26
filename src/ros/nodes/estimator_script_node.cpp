#include <rpp_localization/ros/estimator_script_node.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rpp_localization::EstimatorScriptNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("estimator_script"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
