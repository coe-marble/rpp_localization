#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/geodetic.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace rpp_localization
{

/// Converts GNSS fixes to east-north-up positions in the world frame, so a
/// localization filter can fuse them as a pose input.
class NavSatPoseNode : public rclcpp::Node
{
public:
  NavSatPoseNode()
  : Node("navsat_pose_node")
  {
    world_frame_id_ = declare_parameter("world_frame", std::string("odom"));
    const auto datum = declare_parameter("datum", std::vector<double>{});
    if (datum.size() == 3) {
      datum_ = geodetic::Geodetic{datum[0], datum[1], datum[2]};
    } else if (!datum.empty()) {
      throw std::invalid_argument("datum must contain latitude, longitude, altitude");
    }

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "gnss/pose", rclcpp::QoS(10));
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gnss/fix", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix) {on_fix(*fix);});
  }

private:
  void on_fix(const sensor_msgs::msg::NavSatFix& fix)
  {
    if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      !std::isfinite(fix.altitude))
    {
      return;
    }

    const geodetic::Geodetic position{fix.latitude, fix.longitude, fix.altitude};
    if (!datum_) {
      // Without a configured datum the first fix becomes the world origin.
      datum_ = position;
      RCLCPP_INFO(
        get_logger(), "Using the first fix as datum: %.8f, %.8f, %.3f",
        position.latitude_deg, position.longitude_deg, position.altitude);
    }
    const auto east_north_up = geodetic::to_enu(*datum_, position);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = fix.header.stamp;
    pose.header.frame_id = world_frame_id_;
    pose.pose.pose.position.x = east_north_up.x();
    pose.pose.pose.position.y = east_north_up.y();
    pose.pose.pose.position.z = east_north_up.z();
    pose.pose.pose.orientation.w = 1.0;
    // A fix covariance is east, north, up in metres, like the pose position.
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        pose.pose.covariance[6 * row + column] = fix.position_covariance[3 * row + column];
      }
    }
    pose_publisher_->publish(pose);
  }

  std::string world_frame_id_;
  std::optional<geodetic::Geodetic> datum_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
};

}  // namespace rpp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpp_localization::NavSatPoseNode>());
  rclcpp::shutdown();
  return 0;
}
