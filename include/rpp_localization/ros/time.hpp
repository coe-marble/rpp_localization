#pragma once

#include <chrono>

#include "rpp_localization/core/types.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace rpp_localization::ros
{

[[nodiscard]] inline TimestampNs to_timestamp_ns(const rclcpp::Time& time) noexcept
{
  return time.nanoseconds();
}

[[nodiscard]] inline DurationNs to_duration_ns(const rclcpp::Duration& duration) noexcept
{
  return duration.nanoseconds();
}

[[nodiscard]] inline rclcpp::Time to_ros_time(const TimestampNs timestamp)
{
  return rclcpp::Time(timestamp, RCL_ROS_TIME);
}

[[nodiscard]] inline rclcpp::Duration to_ros_duration(const DurationNs duration)
{
  return rclcpp::Duration(std::chrono::nanoseconds(duration));
}

[[nodiscard]] inline double to_seconds(const rclcpp::Duration& duration) noexcept
{
  return nanoseconds_to_seconds(to_duration_ns(duration));
}

[[nodiscard]] inline double to_seconds(const rclcpp::Time& time) noexcept
{
  return nanoseconds_to_seconds(to_timestamp_ns(time));
}

[[nodiscard]] inline double to_seconds(
  const std_msgs::msg::Header::_stamp_type& stamp) noexcept
{
  return static_cast<double>(stamp.sec) +
         nanoseconds_to_seconds(static_cast<TimestampNs>(stamp.nanosec));
}

}  // namespace rpp_localization::ros
