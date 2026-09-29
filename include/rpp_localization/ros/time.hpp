#pragma once

#include <chrono>

#include "rpp_localization/core/types.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace rpp_localization::ros
{

[[nodiscard]] inline TimestampNs toTimestampNs(const rclcpp::Time& time) noexcept
{
  return time.nanoseconds();
}

[[nodiscard]] inline DurationNs toDurationNs(const rclcpp::Duration& duration) noexcept
{
  return duration.nanoseconds();
}

[[nodiscard]] inline rclcpp::Time toRosTime(const TimestampNs timestamp)
{
  return rclcpp::Time(timestamp, RCL_ROS_TIME);
}

[[nodiscard]] inline rclcpp::Duration toRosDuration(const DurationNs duration)
{
  return rclcpp::Duration(std::chrono::nanoseconds(duration));
}

[[nodiscard]] inline double toSeconds(const rclcpp::Duration& duration) noexcept
{
  return nanosecondsToSeconds(toDurationNs(duration));
}

[[nodiscard]] inline double toSeconds(const rclcpp::Time& time) noexcept
{
  return nanosecondsToSeconds(toTimestampNs(time));
}

[[nodiscard]] inline double toSeconds(
  const std_msgs::msg::Header::_stamp_type& stamp) noexcept
{
  return static_cast<double>(stamp.sec) +
         nanosecondsToSeconds(static_cast<TimestampNs>(stamp.nanosec));
}

}  // namespace rpp_localization::ros
