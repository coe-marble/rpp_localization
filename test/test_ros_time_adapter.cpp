#include "gtest/gtest.h"
#include "rpp_localization/ros/time.hpp"

namespace rpp_localization
{
namespace
{

TEST(RosTimeAdapterTest, PreservesLegacyTimeAndDurationConversions)
{
  const rclcpp::Time time(static_cast<TimestampNs>(2'000'000'123), RCL_ROS_TIME);
  const rclcpp::Duration duration{std::chrono::nanoseconds(3'000'000'456)};

  EXPECT_EQ(ros::toTimestampNs(time), 2'000'000'123);
  EXPECT_EQ(ros::toDurationNs(duration), 3'000'000'456);
  EXPECT_DOUBLE_EQ(ros::toSeconds(time), 2.000000123);
  EXPECT_DOUBLE_EQ(ros::toSeconds(duration), 3.000000456);
  EXPECT_EQ(ros::toRosTime(123).nanoseconds(), 123);
  EXPECT_EQ(ros::toRosDuration(456).nanoseconds(), 456);
}

TEST(RosTimeAdapterTest, PreservesHeaderStampConversion)
{
  std_msgs::msg::Header header;
  header.stamp.sec = 7;
  header.stamp.nanosec = 42;

  EXPECT_DOUBLE_EQ(ros::toSeconds(header.stamp), 7.000000042);
}

}  // namespace
}  // namespace rpp_localization
