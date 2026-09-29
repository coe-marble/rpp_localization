/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS, Helmond
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <memory>
#include <string>

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "test_rpp_localization_listener_publisher");

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 1);
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub =
    node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("accel/filtered", 1);

  tf2_ros::StaticTransformBroadcaster transform_broadcaster(node);

  rclcpp::Time end_time = node->now() + rclcpp::Duration(10, 0);
  while (rclcpp::ok() && node->now() < end_time) {
    rclcpp::Time time1(1000, 0);
    double x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az;
    x = y = z = roll = pitch = yaw = vy = vz = vroll = vpitch = vyaw = ax = ay = az = 0.0;
    vx = 1.0;
    vroll = M_PI / 4.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = time1;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.y = z;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = vz;
    odom_msg.twist.twist.angular.x = vroll;
    odom_msg.twist.twist.angular.y = vpitch;
    odom_msg.twist.twist.angular.z = vyaw;

    geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
    accel_msg.header.stamp = time1;
    accel_msg.header.frame_id = "base_link";
    accel_msg.accel.accel.linear.x = ax;
    accel_msg.accel.accel.linear.y = ay;
    accel_msg.accel.accel.linear.z = az;
    accel_msg.accel.accel.angular.x = 0;
    accel_msg.accel.accel.angular.y = 0;
    accel_msg.accel.accel.angular.z = 0;

    odom_pub->publish(odom_msg);
    accel_pub->publish(accel_msg);

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "sensor";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 1.0;
    transformStamped.transform.translation.z = 0.0;
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, M_PI / 2);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      transform_broadcaster.sendTransform(transformStamped);
    }

    rclcpp::Rate(10).sleep();
  }

  return 0;
}
