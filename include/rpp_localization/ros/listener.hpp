/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS Helmond.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__ROS_RPP_LOCALIZATION_LISTENER_HPP_
#define RPP_LOCALIZATION__ROS_RPP_LOCALIZATION_LISTENER_HPP_

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/filters/estimator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rpp_localization
{

namespace detail
{
inline rclcpp::SubscriptionOptions
get_subscription_options_with_default_qos_override_policies()
{
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  return subscription_options;
}
}  // namespace detail

//! @brief RosRppLocalizationListener class
//!
//! This class wraps the RppLocalizationEstimator. It listens to topics over
//! which the (filtered) robot state is published (odom and accel) and pushes
//! them into its instance of the RppLocalizationEstimator. It exposes a
//! get_state method to offer the user the estimated state at a requested time.
//! This class offers the option to run this listener without the need to run a
//! separate node. If you do wish to run this functionality in a separate node,
//! consider the robot localization listener node.
//!
class RosRppLocalizationListener
{
public:
  //! @brief Constructor
  //!
  //! The RosRppLocalizationListener constructor initializes nodehandles,
  //! subscribers, a filter for synchronized listening to the topics it
  //! subscribes to, and an instance of the RppLocalizationEstimator.
  //!
  //! @param[in] node - rclcpp node shared pointer
  //!
  explicit RosRppLocalizationListener(
    rclcpp::Node::SharedPtr node,
    rclcpp::SubscriptionOptions options =
    detail::get_subscription_options_with_default_qos_override_policies());

  //! @brief Destructor
  //!
  //! Empty destructor
  //!
  ~RosRppLocalizationListener();

  //! @brief Get a state from the localization estimator
  //!
  //! Requests the predicted state and covariance at a given time from the
  //! Robot Localization Estimator.
  //!
  //! @param[in] time - time of the requested state
  //! @param[in] frame_id - frame id of which the state is requested.
  //! @param[out] state - state at the requested time
  //! @param[out] covariance - covariance at the requested time
  //!
  //! @return false if buffer is empty, true otherwise
  //!
  bool get_state(
    const double time, const std::string & frame_id,
    Eigen::VectorXd & state, Eigen::MatrixXd & covariance,
    std::string world_frame_id = "") const;

  //! @brief Get a state from the localization estimator
  //!
  //! Overload of get_state method for using ros::Time.
  //!
  //! @param[in] rclcpp_time - ros time of the requested state
  //! @param[in] frame_id - frame id of which the state is requested.
  //! @param[out] state - state at the requested time
  //! @param[out] covariance - covariance at the requested time
  //!
  //! @return false if buffer is empty, true otherwise
  //!
  bool get_state(
    const rclcpp::Time & rclcpp_time, const std::string & frame_id,
    Eigen::VectorXd & state, Eigen::MatrixXd & covariance,
    const std::string & world_frame_id = "") const;

  //!
  //! \brief getBaseFrameId Returns the base frame id of the localization
  //! listener
  //! \return The base frame id
  //!
  const std::string & getBaseFrameId() const;

  //!
  //! \brief getWorldFrameId Returns the world frame id of the localization
  //! listener
  //! \return The world frame id
  //!
  const std::string & getWorldFrameId() const;

private:
  //! @brief Callback for odom and accel
  //!
  //! Puts the information from the odom and accel messages in a Robot
  //! Localization Estimator state and sets the most
  //! recent state of the estimator.
  //!
  //! @param[in] odometry message
  //! @param[in] accel message
  //!
  void odomAndAccelCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry const> & odom,
    const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped const> &
    accel);

  //! @brief The core state estimator that facilitates inter- and
  //! extrapolation between buffered states.
  //!
  std::unique_ptr<RppLocalizationEstimator> estimator_;

  //! @brief Quality of service definitions
  //!
  rclcpp::QoS qos1_, qos10_;

  //! @brief Subscriber to the odometry state topic (output of a filter node)
  //!
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

  //! @brief Subscriber to the acceleration state topic (output of a filter
  //! node)
  //!
  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
  accel_sub_;

  //! @brief Waits for both an Odometry and an Accel message before calling a
  //! single callback function
  //!
  message_filters::TimeSynchronizer<nav_msgs::msg::Odometry,
    geometry_msgs::msg::AccelWithCovarianceStamped> sync_;

  //! @brief rclcpp interface to clock
  //!
  rclcpp::Clock::SharedPtr node_clock_;

  //! @brief rclcpp interface to logging
  //!
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger_;

  //! @brief Child frame id received from the Odometry message
  //!
  std::string base_frame_id_;

  //! @brief Frame id received from the odometry message
  //!
  std::string world_frame_id_;

  //! @brief Tf buffer for looking up transforms
  //!
  tf2_ros::Buffer tf_buffer_;

  //! @brief Transform listener to fill the tf_buffer
  //!
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS_RPP_LOCALIZATION_LISTENER_HPP_
