/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__ROS_FILTER_HPP_
#define RPP_LOCALIZATION__ROS_FILTER_HPP_

#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_state.hpp"
#include "rpp_localization/srv/toggle_filter_processing.hpp"
#include "rpp_localization/srv/set_pose.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/filters/nav_filter.hpp"

#include "rpp_localization/ros/ros_filter_base.hpp"

namespace rpp_localization
{


using MeasurementQueue =
  std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
    Measurement>;
using MeasurementHistoryDeque = std::deque<MeasurementPtr>;
using FilterStateHistoryDeque = std::deque<FilterStatePtr>;

template<typename T>
class RosFilter : public RosFilterBase<T>
{
public:
  //! @brief Constructor
  //!
  //! The RosFilter constructor makes sure that anyone using
  //! this template is doing so with the correct object type
  //!
  explicit RosFilter(const rclcpp::NodeOptions & options);

  //! @brief Destructor
  //!
  //! Clears out the message filters and topic subscribers.
  //!
  ~RosFilter();


  //! @brief initialize the filter
  //!
  void init();

  //! @brief Resets the filter to its initial state
  //!
  void reset();

  //! @brief Service callback to toggle processing measurements for a standby
  //! mode but continuing to publish
  //! @param[in] request - The state requested, on (True) or off (False)
  //! @param[out] response - status if upon success
  //! @return boolean true if successful, false if not
  //!
  void toggleFilterProcessingCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<
      rpp_localization::srv::ToggleFilterProcessing::Request> req,
    const std::shared_ptr<
      rpp_localization::srv::ToggleFilterProcessing::Response> resp);

  //! @brief Loads all parameters from file
  //!
  void loadParams();

  //! @brief callback function which is called for periodic updates
  //!
  void periodicUpdate();


  //! @brief Service callback for resetting the filter to its initial state. Parameters are unused.
  //!
  void resetSrvCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>);

  //! @brief Callback method for manually setting/resetting the internal pose
  //! estimate
  //! @param[in] msg - The ROS stamped pose with covariance message to take in
  //!
  void setPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  //! @brief Service callback for manually setting/resetting the internal pose
  //! estimate
  //!
  //! @param[in] request - Custom service request with pose information
  //! @return true if successful, false if not
  bool setPoseSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<rpp_localization::srv::SetPose::Request> request,
    std::shared_ptr<rpp_localization::srv::SetPose::Response> response);

  //! @brief Service callback for manually enable the filter
  //! @param[in] request - N/A
  //! @param[out] response - N/A
  //! @return boolean true if successful, false if not
  bool enableFilterSrvCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>);

protected:


  //! @brief Whether we publish the acceleration
  //!
  bool publish_acceleration_;

  //! @brief Whether we publish the transform from the world_frame to the
  //! base_link_frame
  //!
  bool publish_transform_;


  //! @brief Start the Filter disabled at startup
  //!
  //! If this is true, the filter reads parameters and prepares publishers and subscribes
  //! but does not integrate new messages into the state vector.
  //! The filter can be enabled later using a service.
  bool disabled_at_startup_;

  //! @brief Whether the filter is enabled or not. See disabledAtStartup_.
  bool enabled_;

  //! @brief Whether we'll allow old measurements to cause a re-publication of the updated state
  bool permit_corrected_publication_;

  //! @brief Vector to hold our subscribers until they go out of scope
  //!
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subs_;

  //! @brief Service that allows another node to toggle on/off filter
  //! processing while still publishing.
  //! Uses a rpp_localization ToggleFilterProcessing service.
  //!
  rclcpp::Service<rpp_localization::srv::ToggleFilterProcessing>::SharedPtr
    toggle_filter_processing_srv_;

  //! @brief Subscribes to the control input topic
  //!
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;

  //! @brief Subscribes to the set_pose topic (usually published from rviz).
  //! Message type is geometry_msgs/PoseWithCovarianceStamped.
  //!
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    set_pose_sub_;

  //! @brief Service that allows another node to change the current state and
  //! recieve a confirmation. Uses a custom SetPose service.
  //!
  rclcpp::Service<rpp_localization::srv::SetPose>::SharedPtr
    set_pose_service_;

  //! @brief Service that allows another node to enable the filter. Uses a
  //! standard Empty service.
  //!
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_filter_srv_;

  //! @brief Service that resets the filter to its initial state
  //!
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

  //! @brief Transform listener for receiving transforms
  //!
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //! @brief broadcaster of worldTransform tfs
  //!
  std::shared_ptr<tf2_ros::TransformBroadcaster> world_transform_broadcaster_;


  //! @brief Position publisher
  //!
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

  //! Acceleration publisher
  //!
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    accel_pub_;


  //! @brief Timer for filter updates
  //!
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS_FILTER_HPP_
