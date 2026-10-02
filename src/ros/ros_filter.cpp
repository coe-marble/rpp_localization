#include "rpp_localization/ros/ros_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"
#include "rpp_localization/filters/unscented_kalman_filter.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace rpp_localization
{
using namespace std::chrono_literals;

RosFilter::RosFilter(
  const rclcpp::NodeOptions & options, std::string default_configuration)
: RosFilterBase(options, true, std::move(default_configuration)),
  publish_acceleration_(false),
  publish_transform_(true),
  disabled_at_startup_(false),
  enabled_(false),
  permit_corrected_publication_(false)
{
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(this->_tf_buffer->get_buffer());
}

RosFilter::~RosFilter()
{
  this->_tf_buffer.reset();
  topic_subs_.clear();
  timer_.reset();
  set_pose_sub_.reset();
  control_sub_.reset();
  tf_listener_.reset();
  world_transform_broadcaster_.reset();
  set_pose_service_.reset();
  accel_pub_.reset();
  position_pub_.reset();
}

void RosFilter::init()
{
  RosFilterBase::init();

  world_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    RosFilterBase::shared_from_this());

  load_params();

  if (this->print_diagnostics_) {
    this->diagnostic_updater_->add(
      "Filter diagnostic updater",
      std::bind(&RosFilterBase::aggregate_diagnostics, this, std::placeholders::_1));
  }

  // Init the last measurement time so we don't get a huge initial delta
  this->set_rpp_filter_last_measurement_time(ros::to_timestamp_ns(this->now()));

  // Position publisher
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  this->position_pub_ =
    rclcpp::Node::create_publisher<nav_msgs::msg::Odometry>(
    "odometry/filtered", rclcpp::QoS(10), publisher_options);

  // Optional acceleration publisher
  if (publish_acceleration_) {
    this->accel_pub_ =
      rclcpp::Node::create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "accel/filtered", rclcpp::QoS(10), publisher_options);
  }

  const std::chrono::duration<double> timespan{1.0 / this->frequency_};
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
    this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
    std::bind(&RosFilter::periodic_update, this), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void RosFilter::reset()
{
  RosFilterBase::reset();
}

void RosFilter::reset_srv_callback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received a request to reset filter.");

  reset();
}

void RosFilter::toggle_filter_processing_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<
    rpp_localization::srv::ToggleFilterProcessing::Request> req,
  const std::shared_ptr<
    rpp_localization::srv::ToggleFilterProcessing::Response> resp)
{
  if (req->on == this->toggled_on_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Service was called to toggle filter processing but state was already as "
      "requested.");
    resp->status = false;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Toggling filter measurement filtering to %s.", req->on ? "On" : "Off");
    this->toggled_on_ = req->on;
    resp->status = true;
  }
}



void RosFilter::load_params()
{
  this->load_filter_params();

  // Whether we're publshing the world_frame->base_link_frame transform
  this->publish_transform_ = this->declare_parameter("publish_tf", true);

  // Whether we're publishing the acceleration state transform
  this->publish_acceleration_ = this->declare_parameter("publish_acceleration", false);

  // Whether we'll allow old measurements to cause a re-publication of the updated state
  this->permit_corrected_publication_ = this->declare_parameter("permit_corrected_publication", false);

  // Check if the filter should start or not
  this->disabled_at_startup_ = rclcpp::Node::declare_parameter<bool>("disabled_at_startup", false);
  this->enabled_ = !this->disabled_at_startup_;



  ///// CREATE SUBSCRIBERS AND SERVICES /////

  // Create a subscriber for manually setting/resetting pose
  this->set_pose_sub_ =
    rclcpp::Node::create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "set_pose", rclcpp::QoS(1),
    std::bind(&RosFilter::set_pose_callback, this, std::placeholders::_1));

  // Create a service for manually setting/resetting pose
  this->set_pose_service_ =
    rclcpp::Node::create_service<rpp_localization::srv::SetPose>(
    "set_pose", std::bind(
      &RosFilter::set_pose_srv_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create a service for manually enabling the filter
  this->enable_filter_srv_ =
    rclcpp::Node::create_service<std_srvs::srv::Empty>(
    "enable", std::bind(
      &RosFilter::enable_filter_srv_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create a service for manually setting/resetting pose
  this->reset_srv_ =
    rclcpp::Node::create_service<std_srvs::srv::Empty>(
    "reset", std::bind(
      &RosFilter::reset_srv_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create a service for toggling processing new measurements while still
  // publishing
  this->toggle_filter_processing_srv_ =
    rclcpp::Node::create_service<rpp_localization::srv::ToggleFilterProcessing>(
    "toggle", std::bind(
      &RosFilter::toggle_filter_processing_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  // If we're using control, set the parameters and create the necessary
  // subscribers
  if (this->rpp_filter_uses_control()) {
    this->control_sub_ = rclcpp::Node::create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&RosFilter::control_callback, this, std::placeholders::_1));
  }

  std::vector<CallbackData> pose_callback_data_v;
  std::vector<CallbackData> twist_callback_data_v;
  std::vector<CallbackData> acc_callback_data_v;

  std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&)>
  on_registered_odom = [this](const std::string& topic_name, const std::string& topic, int queue_size,
      const CallbackData& pose_callback_data, const CallbackData& twist_callback_data)
  {
    std::function<void(const std::shared_ptr<nav_msgs::msg::Odometry>)>
    odom_callback = std::bind(
      &RosFilterBase::odometry_callback, this,
      std::placeholders::_1, topic_name,
      pose_callback_data, twist_callback_data);
    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
    this->topic_subs_.push_back(
      rclcpp::Node::create_subscription<nav_msgs::msg::Odometry>(
        topic, custom_qos,
        odom_callback));

  };
  auto shared_this = RosFilterBase::shared_from_this();
  ros_filter_utilities::handle_odom_params(*shared_this,
    &this->_debug_stream,
    pose_callback_data_v,
    twist_callback_data_v,
    on_registered_odom
  );

  std::function<void(const std::string&, const std::string&, int, const CallbackData&)>
  on_registered_pose = [this](const std::string& topic, const std::string&, int queue_size,
      const CallbackData& pose_callback_data)
  {
    std::function<void(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>)>
    pose_callback = std::bind(
      &RosFilterBase::pose_callback, this,
      std::placeholders::_1,
      pose_callback_data,
      this->world_frame_id_, this->base_link_frame_id_, false);
    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
    this->topic_subs_.push_back(
      rclcpp::Node::create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, custom_qos,
        pose_callback));

  };

  ros_filter_utilities::handle_pose_params(*shared_this,
    &this->_debug_stream,
    pose_callback_data_v,
    on_registered_pose
  );


  std::function<void(const std::string&, const std::string&, int, const CallbackData&)>
  on_registered_twist = [this](const std::string& topic, const std::string&, int queue_size,
      const CallbackData& twist_callback_data)
  {
    std::function<void(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>)>
    twist_callback = std::bind(
      &RosFilterBase::twist_callback, this,
      std::placeholders::_1,
      twist_callback_data, this->base_link_frame_id_);
    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
    this->topic_subs_.push_back(
      rclcpp::Node::create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        topic, custom_qos,
        twist_callback));

  };

  ros_filter_utilities::handle_twist_params(*shared_this,
    &this->_debug_stream,
    twist_callback_data_v,
    on_registered_twist
  );



  std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&, const CallbackData&)>
  on_registered_imu = [this](const std::string& topic_name, const std::string& topic, int queue_size,
      const CallbackData& pose_callback_data, const CallbackData& twist_callback_data, const CallbackData& acc_callback_data)
  {
    std::function<void(const std::shared_ptr<sensor_msgs::msg::Imu>)>
    imu_callback = std::bind(
      &RosFilterBase::imu_callback, this,
      std::placeholders::_1,
      topic_name, pose_callback_data, twist_callback_data, acc_callback_data);
    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
    this->topic_subs_.push_back(
      rclcpp::Node::create_subscription<sensor_msgs::msg::Imu>(
        topic, custom_qos,
        imu_callback));

  };

  auto control_update_vector = this->rpp_filter_control_update_vector();
  ros_filter_utilities::handle_imu_params(*shared_this,
    &this->_debug_stream, control_update_vector,
    this->remove_gravitational_acceleration_,
    pose_callback_data_v,
    twist_callback_data_v,
    acc_callback_data_v,
    on_registered_imu
  );

  this->count_var_counts(pose_callback_data_v, twist_callback_data_v, acc_callback_data_v);

  ros_filter_utilities::warn_if_misconfigured(
    *shared_this,
    this->two_d_mode_,
    this->state_variable_names_,
    this->_abs_pose_var_counts,
    this->_twist_var_counts);

}


void RosFilter::periodic_update()
{
  // Wait for the filter to be enabled
  if (!this->enabled_) {
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "Filter is disabled. To enable it call the %s service",
      this->enable_filter_srv_->get_service_name());
    return;
  }

  rclcpp::Time cur_time = this->now();

  if (this->toggled_on_) {
    // Now we'll integrate any measurements we've received if requested,
    // and update angular acceleration.
    this->integrate_measurements(cur_time);
    this->differentiate_measurements(cur_time);
  } else {
    // Clear out measurements since we're not currently processing new entries
    this->clear_measurement_queue();

    // Reset last measurement time so we don't get a large time delta on toggle
    if (this->rpp_filter_initialized()) {
      this->set_rpp_filter_last_measurement_time(ros::to_timestamp_ns(this->now()));
    }
  }

  // Get latest state and publish it
  auto filtered_position = std::make_unique<nav_msgs::msg::Odometry>();

  bool corrected_data = false;

  if (this->get_filtered_odometry_message(filtered_position.get())) {
    this->world_base_link_trans_msg_.header.stamp =
      static_cast<rclcpp::Time>(filtered_position->header.stamp) + this->tf_time_offset_;
    this->world_base_link_trans_msg_.header.frame_id =
      filtered_position->header.frame_id;
    this->world_base_link_trans_msg_.child_frame_id =
      filtered_position->child_frame_id;

    this->world_base_link_trans_msg_.transform.translation.x =
      filtered_position->pose.pose.position.x;
    this->world_base_link_trans_msg_.transform.translation.y =
      filtered_position->pose.pose.position.y;
    this->world_base_link_trans_msg_.transform.translation.z =
      filtered_position->pose.pose.position.z;
    this->world_base_link_trans_msg_.transform.rotation =
      filtered_position->pose.pose.orientation;

    // The filtered_position is the message containing the state and covariances:
    // nav_msgs Odometry
    if (!this->validate_filter_output(filtered_position.get())) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Critical Error, NaNs were detected in the output state of the filter. "
        "This was likely due to poorly coniditioned process, noise, or sensor "
        "covariances.");
    }

    // If we're trying to publish with the same time stamp, it means that we had a measurement get
    // inserted into the filter history, and our state estimate was updated after it was already
    // published. As of ROS Noetic, TF2 will issue warnings whenever this occurs, so we make this
    // behavior optional. Just for safety, we also check for the condition where the last published
    // stamp is *later* than this stamp. This should never happen, but we should handle the case
    // anyway.
    corrected_data = (!this->permit_corrected_publication_ &&
      this->last_published_stamp_ >= filtered_position->header.stamp);

    // If the world_frame_id_ is the odom_frame_id_ frame, then we can just
    // send the transform. If the world_frame_id_ is the map_frame_id_ frame,
    // we'll have some work to do.
    if (this->publish_transform_ && !corrected_data) {
      if (filtered_position->header.frame_id == this->odom_frame_id_) {
        this->world_transform_broadcaster_->sendTransform(this->world_base_link_trans_msg_);
      } else if (filtered_position->header.frame_id == this->map_frame_id_) {
        try {
          tf2::Transform world_base_link_trans;
          tf2::fromMsg(
            this->world_base_link_trans_msg_.transform,
            world_base_link_trans);

          tf2::Transform base_link_odom_trans;
          tf2::fromMsg(
            this->_tf_buffer->lookup_transform(
              this->base_link_frame_id_,
              this->odom_frame_id_,
              tf2::TimePointZero)
            .transform,
            base_link_odom_trans);

          /*
           * First, see these two references:
           * http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform
           * http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction
           * We have a transform from map_frame_id_->base_link_frame_id_, but
           * it would actually transform a given pose from
           * base_link_frame_id_->map_frame_id_. We then used lookupTransform,
           * whose first two arguments are target frame and source frame, to
           * get a transform from base_link_frame_id_->odom_frame_id_.
           * However, this transform would actually transform data from
           * odom_frame_id_->base_link_frame_id_. Now imagine that we have a
           * position in the map_frame_id_ frame. First, we multiply it by the
           * inverse of the map_frame_id_->baseLinkFrameId, which will
           * transform that data from map_frame_id_ to base_link_frame_id_.
           * Now we want to go from base_link_frame_id_->odom_frame_id_, but
           * the transform we have takes data from
           * odom_frame_id_->base_link_frame_id_, so we need its inverse as
           * well. We have now transformed our data from map_frame_id_ to
           * odom_frame_id_. However, if we want other users to be able to do
           * the same, we need to broadcast the inverse of that entire
           * transform.
           */
          tf2::Transform map_odom_trans;
          map_odom_trans.mult(world_base_link_trans, base_link_odom_trans);

          geometry_msgs::msg::TransformStamped map_odom_trans_msg;
          map_odom_trans_msg.transform = tf2::toMsg(map_odom_trans);
          map_odom_trans_msg.header.stamp =
            static_cast<rclcpp::Time>(filtered_position->header.stamp) + this->tf_time_offset_;
          map_odom_trans_msg.header.frame_id = this->map_frame_id_;
          map_odom_trans_msg.child_frame_id = this->odom_frame_id_;

          world_transform_broadcaster_->sendTransform(map_odom_trans_msg);
        } catch (...) {
          RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5.0,
            "Could not obtain transform from " << this->odom_frame_id_ << "->" << this->base_link_frame_id_);
        }
      } else {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Odometry message frame_id was " << filtered_position->header.frame_id <<
            ", expected " << this->map_frame_id_ << " or " << this->odom_frame_id_);
      }
    }

    // Retain the last published stamp so we can detect repeated transforms in future cycles
    this->last_published_stamp_ = filtered_position->header.stamp;

    // Fire off the position and the transform
    if (!corrected_data) {
      this->position_pub_->publish(std::move(filtered_position));
    }

    if (this->print_diagnostics_) {
      this->freq_diag_->tick();
    }
  }

  // Publish the acceleration if desired and filter is initialized
  auto filtered_acceleration = std::make_unique<geometry_msgs::msg::AccelWithCovarianceStamped>();
  if (!corrected_data && this->publish_acceleration_ &&
    this->get_filtered_accel_message(filtered_acceleration.get()))
  {
    this->accel_pub_->publish(std::move(filtered_acceleration));
  }

  /* Diagnostics can behave strangely when playing back from bag
   * files and using simulated time, so we have to check for
   * time suddenly moving backwards as well as the standard
   * timeout criterion before publishing. */

  double diag_duration = (cur_time - this->last_diag_time_).nanoseconds();
  if (this->print_diagnostics_ &&
    (diag_duration >= this->diagnostic_updater_->getPeriod().nanoseconds() ||
    diag_duration < 0.0))
  {
    this->diagnostic_updater_->force_update();
    this->last_diag_time_ = cur_time;
  }

  // Clear out expired history data
  if (this->smooth_lagged_data_) {
    this->clear_expired_history(
      this->rpp_filter_last_measurement_time() -
      ros::to_duration_ns(this->history_length_));
  }

  // Warn the user if the update took too long
  const double loop_elapsed = (this->now() - cur_time).seconds();
  if (loop_elapsed > 1. / this->frequency_) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Failed to meet update rate! Took " << std::setprecision(20) << loop_elapsed << "seconds. "
        "Try decreasing the rate, limiting sensor output frequency, or limiting the number of "
        "sensors.");
  }
}

void RosFilter::set_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RF_DEBUG(
    "------ RosFilter::set_pose_callback ------\nPose message:\n" << msg);

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Received set_pose request with value\n" << geometry_msgs::msg::to_yaml(*msg));

  std::string topic_name("set_pose");

  // Get rid of any initial poses (pretend we've never had a measurement)
  this->initial_measurements_.clear();
  this->previous_measurements_.clear();
  this->previous_measurement_covariances_.clear();

  this->clear_measurement_queue();

  this->filter_state_history_.clear();
  this->measurement_history_.clear();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  this->last_set_pose_time_ = msg->header.stamp;

  // Set the state vector to the reported pose
  Eigen::VectorXd measurement(STATE_SIZE);
  Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);
  std::vector<bool> update_vector(STATE_SIZE, true);

  // We only measure pose variables, so initialize the vector to 0
  measurement.setZero();

  // Set this to the identity and let the message reset it
  measurement_covariance.setIdentity();
  measurement_covariance *= 1e-6;

  // Prepare the pose data (really just using this to transform it into the
  // target frame). Twist data is going to get zeroed out.
  // Since pose messages do not provide a child_frame_id, it defaults to baseLinkFrameId_
  this->prepare_pose(
    msg, topic_name, this->world_frame_id_, this->base_link_frame_id_, false, false, false,
    update_vector, measurement, measurement_covariance);

  // For the state
  this->set_rpp_filter_state(measurement);
  this->set_rpp_filter_covariance(measurement_covariance);

  this->set_rpp_filter_last_measurement_time(ros::to_timestamp_ns(this->now()));

  RF_DEBUG("\n------ /RosFilter::set_pose_callback ------\n");
}

bool RosFilter::set_pose_srv_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<rpp_localization::srv::SetPose::Request> request,
  std::shared_ptr<rpp_localization::srv::SetPose::Response>/*response*/)
{
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
    request->pose);
  this->set_pose_callback(msg);

  return true;
}

bool RosFilter::enable_filter_srv_callback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RF_DEBUG(
    "\n[" << this->get_name() << ":]" <<
      " ------ /RosFilter::enable_filter_srv_callback ------\n");
  if (enabled_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[%s:] Asking for enabling filter service, "
      "but the filter was already enabled! Use param disabled_at_startup.",
      this->get_name());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s:] Enabling filter...",
      this->get_name());
    enabled_ = true;
  }

  return true;
}


}  // namespace rpp_localization

