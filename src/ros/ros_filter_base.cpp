#include "rpp_localization/ros/ros_filter_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rpp_cpp/context_builder.hpp>
#include <rpp_cpp/data_manager.hpp>
#include <rpp_cpp/rpp_paths.hpp>

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
#include "rpp_localization/core/validation.hpp"
#include "rpp_localization/ros/localization_script.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"


namespace rpp_localization
{
using namespace std::chrono_literals;

RosFilterBase::RosFilterBase(
  const rclcpp::NodeOptions & options, const bool online,
  std::string default_configuration)
: Node(options.arguments()[0], options),
  print_diagnostics_(true),
  reset_on_time_jump_(false),
  smooth_lagged_data_(false),
  toggled_on_(true),
  two_d_mode_(false),
  dynamic_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  static_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  frequency_(30.0),
  gravitational_acceleration_(9.80665),
  _sensor_timeout(rclcpp::Duration::from_nanoseconds(0)),
  _latest_control(),
  last_diag_time_(0, 0, RCL_ROS_TIME),
  last_published_stamp_(0, 0, RCL_ROS_TIME),
  history_length_(rclcpp::Duration::from_nanoseconds(0)),
  predict_to_current_time_(false),
  last_set_pose_time_(0, 0, RCL_ROS_TIME),
  tf_timeout_(rclcpp::Duration::from_nanoseconds(0)),
  tf_time_offset_(rclcpp::Duration::from_nanoseconds(0)),
  rpp_sensor_timeout_(rclcpp::Duration::from_nanoseconds(0))
{
  default_configuration_ = std::move(default_configuration);
  _latest_control.stamp = 0;
  _latest_control.control.setZero(TWIST_SIZE);

  this->_tf_buffer = std::make_unique<TfBufferWrapper>(this->get_clock(), online);
  state_variable_names_.push_back("X");
  state_variable_names_.push_back("Y");
  state_variable_names_.push_back("Z");
  state_variable_names_.push_back("ROLL");
  state_variable_names_.push_back("PITCH");
  state_variable_names_.push_back("YAW");
  state_variable_names_.push_back("X_VELOCITY");
  state_variable_names_.push_back("Y_VELOCITY");
  state_variable_names_.push_back("Z_VELOCITY");
  state_variable_names_.push_back("ROLL_VELOCITY");
  state_variable_names_.push_back("PITCH_VELOCITY");
  state_variable_names_.push_back("YAW_VELOCITY");
  state_variable_names_.push_back("X_ACCELERATION");
  state_variable_names_.push_back("Y_ACCELERATION");
  state_variable_names_.push_back("Z_ACCELERATION");

  reset_var_counts();
}

RosFilterBase::~RosFilterBase()
{
  diagnostic_updater_.reset();
  freq_diag_.reset();
}

void RosFilterBase::init()
{
  // first init nav filter
  initialize_rpp_filter();
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(
    shared_from_this());
  diagnostic_updater_->setHardwareID("none");

  // Set up the frequency diagnostic
  min_frequency_ = frequency_ - 2;
  max_frequency_ = frequency_ + 2;
  freq_diag_ =
    std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    "odometry/filtered",
    *diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(
      &min_frequency_,
      &max_frequency_, 0.1, 10));

  last_diag_time_ = this->now();

  angular_acceleration_cov_.resize(ORIENTATION_SIZE, ORIENTATION_SIZE);
  angular_acceleration_cov_.setZero();

  // Clear out the transforms
  world_base_link_trans_msg_.transform =
    tf2::toMsg(tf2::Transform::getIdentity());
}

void RosFilterBase::reset()
{
  // Get rid of any initial poses (pretend we've never had a measurement)
  initial_measurements_.clear();
  previous_measurements_.clear();
  previous_measurement_covariances_.clear();

  // clear tf buffer to avoid TF_OLD_DATA errors
  _tf_buffer->clear();
  clear_measurement_queue();

  filter_state_history_.clear();
  measurement_history_.clear();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  last_set_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_diag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  _latest_control_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_published_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // clear last message timestamp, so older messages will be accepted
  last_message_times_.clear();

  // reset filter to uninitialized state
  reset_rpp_filter();
}

// @todo: Replace with AccelWithCovarianceStamped
void RosFilterBase::acceleration_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const CallbackData & callback_data,
  const std::string & target_frame)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    return;
  }

  const std::string & topic_name = callback_data.topic_name_;

  RF_DEBUG(
    "------ RosFilterBase::acceleration_callback (" << topic_name <<
      ") ------\n")
  // "Twist message:\n" << *msg);

  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG("Update vector for " << topic_name << " is:\n" << topic_name);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the twist data for inclusion in the filter
    if (prepare_acceleration(
        msg, topic_name, target_frame, callback_data.relative_,
        update_vector_corrected, measurement,
        measurement_covariance))
    {
      // Store the measurement. Add an "acceleration" suffix so we know what
      // kind of measurement we're dealing with when we debug the core filter
      // logic.
      enqueue_measurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG(
        "Enqueued new measurement for " << topic_name <<
          "_acceleration\n");
    } else {
      RF_DEBUG(
        "Did *not* enqueue measurement for " << topic_name <<
          "_acceleration\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        "\n");
  } else {
    // else if (reset_on_time_jump_ && rclcpp::Time::isSimTime())
    //{
    //  reset();
    //}

    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may"
      " indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";

    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, topic_name +
      "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " <<
        topic_name << " is " <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        ", current message time is " <<
        ros::to_seconds(msg->header.stamp) << ".\n");
  }

  RF_DEBUG(
    "\n----- /RosFilterBase::acceleration_callback (" << topic_name <<
      ") ------\n");
}

void RosFilterBase::control_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped::SharedPtr twist_stamped_ptr =
    std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped_ptr->twist = *msg;
  twist_stamped_ptr->header.frame_id = base_link_frame_id_;
  twist_stamped_ptr->header.stamp = this->now();
  control_stamped_callback(twist_stamped_ptr);
}

void RosFilterBase::control_stamped_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id == base_link_frame_id_ ||
    msg->header.frame_id == "")
  {
    _latest_control.control(ControlMemberVx) = msg->twist.linear.x;
    _latest_control.control(ControlMemberVy) = msg->twist.linear.y;
    _latest_control.control(ControlMemberVz) = msg->twist.linear.z;
    _latest_control.control(ControlMemberVroll) = msg->twist.angular.x;
    _latest_control.control(ControlMemberVpitch) = msg->twist.angular.y;
    _latest_control.control(ControlMemberVyaw) = msg->twist.angular.z;
    _latest_control.stamp = ros::to_timestamp_ns(rclcpp::Time(msg->header.stamp));

    // Update the filter with this control term
    set_rpp_filter_control(_latest_control);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5.0, "Commanded velocities "
      " must be given in the robot's body frame (" << base_link_frame_id_ <<
        "). Message frame was " << msg->header.frame_id);
  }
}

void RosFilterBase::enqueue_measurement(
  const std::string & topic_name, const Eigen::VectorXd & measurement,
  const Eigen::MatrixXd & measurement_covariance,
  const std::vector<bool> & update_vector, const double mahalanobis_thresh,
  const rclcpp::Time & time)
{
  MeasurementPtr meas = MeasurementPtr(new Measurement());

  meas->topic_name_ = topic_name;
  meas->measurement_ = measurement;
  meas->covariance_ = measurement_covariance;
  meas->update_vector_ = update_vector;
  meas->time_ = ros::to_timestamp_ns(time);
  meas->mahalanobis_thresh_ = mahalanobis_thresh;
  meas->latest_control_ = _latest_control;
  measurement_queue_.push(meas);
}

void RosFilterBase::force_two_d(
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  measurement(StateMemberZ) = 0.0;
  measurement(StateMemberRoll) = 0.0;
  measurement(StateMemberPitch) = 0.0;
  measurement(StateMemberVz) = 0.0;
  measurement(StateMemberVroll) = 0.0;
  measurement(StateMemberVpitch) = 0.0;
  measurement(StateMemberAz) = 0.0;

  measurement_covariance(StateMemberZ, StateMemberZ) = 1e-6;
  measurement_covariance(StateMemberRoll, StateMemberRoll) = 1e-6;
  measurement_covariance(StateMemberPitch, StateMemberPitch) = 1e-6;
  measurement_covariance(StateMemberVz, StateMemberVz) = 1e-6;
  measurement_covariance(StateMemberVroll, StateMemberVroll) = 1e-6;
  measurement_covariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
  measurement_covariance(StateMemberAz, StateMemberAz) = 1e-6;

  update_vector[StateMemberZ] = 1;
  update_vector[StateMemberRoll] = 1;
  update_vector[StateMemberPitch] = 1;
  update_vector[StateMemberVz] = 1;
  update_vector[StateMemberVroll] = 1;
  update_vector[StateMemberVpitch] = 1;
  update_vector[StateMemberAz] = 1;
}

bool RosFilterBase::get_filtered_odometry_message(nav_msgs::msg::Odometry * message)
{
  // If the filter has received a measurement at some point...
  if (rpp_filter_initialized()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = rpp_filter_state();
    const Eigen::MatrixXd & estimate_error_covariance =
      rpp_filter_covariance();

    // Convert from roll, pitch, and yaw back to quaternion for
    // orientation values
    tf2::Quaternion quat;
    quat.setRPY(
      state(StateMemberRoll), state(StateMemberPitch),
      state(StateMemberYaw));

    // Fill out the message
    message->pose.pose.position.x = state(StateMemberX);
    message->pose.pose.position.y = state(StateMemberY);
    message->pose.pose.position.z = state(StateMemberZ);
    message->pose.pose.orientation.x = quat.x();
    message->pose.pose.orientation.y = quat.y();
    message->pose.pose.orientation.z = quat.z();
    message->pose.pose.orientation.w = quat.w();
    message->twist.twist.linear.x = state(StateMemberVx);
    message->twist.twist.linear.y = state(StateMemberVy);
    message->twist.twist.linear.z = state(StateMemberVz);
    message->twist.twist.angular.x = state(StateMemberVroll);
    message->twist.twist.angular.y = state(StateMemberVpitch);
    message->twist.twist.angular.z = state(StateMemberVyaw);

    // Our covariance matrix layout doesn't quite match
    for (size_t i = 0; i < POSE_SIZE; i++) {
      for (size_t j = 0; j < POSE_SIZE; j++) {
        message->pose.covariance[POSE_SIZE * i + j] =
          estimate_error_covariance(i, j);
      }
    }

    // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a
    // few cycles to be meticulous and not index a twist covariance array on the
    // size of a pose covariance array
    for (size_t i = 0; i < TWIST_SIZE; i++) {
      for (size_t j = 0; j < TWIST_SIZE; j++) {
        message->twist.covariance[TWIST_SIZE * i + j] =
          estimate_error_covariance(
          i + POSITION_V_OFFSET,
          j + POSITION_V_OFFSET);
      }
    }

    message->header.stamp = ros::to_ros_time(rpp_filter_last_measurement_time());
    message->header.frame_id = world_frame_id_;
    message->child_frame_id = base_link_output_frame_id_;
  }

  return rpp_filter_initialized();
}

bool RosFilterBase::get_filtered_accel_message(
  geometry_msgs::msg::AccelWithCovarianceStamped * message)
{
  // If the filter has received a measurement at some point...
  if (rpp_filter_initialized()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = rpp_filter_state();
    const Eigen::MatrixXd & estimate_error_covariance =
      rpp_filter_covariance();

    //! Fill out the accel_msg
    message->accel.accel.linear.x = state(StateMemberAx);
    message->accel.accel.linear.y = state(StateMemberAy);
    message->accel.accel.linear.z = state(StateMemberAz);
    message->accel.accel.angular.x = angular_acceleration_.x();
    message->accel.accel.angular.y = angular_acceleration_.y();
    message->accel.accel.angular.z = angular_acceleration_.z();

    // Fill the covariance (only the left-upper matrix since we are not
    // estimating the rotational accelerations arround the axes
    for (size_t i = 0; i < ACCELERATION_SIZE; i++) {
      for (size_t j = 0; j < ACCELERATION_SIZE; j++) {
        // We use the POSE_SIZE since the accel cov matrix of ROS is 6x6
        message->accel.covariance[POSE_SIZE * i + j] = estimate_error_covariance(
          i + POSITION_A_OFFSET, j + POSITION_A_OFFSET);
      }
    }
    for (size_t i = ACCELERATION_SIZE; i < POSE_SIZE; i++) {
      for (size_t j = ACCELERATION_SIZE; j < POSE_SIZE; j++) {
        // fill out the angular portion. We assume the linear and angular portions are independent.
        message->accel.covariance[POSE_SIZE * i + j] =
          angular_acceleration_cov_(i - ACCELERATION_SIZE, j - ACCELERATION_SIZE);
      }
    }

    // Fill header information
    message->header.stamp = ros::to_ros_time(rpp_filter_last_measurement_time());
    message->header.frame_id = base_link_output_frame_id_;
  }

  return rpp_filter_initialized();
}

void RosFilterBase::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const std::string & topic_name,
  const CallbackData & pose_callback_data,
  const CallbackData & twist_callback_data,
  const CallbackData & accel_callback_data)
{
  RF_DEBUG(
    "------ RosFilterBase::imu_callback (" <<
      topic_name << ") ------\n")         // << "IMU message:\n" << *msg);

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp equal to or"
      " before the last filter reset, " << "this message will be ignored. This may"
      "indicate an empty or bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);


    RF_DEBUG(
      "Received message that preceded the most recent pose reset. "
      "Ignoring...");

    return;
  }

  // As with the odometry message, we can separate out the pose- and
  // twist-related variables in the IMU message and pass them to the pose and
  // twist callbacks (filters)
  if (pose_callback_data.update_sum_ > 0) {
    // Per the IMU message specification, if the IMU does not provide
    // orientation, then its first covariance value should be set to -1, and we
    // should ignore that portion of the message. rpp_localization allows
    // users to explicitly ignore data using its parameters, but we should also
    // be compliant with message specs.
    if (std::abs(msg->orientation_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "orientation. "
        "Ignoring orientation...");
    } else {
      // Extract the pose (orientation) data, pass it to its filter
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos_ptr =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pos_ptr->header = msg->header;
      pos_ptr->pose.pose.orientation = msg->orientation;

      // Copy the covariance for roll, pitch, and yaw
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          pos_ptr->pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->orientation_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      // IMU data gets handled a bit differently, since the message is ambiguous
      // and has only a single frame_id, even though the data in it is reported
      // in two different frames. As we assume users will specify a base_link to
      // imu transform, we make the target and child frame base_link_frame_id_ and
      // tell the pose_callback that it is working with IMU data. This will cause
      // it to apply different logic to the data.
      pose_callback(
        pos_ptr, pose_callback_data, base_link_frame_id_,
        base_link_frame_id_, true);
    }
  }

  if (twist_callback_data.update_sum_ > 0) {
    // Ignore rotational velocity if the first covariance value is -1
    if (std::abs(msg->angular_velocity_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "angular "
        "velocity. Ignoring angular velocity...");
    } else {
      // Repeat for velocity
      geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
        std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
      twist_ptr->header = msg->header;
      twist_ptr->twist.twist.angular = msg->angular_velocity;

      // Copy the covariance
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          twist_ptr->twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      twist_callback(twist_ptr, twist_callback_data, base_link_frame_id_);
    }
  }

  if (accel_callback_data.update_sum_ > 0) {
    // Ignore linear acceleration if the first covariance value is -1
    if (std::abs(msg->linear_acceleration_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "linear "
        "acceleration. Ignoring linear acceleration...");
    } else {
      // Pass the message on
      acceleration_callback(msg, accel_callback_data, base_link_frame_id_);
    }
  }

  RF_DEBUG("\n----- /RosFilterBase::imu_callback (" << topic_name << ") ------\n");
}

void RosFilterBase::integrate_measurements(const rclcpp::Time & current_time)
{
  RF_DEBUG(
    "------ RosFilterBase::integrate_measurements ------\n\n"
    "Integration time is " <<
      std::setprecision(20) << ros::to_seconds(current_time) <<
      "\n" <<
      measurement_queue_.size() << " measurements in queue.\n");

  bool predict_to_current_time = predict_to_current_time_;
  const TimestampNs current_time_ns = ros::to_timestamp_ns(current_time);

  // If we have any measurements in the queue, process them
  if (!measurement_queue_.empty()) {
    // Check if the first measurement we're going to process is older than the
    // filter's last measurement. This means we have received an out-of-sequence
    // message (one with an old timestamp), and we need to revert both the
    // filter state and measurement queue to the first state that preceded the
    // time stamp of our first measurement.
    const MeasurementPtr & first_measurement = measurement_queue_.top();
    int restored_measurement_count = 0;
    if (smooth_lagged_data_ &&
      first_measurement->time_ < rpp_filter_last_measurement_time())
    {
      RF_DEBUG(
        "Received a measurement that was " <<
          nanoseconds_to_seconds(
          rpp_filter_last_measurement_time() -
          first_measurement->time_) <<
          " seconds in the past. Reverting filter state and "
          "measurement queue...");

      int original_count = static_cast<int>(measurement_queue_.size());
      const TimestampNs first_measurement_time = first_measurement->time_;
      const std::string first_measurement_topic =
        first_measurement->topic_name_;
      // revert_to may invalidate first_measurement
      if (!revert_to(first_measurement_time - 1)) {
        RF_DEBUG(
          "ERROR: history interval is too small to revert to time " <<
            nanoseconds_to_seconds(first_measurement_time) << "\n");
        // ROS_WARN_STREAM_DELAYED_THROTTLE(history_length_,
        //   "Received old measurement for topic " << first_measurement_topic <<
        //   ", but history interval is insufficiently sized. "
        //   "Measurement time is " << std::setprecision(20) <<
        //   first_measurement_time <<
        //   ", current time is " << current_time <<
        //   ", history length is " << history_length_ << ".");
        restored_measurement_count = 0;
      }

      restored_measurement_count =
        static_cast<int>(measurement_queue_.size()) - original_count;
    }

    while (!measurement_queue_.empty() && rclcpp::ok()) {
      MeasurementPtr measurement = measurement_queue_.top();

      // If we've reached a measurement that has a time later than now, it
      // should wait until a future iteration. Since measurements are stored in
      // a priority queue, all remaining measurements will be in the future.
      if (current_time_ns < measurement->time_) {
        break;
      }

      measurement_queue_.pop();

      // When we receive control messages, we call this directly in the control
      // callback. However, we also associate a control with each sensor message
      // so that we can support lagged smoothing. As we cannot guarantee that
      // the new control callback will fire before a new measurement, we should
      // only perform this operation if we are processing messages from the
      // history. Otherwise, we may get a new measurement, store the "old"
      // latest control, then receive a control, call set_control, and then
      // overwrite that value with this one (i.e., with the "old" control we
      // associated with the measurement).
      if (rpp_filter_uses_control() && restored_measurement_count > 0) {
        set_rpp_filter_control(measurement->latest_control_);
        restored_measurement_count--;
      }

      // This will call predict and, if necessary, correct
      process_rpp_measurement(*(measurement.get()));

      // Store old states and measurements if we're smoothing
      if (smooth_lagged_data_) {
        // Invariant still holds: measurementHistoryDeque_.back().time_ <
        // measurement_queue_.top().time_
        measurement_history_.push_back(measurement);

        // We should only save the filter state once per unique timstamp
        if (measurement_queue_.empty() ||
          measurement_queue_.top()->time_ !=
          rpp_filter_last_measurement_time())
        {
          save_filter_state();
        }
      }
    }
  } else if (rpp_filter_initialized()) {
    // In the event that we don't get any measurements for a long time,
    // we still need to continue to estimate our state. Therefore, we
    // should project the state forward here.
    DurationNs last_update_delta =
      current_time_ns - rpp_filter_last_measurement_time();

    // If we get a large delta, then continuously predict until
    if (last_update_delta >= ros::to_duration_ns(rpp_filter_sensor_timeout())) {
      predict_to_current_time = true;

      RF_DEBUG(
        "Sensor timeout! Last measurement time was " <<
          nanoseconds_to_seconds(rpp_filter_last_measurement_time()) <<
          ", current time is " << ros::to_seconds(current_time) <<
          ", delta is " << nanoseconds_to_seconds(last_update_delta) <<
          "\n");
    }
  } else {
    RF_DEBUG("Filter not yet initialized.\n");
  }

  if (rpp_filter_initialized() && predict_to_current_time) {
    DurationNs last_update_delta =
      current_time_ns - rpp_filter_last_measurement_time();

    rclcpp::Duration ros_last_update_delta =
      ros::to_ros_duration(last_update_delta);
    validate_rpp_filter_delta(ros_last_update_delta);
    predict_rpp_filter(current_time_ns, ros_last_update_delta.nanoseconds());

    // Update the last measurement time and last update time
    set_rpp_filter_last_measurement_time(
      rpp_filter_last_measurement_time() + last_update_delta);
  }

  RF_DEBUG("\n----- /RosFilterBase::integrate_measurements ------\n");
}

void RosFilterBase::differentiate_measurements(const rclcpp::Time & current_time)
{
  if (rpp_filter_initialized()) {
    const double time_now = ros::to_seconds(current_time);
    const double dt = time_now - last_diff_time_;
    const Eigen::VectorXd & state = rpp_filter_state();
    tf2::Vector3 new_state_twist_rot(
      state(StateMemberVroll),
      state(StateMemberVpitch),
      state(StateMemberVyaw));
    angular_acceleration_ = (new_state_twist_rot - last_state_twist_rot_) / dt;
    const Eigen::MatrixXd & cov = rpp_filter_covariance();
    for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
      for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
        angular_acceleration_cov_(i, j) =
          cov(i + ORIENTATION_V_OFFSET, j + ORIENTATION_V_OFFSET) * 2. /
          ( dt * dt );
      }
    }
    last_state_twist_rot_ = new_state_twist_rot;
    last_diff_time_ = time_now;
  }
}

void RosFilterBase::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg,
  const std::string & topic_name,
  const CallbackData & pose_callback_data,
  const CallbackData & twist_callback_data)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      ros::to_seconds(msg->header.stamp) << ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    RF_DEBUG(
      "Received message that preceded the most recent pose reset. "
      "Ignoring...");

    return;
  }

  RF_DEBUG(
    "------ RosFilterBase::odometry_callback (" <<
      topic_name << ") ------\n")         // << "Odometry message:\n" << *msg);

  if (pose_callback_data.update_sum_ > 0) {
    // Grab the pose portion of the message and pass it to the pose_callback
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pos_ptr->header = msg->header;
    pos_ptr->pose = msg->pose;  // Entire pose object, also copies covariance

    if (pose_callback_data.pose_use_child_frame_) {
      pose_callback(pos_ptr, pose_callback_data, world_frame_id_, msg->child_frame_id, false);
    } else {
      pose_callback(pos_ptr, pose_callback_data, world_frame_id_, base_link_frame_id_, false);
    }
  }

  if (twist_callback_data.update_sum_ > 0) {
    // Grab the twist portion of the message and pass it to the twist_callback
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
      std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_ptr->header = msg->header;
    twist_ptr->header.frame_id = msg->child_frame_id;
    twist_ptr->twist =
      msg->twist;   // Entire twist object, also copies covariance

    twist_callback(twist_ptr, twist_callback_data, base_link_frame_id_);
  }

  RF_DEBUG(
    "\n----- /RosFilterBase::odometry_callback (" << topic_name <<
      ") ------\n");
}

void RosFilterBase::pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const CallbackData & callback_data, const std::string & target_frame,
  const std::string & pose_source_frame, const bool imu_data)
{
  const std::string & topic_name = callback_data.topic_name_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      ros::to_seconds(msg->header.stamp) << ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    return;
  }

  RF_DEBUG(
    "------ RosFilterBase::pose_callback (" << topic_name << ") ------\n"
      "Pose message:\n" << msg);

  //  Put the initial value in the lastMessagTimes_ for this variable if it's
  //  empty
  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG(
      "Update vector for " << topic_name << " is:\n" <<
        callback_data.update_vector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the pose data for inclusion in the filter
    if (prepare_pose(
        msg, topic_name, target_frame, pose_source_frame, callback_data.differential_,
        callback_data.relative_, imu_data, update_vector_corrected,
        measurement, measurement_covariance))
    {
      // Store the measurement. Add a "pose" suffix so we know what kind of
      // measurement we're dealing with when we debug the core filter logic.
      enqueue_measurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topic_name << "\n");
    } else {
      RF_DEBUG("Did *not* enqueue measurement for " << topic_name << "\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        "\n");
  } else {
    // else if (reset_on_time_jump_ && rclcpp::Time::isSimTime())
    //{
    //  reset();
    // }

    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may "
      "indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " << topic_name << " is " <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        ", current message time is " << ros::to_seconds(msg->header.stamp) <<
        ".\n");
  }

  RF_DEBUG("\n----- /RosFilterBase::pose_callback (" << topic_name << ") ------\n");
}


void RosFilterBase::twist_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
  const CallbackData & callback_data, const std::string & target_frame)
{
  const std::string & topic_name = callback_data.topic_name_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      ros::to_seconds(msg->header.stamp) << ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    return;
  }

  RF_DEBUG(
    "------ RosFilterBase::twist_callback (" << topic_name << ") ------\n"
      "Twist message:\n" << msg);

  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG(
      "Update vector for " << topic_name << " is:\n" <<
        callback_data.update_vector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the twist data for inclusion in the filter
    if (prepare_twist(
        msg, topic_name, target_frame, update_vector_corrected,
        measurement, measurement_covariance))
    {
      // Store the measurement. Add a "twist" suffix so we know what kind of
      // measurement we're dealing with when we debug the core filter logic.
      enqueue_measurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topic_name << "_twist\n");
    } else {
      RF_DEBUG(
        "Did *not* enqueue measurement for " << topic_name <<
          "_twist\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        "\n");
  } else {
    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may "
      "indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec << ")";
    add_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " << topic_name << " is" <<
        ros::to_seconds(last_message_times_[topic_name]) <<
        ", current message time is " << ros::to_seconds(msg->header.stamp) <<
        ".\n");
  }

  RF_DEBUG("\n----- /RosFilterBase::twist_callback (" << topic_name << ") ------\n");
}

void RosFilterBase::add_diagnostic(
  const int errLevel,
  const std::string & topicAndClass,
  const std::string & message,
  const bool staticDiag)
{
  if (staticDiag) {
    static_diagnostics_[topicAndClass] = message;
    static_diag_error_level_ = std::max(static_diag_error_level_, errLevel);
  } else {
    dynamic_diagnostics_[topicAndClass] = message;
    dynamic_diag_error_level_ = std::max(dynamic_diag_error_level_, errLevel);
  }
}

void RosFilterBase::aggregate_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & wrapper)
{
  wrapper.clear();
  wrapper.clearSummary();

  int maxErrLevel = std::max(static_diag_error_level_, dynamic_diag_error_level_);

  // Report the overall status
  switch (maxErrLevel) {
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      wrapper.summary(
        maxErrLevel,
        "Erroneous data or settings detected for a "
        "rpp_localization state estimation this->");
      break;
    case
      diagnostic_msgs::msg::DiagnosticStatus::WARN: wrapper.summary(
        maxErrLevel,
        "Potentially erroneous data or settings detected for "
        "a rpp_localization state estimation this->");
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      wrapper.summary(
        maxErrLevel,
        "The state of the rpp_localization state estimation "
        "node is stale.");
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      wrapper.summary(
        maxErrLevel,
        "The rpp_localization state estimation node appears to "
        "be functioning properly.");
      break;
    default:
      break;
  }

  // Aggregate all the static messages
  for (std::map<std::string, std::string>::iterator diagIt =
    static_diagnostics_.begin(); diagIt != static_diagnostics_.end();
    ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }

  // Aggregate all the dynamic messages, then clear them
  for (std::map<std::string, std::string>::iterator diagIt =
    dynamic_diagnostics_.begin(); diagIt != dynamic_diagnostics_.end();
    ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }
  dynamic_diagnostics_.clear();

  // Reset the warning level for the dynamic diagnostic messages
  dynamic_diag_error_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
}

void RosFilterBase::copy_covariance(
  const double * arr, Eigen::MatrixXd & covariance,
  const std::string & topic_name,
  const std::vector<bool> & update_vector,
  const size_t offset, const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++) {
    for (size_t j = 0; j < dimension; j++) {
      covariance(i, j) = arr[dimension * i + j];

      if (print_diagnostics_) {
        std::string iVar = state_variable_names_[offset + i];

        if (covariance(i, j) > 1e3 && (update_vector[offset + i] ||
          update_vector[offset + j]))
        {
          std::string jVar = state_variable_names_[offset + j];

          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << (i == j ? iVar + " variance" : iVar + " and " +
          jVar + " covariance") <<
            ", the value is extremely large (" << covariance(i, j) << "), but "
            "the update vector for " << (i == j ? iVar : iVar + " and/or " + jVar) <<
            "is set to true. This may produce undesirable results.";

          add_diagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        } else if (update_vector[i] && i == j && covariance(i, j) == 0) {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << iVar << " variance, was zero. This"
            "will be replaced with a small value to maintain filter stability, "
            "but should be corrected at the message origin this->";

          add_diagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        } else if (update_vector[i] && i == j && covariance(i, j) < 0) {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << iVar << " variance, was"
            "negative. This will be replaced with a small positive value to maintain"
            "filter stability, but should be corrected at the message origin this->";

          add_diagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        }
      }
    }
  }
}

void RosFilterBase::copy_covariance(
  const Eigen::MatrixXd & covariance, double * arr,
  const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++) {
    for (size_t j = 0; j < dimension; j++) {
      arr[dimension * i + j] = covariance(i, j);
    }
  }
}

bool RosFilterBase::prepare_acceleration(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const std::string & topic_name,
  const std::string & target_frame,
  const bool relative,
  std::vector<bool> & update_vector,
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  RF_DEBUG(
    "------ RosFilterBase::prepare_acceleration (" << topic_name <<
      ") ------\n");

  // 1. Get the measurement into a vector
  tf2::Vector3 acc_tmp(msg->linear_acceleration.x, msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  // Set relevant header info
  std::string msg_frame =
    (msg->header.frame_id == "" ? base_link_frame_id_ : msg->header.frame_id);

  // 2. rpp_localization lets users configure which variables from the sensor
  // should be
  //    fused with the filter. This is specified at the sensor level. However,
  //    the data may go through transforms before being fused with the state
  //    estimate. In that case, we need to know which of the transformed
  //    variables came from the pre-transformed "approved" variables (i.e., the
  //    ones that had "true" in their xxx_config parameter). To do this, we
  //    create a pose from the original upate vector, which contains only zeros
  //    and ones. This pose goes through the same transforms as the measurement.
  //    The non-zero values that result will be used to modify the
  //    update_vector.
  tf2::Matrix3x3 maskAcc(update_vector[StateMemberAx], 0, 0, 0,
    update_vector[StateMemberAy], 0, 0, 0,
    update_vector[StateMemberAz]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covariance_rotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
  covariance_rotated.setZero();

  this->copy_covariance(
    &(msg->linear_acceleration_covariance[0]),
    covariance_rotated, topic_name, update_vector,
    POSITION_A_OFFSET, ACCELERATION_SIZE);

  RF_DEBUG(
    "Original measurement as tf object: " <<
      acc_tmp << "\nOriginal update vector:\n" <<
      update_vector << "\nOriginal covariance matrix:\n" <<
      covariance_rotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  // It's unlikely that we'll get a velocity measurement in another frame, but
  // we have to handle the situation.
  tf2::Transform target_frame_trans;
  bool can_transform = _tf_buffer->lookup_transform_safe(
    target_frame, msg_frame, msg->header.stamp, tf_timeout_,
    target_frame_trans);

  if (can_transform) {
    const Eigen::VectorXd & state = rpp_filter_state();

    // Transform to correct frame, prior to removal of gravity.
    tf2::Vector3 state_twist_rot(
      state(StateMemberVroll),
      state(StateMemberVpitch),
      state(StateMemberVyaw));
    acc_tmp = target_frame_trans.getBasis() * acc_tmp +
      target_frame_trans.getOrigin().cross(angular_acceleration_) -
      target_frame_trans.getOrigin().cross(state_twist_rot).cross(
      state_twist_rot);

    // We don't know if the user has already handled the removal
    // of normal forces, so we use a parameter
    if (remove_gravitational_acceleration_[topic_name]) {
      tf2::Vector3 normAcc(0, 0, gravitational_acceleration_);
      tf2::Transform trans;
      tf2::Vector3 rotNorm;

      if (std::abs(msg->orientation_covariance[0] + 1) < 1e-9) {
        // Imu message contains no orientation, so we should use orientation
        // from filter state to transform and remove acceleration
        tf2::Matrix3x3 stateTmp;
        stateTmp.setRPY(
          state(StateMemberRoll),
          state(StateMemberPitch),
          state(StateMemberYaw));

        // transform state orientation to IMU frame
        trans.setBasis(stateTmp * target_frame_trans.getBasis());
        rotNorm = trans.getBasis().inverse() * normAcc;
      } else {
        tf2::Quaternion curAttitude;
        tf2::fromMsg(msg->orientation, curAttitude);
        if (std::abs(curAttitude.length() - 1.0) > 0.01) {
          RCLCPP_WARN_ONCE(
            get_logger(),
            "An input was not normalized, this should NOT happen, but will normalize.");
          curAttitude.normalize();
        }
        trans.setRotation(curAttitude);
        if (!relative) {
          // curAttitude is the true world-frame attitude of the sensor
          rotNorm = target_frame_trans.getBasis() * trans.getBasis().inverse() * normAcc;
        } else {
          // curAttitude is relative to the initial pose of the sensor.
          // Assumption: IMU sensor is rigidly attached to the base_link
          // (but a static rotation is possible).
          rotNorm = target_frame_trans.getBasis().inverse() * trans.getBasis().inverse() * normAcc;
        }
      }
      acc_tmp.setX(acc_tmp.getX() - rotNorm.getX());
      acc_tmp.setY(acc_tmp.getY() - rotNorm.getY());
      acc_tmp.setZ(acc_tmp.getZ() - rotNorm.getZ());

      RF_DEBUG(
        "Orientation is " <<
          trans.getRotation() << "Acceleration due to gravity is " << rotNorm <<
          "After removing acceleration due to gravity, acceleration is " <<
          acc_tmp << "\n");
    }

    maskAcc = target_frame_trans.getBasis() * maskAcc;

    // Now use the mask values to determine which update vector values should be
    // true
    update_vector[StateMemberAx] = static_cast<int>(
      maskAcc.getRow(StateMemberAx - POSITION_A_OFFSET).length() >= 1e-6);
    update_vector[StateMemberAy] = static_cast<int>(
      maskAcc.getRow(StateMemberAy - POSITION_A_OFFSET).length() >= 1e-6);
    update_vector[StateMemberAz] = static_cast<int>(
      maskAcc.getRow(StateMemberAz - POSITION_A_OFFSET).length() >= 1e-6);

    RF_DEBUG(
      msg->header.frame_id <<
        "->" << target_frame << " transform:\n" <<
        target_frame_trans << "\nAfter applying transform to " <<
        target_frame << ", update vector is:\n" <<
        update_vector << "\nAfter applying transform to " <<
        target_frame << ", measurement is:\n" <<
        acc_tmp << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(target_frame_trans.getRotation());
    Eigen::MatrixXd rot3d(ACCELERATION_SIZE, ACCELERATION_SIZE);
    rot3d.setIdentity();

    for (size_t r_ind = 0; r_ind < ACCELERATION_SIZE; ++r_ind) {
      rot3d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot3d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot3d(r_ind, 2) = rot.getRow(r_ind).getZ();
    }

    // Carry out the rotation
    covariance_rotated = rot3d * covariance_rotated.eval() * rot3d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covariance_rotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberAx) = acc_tmp.getX();
    measurement(StateMemberAy) = acc_tmp.getY();
    measurement(StateMemberAz) = acc_tmp.getZ();

    // Copy the covariances
    measurement_covariance.block(
      POSITION_A_OFFSET, POSITION_A_OFFSET,
      ACCELERATION_SIZE, ACCELERATION_SIZE) =
      covariance_rotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

    // 7. Handle 2D mode
    if (two_d_mode_) {
      force_two_d(measurement, measurement_covariance, update_vector);
    }
  } else {
    RF_DEBUG(
      "Could not transform measurement into " << target_frame <<
        ". Ignoring...\n");
  }

  RF_DEBUG(
    "\n----- /RosFilterBase::prepare_acceleration(" << topic_name <<
      ") ------\n");

  return can_transform;
}

bool RosFilterBase::prepare_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const std::string & topic_name, const std::string & target_frame,
  const std::string & source_frame,
  const bool differential, const bool relative, const bool imu_data,
  std::vector<bool> & update_vector, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  bool retVal = false;

  RF_DEBUG("------ RosFilterBase::prepare_pose (" << topic_name << ") ------\n");

  // 1. Get the measurement into a tf-friendly transform (pose) object
  tf2::Stamped<tf2::Transform> pose_tmp;

  // We'll need this later for storing this measurement for differential
  // integration
  tf2::Transform cur_measurement;

  // Handle issues where frame_id data is not filled out properly
  // @todo: verify that this is necessary still. New IMU handling may
  // have rendered this obsolete.
  std::string final_target_frame;
  if (target_frame == "") {
    if (msg->header.frame_id == "") {
      // Blank target and message frames mean we can just
      // use our world_frame
      final_target_frame = world_frame_id_;
      pose_tmp.frame_id_ = final_target_frame;
    } else {
      // Under the (target_frame == "") condition,
      // a blank target frame means we shouldn't bother
      // transforming the data
      final_target_frame = msg->header.frame_id;
      pose_tmp.frame_id_ = final_target_frame;
    }
  } else {
    // Otherwise, we should use our target frame
    final_target_frame = target_frame;
    pose_tmp.frame_id_ =
      (differential && !imu_data ? final_target_frame : msg->header.frame_id);
  }

  RF_DEBUG(
    "Final target frame for " << topic_name << " is " <<
      final_target_frame << "\n");

  pose_tmp.stamp_ = tf2::timeFromSec(
    static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.nanosec) / 1000000000.0);

  // Fill out the position data
  pose_tmp.setOrigin(
    tf2::Vector3(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z));

  tf2::Quaternion orientation;

  // Handle bad (empty) quaternions
  if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
    msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
  {
    orientation.setValue(0.0, 0.0, 0.0, 1.0);

    if (update_vector[StateMemberRoll] ||
      update_vector[StateMemberPitch] ||
      update_vector[StateMemberYaw])
    {
      std::stringstream stream;
      stream << "The " << topic_name <<
        " message contains an invalid orientation quaternion, " <<
        "but its configuration is such that orientation data is being used."
        " Correcting...";

      add_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        topic_name + "_orientation", stream.str(), false);
    }
  } else {
    tf2::fromMsg(msg->pose.pose.orientation, orientation);
    if (std::abs(orientation.length() - 1.0) > 0.01) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "An input was not normalized, this should NOT happen, but will normalize.");
      orientation.normalize();
    }
  }

  // Fill out the orientation data
  pose_tmp.setRotation(orientation);

  // 2. Get the target frame transformation
  tf2::Transform target_frame_trans;
  bool can_transform = _tf_buffer->lookup_transform_safe(
    final_target_frame, pose_tmp.frame_id_,
    rclcpp::Time(tf2::timeToSec(pose_tmp.stamp_)), tf_timeout_,
    target_frame_trans);

  // handling multiple odometry origins: convert to the origin adherent to base_link.
  // make pose refer to the baseLinkFrame as source
  tf2::Transform source_frame_trans;
  bool can_src_transform = false;
  if (source_frame != base_link_frame_id_) {
    can_src_transform = _tf_buffer->lookup_transform_safe(
      source_frame, base_link_frame_id_,
      rclcpp::Time(tf2::timeToSec(pose_tmp.stamp_)), tf_timeout_,
      source_frame_trans);
  }

  // 3. Make sure we can work with this data before carrying on
  if (can_transform) {
    /* 4. rpp_localization lets users configure which variables from the
     * sensor should be fused with the filter. This is specified at the sensor
     * level. However, the data may go through transforms before being fused
     * with the state estimate. In that case, we need to know which of the
     * transformed variables came from the pre-transformed "approved" variables
     * (i.e., the ones that had "true" in their xxx_config parameter). To do
     * this, we construct matrices using the update vector values on the
     * diagonals, pass this matrix through the rotation, and use the length of
     * each row to determine the transformed update vector. The process is
     * slightly different for IMUs, as the coordinate frame transform is really
     * the base_link->imu_frame transform, and not a transform from some other
     * world-fixed frame (even though the IMU data itself *is* reported in a
     * world fixed frame). */
    tf2::Matrix3x3 mask_position(update_vector[StateMemberX], 0, 0, 0,
      update_vector[StateMemberY], 0, 0, 0,
      update_vector[StateMemberZ]);

    tf2::Matrix3x3 mask_orientation(update_vector[StateMemberRoll], 0, 0, 0,
      update_vector[StateMemberPitch], 0, 0, 0,
      update_vector[StateMemberYaw]);

    if (imu_data) {
      /* We have to treat IMU orientation data differently. Even though we are
       * dealing with pose data when we work with orientations, for IMUs, the
       * frame_id is the frame in which the sensor is mounted, and not the
       * coordinate frame of the IMU. Imagine an IMU that is mounted facing
       * sideways. The pitch in the IMU frame becomes roll for the vehicle. This
       * means that we need to rotate roll and pitch angles by the IMU's
       * mounting yaw offset, and we must apply similar treatment to its update
       * mask and covariance.
       * */

      double dummy, yaw;
      target_frame_trans.getBasis().getRPY(dummy, dummy, yaw);
      tf2::Matrix3x3 trans_tmp;
      trans_tmp.setRPY(0.0, 0.0, yaw);

      mask_position = trans_tmp * mask_position;
      mask_orientation = trans_tmp * mask_orientation;
    } else {
      mask_position = target_frame_trans.getBasis() * mask_position;
      mask_orientation = target_frame_trans.getBasis() * mask_orientation;
    }

    // Now copy the mask values back into the update vector: any row with a
    // significant vector length indicates that we want to set that variable to
    // true in the update vector.
    update_vector[StateMemberX] = static_cast<int>(
      mask_position.getRow(StateMemberX - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberY] = static_cast<int>(
      mask_position.getRow(StateMemberY - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberZ] = static_cast<int>(
      mask_position.getRow(StateMemberZ - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberRoll] = static_cast<int>(
      mask_orientation.getRow(StateMemberRoll - ORIENTATION_OFFSET)
      .length() >= 1e-6);
    update_vector[StateMemberPitch] = static_cast<int>(
      mask_orientation.getRow(StateMemberPitch - ORIENTATION_OFFSET)
      .length() >= 1e-6);
    update_vector[StateMemberYaw] = static_cast<int>(
      mask_orientation.getRow(StateMemberYaw - ORIENTATION_OFFSET).length() >=
      1e-6);

    // 5a. We'll need to rotate the covariance as well. Create a container and
    // copy over the covariance data
    Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
    covariance.setZero();
    copy_covariance(
      &(msg->pose.covariance[0]), covariance, topic_name,
      update_vector, POSITION_OFFSET, POSE_SIZE);

    // 5b. Now rotate the covariance: create an augmented matrix that
    // contains a 3D rotation matrix in the upper-left and lower-right
    // quadrants, with zeros elsewhere.
    tf2::Matrix3x3 rot;
    Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
    rot6d.setIdentity();
    Eigen::MatrixXd covariance_rotated;

    // Transform pose covariance due to a different pose source origin
    if (can_src_transform) {
      // (source_frame != base_link_frame_id_) already satisfied
      rot.setRotation(source_frame_trans.getRotation());
      for (size_t r_ind = 0; r_ind < POSITION_SIZE; ++r_ind) {
        // let's borrow rot6d here...
        rot6d(r_ind, 0) = rot.getRow(r_ind).getX();
        rot6d(r_ind, 1) = rot.getRow(r_ind).getY();
        rot6d(r_ind, 2) = rot.getRow(r_ind).getZ();
        rot6d(r_ind + POSITION_SIZE, 3) = rot.getRow(r_ind).getX();
        rot6d(r_ind + POSITION_SIZE, 4) = rot.getRow(r_ind).getY();
        rot6d(r_ind + POSITION_SIZE, 5) = rot.getRow(r_ind).getZ();
      }
      // since the transformation is a post-multiply
      covariance = rot6d.transpose() * covariance.eval() * rot6d;
    }
    // return rot6d to its initial state.
    rot6d.setIdentity();

    if (imu_data) {
      // Apply the same special logic to the IMU covariance rotation
      double dummy, yaw;
      target_frame_trans.getBasis().getRPY(dummy, dummy, yaw);
      rot.setRPY(0.0, 0.0, yaw);
    } else {
      rot.setRotation(target_frame_trans.getRotation());
    }

    for (size_t r_ind = 0; r_ind < POSITION_SIZE; ++r_ind) {
      rot6d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot6d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot6d(r_ind, 2) = rot.getRow(r_ind).getZ();
      rot6d(r_ind + POSITION_SIZE, 3) = rot.getRow(r_ind).getX();
      rot6d(r_ind + POSITION_SIZE, 4) = rot.getRow(r_ind).getY();
      rot6d(r_ind + POSITION_SIZE, 5) = rot.getRow(r_ind).getZ();
    }

    // Now carry out the rotation
    covariance_rotated = rot6d * covariance * rot6d.transpose();

    RF_DEBUG(
      "After rotating into the " << final_target_frame <<
        " frame, covariance is \n" <<
        covariance_rotated << "\n");

    /* 6a. For IMU data, the transform that we get is the transform from the
     * body frame of the robot (e.g., base_link) to the mounting frame of the
     * robot. It is *not* the coordinate frame in which the IMU orientation data
     * is reported. If the IMU is mounted in a non-neutral orientation, we need
     * to remove those offsets, and then we need to potentially "swap" roll and
     * pitch. Note that this transform does NOT handle NED->ENU conversions.
     * Data is assumed to be in the ENU frame when it is received.
     * */
    if (imu_data) {
      // First, convert the transform and measurement rotation to RPY
      // @todo: There must be a way to handle this with quaternions. Need to
      // look into it.
      double roll_offset = 0;
      double pitch_offset = 0;
      double yaw_offset = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      ros_filter_utilities::quat_to_rpy(
        target_frame_trans.getRotation(),
        roll_offset, pitch_offset, yaw_offset);
      ros_filter_utilities::quat_to_rpy(pose_tmp.getRotation(), roll, pitch, yaw);

      // 6b. Apply the offset (making sure to bound them), and throw them in a
      // vector
      tf2::Vector3 rpy_angles(
        angles::normalize_angle(roll - roll_offset),
        angles::normalize_angle(pitch - pitch_offset),
        angles::normalize_angle(yaw - yaw_offset));

      // 6c. Now we need to rotate the roll and pitch by the yaw offset value.
      // Imagine a case where an IMU is mounted facing sideways. In that case
      // pitch for the IMU's world frame is roll for the robot.
      tf2::Matrix3x3 mat;
      mat.setRPY(0.0, 0.0, yaw_offset);
      rpy_angles = mat * rpy_angles;
      pose_tmp.getBasis().setRPY(
        rpy_angles.getX(), rpy_angles.getY(),
        rpy_angles.getZ());

      // We will use this target transformation later on, but
      // we've already transformed this data as if the IMU
      // were mounted neutrall on the robot, so we can just
      // make the transform the identity.
      target_frame_trans.setIdentity();
    }

    // 7. Two cases: if we're in differential mode, we need to generate a twist
    // message. Otherwise, we just transform it to the target frame.
    if (differential) {
      bool success = false;

      // We're going to be playing with pose_tmp, so store it,
      // as we'll need to save its current value for the next
      // measurement.
      cur_measurement = pose_tmp;

      // Make sure we have previous measurements to work with
      if (previous_measurements_.count(topic_name) > 0 &&
        previous_measurement_covariances_.count(topic_name) > 0)
      {
        // 7a. If we are carrying out differential integration and
        // we have a previous measurement for this sensor,then we
        // need to apply the inverse of that measurement to this new
        // measurement to produce a "delta" measurement between the two.
        // Even if we're not using all of the variables from this sensor,
        // we need to use the whole measurement to determine the delta
        // to the new measurement
        tf2::Transform prev_measurement = previous_measurements_[topic_name];
        pose_tmp.setData(prev_measurement.inverseTimes(pose_tmp));

        RF_DEBUG(
          "Previous measurement:\n" <<
            previous_measurements_[topic_name] <<
            "\nAfter removing previous measurement, measurement delta is:\n" <<
            pose_tmp << "\n");

        // 7b. Now we we have a measurement delta in the frame_id of the
        // message, but we want that delta to be in the target frame, so
        // we need to apply the rotation of the target frame transform.
        target_frame_trans.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        pose_tmp.mult(target_frame_trans, pose_tmp);

        RF_DEBUG(
          "After rotating to the target frame, measurement delta is:\n" <<
            pose_tmp << "\n");

        // 7c. Now use the time difference from the last message to compute
        // translational and rotational velocities
        double dt = ros::to_seconds(msg->header.stamp) -
          ros::to_seconds(last_message_times_[topic_name]);
        double xVel = pose_tmp.getOrigin().getX() / dt;
        double yVel = pose_tmp.getOrigin().getY() / dt;
        double zVel = pose_tmp.getOrigin().getZ() / dt;

        double rollVel = 0;
        double pitchVel = 0;
        double yawVel = 0;

        ros_filter_utilities::quat_to_rpy(
          pose_tmp.getRotation(), rollVel,
          pitchVel, yawVel);
        rollVel /= dt;
        pitchVel /= dt;
        yawVel /= dt;

        RF_DEBUG(
          "Previous message time was " <<
            ros::to_seconds(last_message_times_[topic_name]) <<
            ", current message time is " <<
            ros::to_seconds(msg->header.stamp) << ", delta is " <<
            dt << ", velocity is (vX, vY, vZ): (" << xVel << ", " <<
            yVel << ", " << zVel << ")\n" <<
            "(vRoll, vPitch, vYaw): (" << rollVel << ", " << pitchVel <<
            ", " << yawVel << ")\n");

        // 7d. Fill out the velocity data in the message
        geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
          std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
        twist_ptr->header = msg->header;
        twist_ptr->header.frame_id = source_frame;
        twist_ptr->twist.twist.linear.x = xVel;
        twist_ptr->twist.twist.linear.y = yVel;
        twist_ptr->twist.twist.linear.z = zVel;
        twist_ptr->twist.twist.angular.x = rollVel;
        twist_ptr->twist.twist.angular.y = pitchVel;
        twist_ptr->twist.twist.angular.z = yawVel;
        std::vector<bool> twist_update_vec(STATE_SIZE, false);
        std::copy(
          update_vector.begin() + POSITION_OFFSET,
          update_vector.begin() + POSE_SIZE,
          twist_update_vec.begin() + POSITION_V_OFFSET);
        std::copy(
          twist_update_vec.begin(), twist_update_vec.end(),
          update_vector.begin());

        // 7e. Now rotate the previous covariance for this measurement to get it
        // into the target frame, and add the current measurement's rotated
        // covariance to the previous measurement's rotated covariance, and
        // multiply by the time delta.
        Eigen::MatrixXd prev_covar_rotated =
          rot6d * previous_measurement_covariances_[topic_name] *
          rot6d.transpose();
        covariance_rotated =
          (covariance_rotated.eval() + prev_covar_rotated) * dt;
        copy_covariance(
          covariance_rotated, &(twist_ptr->twist.covariance[0]),
          POSE_SIZE);

        RF_DEBUG(
          "Previous measurement covariance:\n" <<
            previous_measurement_covariances_[topic_name] <<
            "\nPrevious measurement covariance rotated:\n" <<
            prev_covar_rotated << "\nFinal twist covariance:\n" <<
            covariance_rotated << "\n");

        // Now pass this on to prepare_twist, which will convert it to the
        // required frame
        success = prepare_twist(
          twist_ptr, topic_name + "_twist",
          base_link_frame_id_, update_vector,
          measurement, measurement_covariance);
      }

      // 7f. Update the previous measurement and measurement covariance
      previous_measurements_[topic_name] = cur_measurement;
      previous_measurement_covariances_[topic_name] = covariance;

      retVal = success;
    } else {
      // make pose refer to the baseLinkFrame as source
      // can_src_transform == true => ( sourceFrame != baseLinkFrameId_ )
      if (can_src_transform) {
        pose_tmp.setData(pose_tmp * source_frame_trans);
      }

      // 7g. If we're in relative mode, remove the initial measurement
      if (relative) {
        if (initial_measurements_.count(topic_name) == 0) {
          initial_measurements_.insert(
            std::pair<std::string, tf2::Transform>(topic_name, pose_tmp));
        }

        tf2::Transform initial_measurement = initial_measurements_[topic_name];
        pose_tmp.setData(initial_measurement.inverseTimes(pose_tmp));
      }

      // 7h. Apply the target frame transformation to the pose object.
      pose_tmp.mult(target_frame_trans, pose_tmp);
      pose_tmp.frame_id_ = final_target_frame;

      // 7i. Finally, copy everything into our measurement and covariance
      // objects
      measurement(StateMemberX) = pose_tmp.getOrigin().x();
      measurement(StateMemberY) = pose_tmp.getOrigin().y();
      measurement(StateMemberZ) = pose_tmp.getOrigin().z();

      // The filter needs roll, pitch, and yaw values instead of quaternions
      double roll, pitch, yaw;
      ros_filter_utilities::quat_to_rpy(pose_tmp.getRotation(), roll, pitch, yaw);
      measurement(StateMemberRoll) = roll;
      measurement(StateMemberPitch) = pitch;
      measurement(StateMemberYaw) = yaw;

      measurement_covariance.block(0, 0, POSE_SIZE, POSE_SIZE) =
        covariance_rotated.block(0, 0, POSE_SIZE, POSE_SIZE);

      // 8. Handle 2D mode
      if (two_d_mode_) {
        force_two_d(measurement, measurement_covariance, update_vector);
      }

      retVal = true;
    }
  } else {
    retVal = false;

    RF_DEBUG(
      "Could not transform measurement into " << final_target_frame <<
        ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilterBase::prepare_pose (" << topic_name << ") ------\n");

  return retVal;
}

bool RosFilterBase::prepare_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
  const std::string & topic_name, const std::string & target_frame,
  std::vector<bool> & update_vector, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  RF_DEBUG("------ RosFilterBase::prepare_twist (" << topic_name << ") ------\n");

  // 1. Get the measurement into two separate vector objects.
  tf2::Vector3 twist_lin(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  tf2::Vector3 meas_twist_rot(msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z);

  // 1a. This sensor may or may not measure rotational velocity. Regardless,
  // if it measures linear velocity, then later on, we'll need to remove "false"
  // linear velocity resulting from angular velocity and the translational
  // offset of the sensor from the vehicle origin.
  const Eigen::VectorXd & state = rpp_filter_state();
  tf2::Vector3 state_twist_rot(state(StateMemberVroll),
    state(StateMemberVpitch),
    state(StateMemberVyaw));

  // Determine the frame_id of the data
  std::string msg_frame =
    (msg->header.frame_id == "" ? target_frame : msg->header.frame_id);

  // 2. rpp_localization lets users configure which variables from the sensor
  // should be
  //    fused with the filter. This is specified at the sensor level. However,
  //    the data may go through transforms before being fused with the state
  //    estimate. In that case, we need to know which of the transformed
  //    variables came from the pre-transformed "approved" variables (i.e., the
  //    ones that had "true" in their xxx_config parameter). To do this, we
  //    construct matrices using the update vector values on the diagonals, pass
  //    this matrix through the rotation, and use the length of each row to
  //    determine the transformed update vector.
  tf2::Matrix3x3 maskLin(update_vector[StateMemberVx], 0, 0, 0,
    update_vector[StateMemberVy], 0, 0, 0,
    update_vector[StateMemberVz]);

  tf2::Matrix3x3 maskRot(update_vector[StateMemberVroll], 0, 0, 0,
    update_vector[StateMemberVpitch], 0, 0, 0,
    update_vector[StateMemberVyaw]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covariance_rotated(TWIST_SIZE, TWIST_SIZE);
  covariance_rotated.setZero();

  copy_covariance(
    &(msg->twist.covariance[0]), covariance_rotated, topic_name,
    update_vector, POSITION_V_OFFSET, TWIST_SIZE);

  RF_DEBUG(
    "Original measurement as tf object:\nLinear: " <<
      twist_lin << "Rotational: " << meas_twist_rot <<
      "\nOriginal update vector:\n" <<
      update_vector << "\nOriginal covariance matrix:\n" <<
      covariance_rotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  tf2::Transform target_frame_trans;
  bool can_transform = _tf_buffer->lookup_transform_safe(
    target_frame, msg_frame, msg->header.stamp, tf_timeout_,
    target_frame_trans);

  if (can_transform) {
    // Transform to correct frame. Note that we can get linear velocity
    // as a result of the sensor offset and rotational velocity
    meas_twist_rot = target_frame_trans.getBasis() * meas_twist_rot;
    twist_lin = target_frame_trans.getBasis() * twist_lin +
      target_frame_trans.getOrigin().cross(state_twist_rot);
    maskLin = target_frame_trans.getBasis() * maskLin;
    maskRot = target_frame_trans.getBasis() * maskRot;

    // Now copy the mask values back into the update vector
    update_vector[StateMemberVx] = static_cast<int>(
      maskLin.getRow(StateMemberVx - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVy] = static_cast<int>(
      maskLin.getRow(StateMemberVy - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVz] = static_cast<int>(
      maskLin.getRow(StateMemberVz - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVroll] = static_cast<int>(
      maskRot.getRow(StateMemberVroll - ORIENTATION_V_OFFSET).length() >=
      1e-6);
    update_vector[StateMemberVpitch] = static_cast<int>(
      maskRot.getRow(StateMemberVpitch - ORIENTATION_V_OFFSET).length() >=
      1e-6);
    update_vector[StateMemberVyaw] = static_cast<int>(
      maskRot.getRow(StateMemberVyaw - ORIENTATION_V_OFFSET).length() >=
      1e-6);

    RF_DEBUG(
      msg->header.frame_id <<
        "->" << target_frame << " transform:\n" <<
        target_frame_trans << "\nAfter applying transform to " <<
        target_frame << ", update vector is:\n" <<
        update_vector << "\nAfter applying transform to " <<
        target_frame << ", measurement is:\n" <<
        "Linear: " << twist_lin << "Rotational: " << meas_twist_rot <<
        "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(target_frame_trans.getRotation());
    Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
    rot6d.setIdentity();

    for (size_t r_ind = 0; r_ind < POSITION_SIZE; ++r_ind) {
      rot6d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot6d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot6d(r_ind, 2) = rot.getRow(r_ind).getZ();
      rot6d(r_ind + POSITION_SIZE, 3) = rot.getRow(r_ind).getX();
      rot6d(r_ind + POSITION_SIZE, 4) = rot.getRow(r_ind).getY();
      rot6d(r_ind + POSITION_SIZE, 5) = rot.getRow(r_ind).getZ();
    }

    // Carry out the rotation
    covariance_rotated = rot6d * covariance_rotated.eval() * rot6d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covariance_rotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberVx) = twist_lin.getX();
    measurement(StateMemberVy) = twist_lin.getY();
    measurement(StateMemberVz) = twist_lin.getZ();
    measurement(StateMemberVroll) = meas_twist_rot.getX();
    measurement(StateMemberVpitch) = meas_twist_rot.getY();
    measurement(StateMemberVyaw) = meas_twist_rot.getZ();

    // Copy the covariances
    measurement_covariance.block(
      POSITION_V_OFFSET, POSITION_V_OFFSET,
      TWIST_SIZE, TWIST_SIZE) =
      covariance_rotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

    // 7. Handle 2D mode
    if (two_d_mode_) {
      force_two_d(measurement, measurement_covariance, update_vector);
    }
  } else {
    RF_DEBUG(
      "Could not transform measurement into " << target_frame <<
        ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilterBase::prepare_twist (" << topic_name << ") ------\n");

  return can_transform;
}

void RosFilterBase::save_filter_state()
{
  FilterStatePtr state = std::make_shared<FilterState>();
  state->_state = rpp_filter_state();
  state->_estimate_error_covariance = rpp_filter_covariance();
  state->_last_measurement_time = rpp_filter_last_measurement_time();
  const auto & control = rpp_filter_control();
  state->_latest_control = control.control;
  state->_latest_control_time = control.stamp;
  filter_state_history_.push_back(state);
  RF_DEBUG(
    "Saved state with timestamp " <<
      std::setprecision(20) <<
      nanoseconds_to_seconds(state->_last_measurement_time) <<
      " to history. " << filter_state_history_.size() <<
      " measurements are in the queue.\n");
}

void RosFilterBase::clear_expired_history(const TimestampNs cutoff_time)
{
  RF_DEBUG(
    "\n----- RosFilterBase::clear_expired_history -----" <<
      "\nCutoff time is " << nanoseconds_to_seconds(cutoff_time) <<
      "\n");

  int popped_measurements = 0;
  int popped_states = 0;

  while (!measurement_history_.empty() &&
    measurement_history_.front()->time_ < cutoff_time)
  {
    measurement_history_.pop_front();
    popped_measurements++;
  }

  while (!filter_state_history_.empty() &&
    filter_state_history_.front()->_last_measurement_time < cutoff_time)
  {
    filter_state_history_.pop_front();
    popped_states++;
  }

  RF_DEBUG(
    "\nPopped " << popped_measurements << " measurements and " <<
      popped_states <<
      " states from their respective queues." <<
      "\n---- /RosFilterBase::clear_expired_history ----\n");
}


void RosFilterBase::reset_var_counts()
{
  _abs_pose_var_counts[StateMemberX] = 0;
  _abs_pose_var_counts[StateMemberY] = 0;
  _abs_pose_var_counts[StateMemberZ] = 0;
  _abs_pose_var_counts[StateMemberRoll] = 0;
  _abs_pose_var_counts[StateMemberPitch] = 0;
  _abs_pose_var_counts[StateMemberYaw] = 0;

  _twist_var_counts[StateMemberVx] = 0;
  _twist_var_counts[StateMemberVy] = 0;
  _twist_var_counts[StateMemberVz] = 0;
  _twist_var_counts[StateMemberVroll] = 0;
  _twist_var_counts[StateMemberVpitch] = 0;
  _twist_var_counts[StateMemberVyaw] = 0;

}

void RosFilterBase::count_var_counts(
  const std::vector<CallbackData>& pose_callback_data,
  const std::vector<CallbackData>& twist_callback_data,
  const std::vector<CallbackData>& acc_callback_data
)
{
  static_cast<void>(acc_callback_data);

 // COUNT
  for (const CallbackData& data : pose_callback_data)
  {
    int pose_update_sum =
      std::accumulate(data.update_vector_.begin(), data.update_vector_.end(), 0);
    const std::vector<bool>& pose_update_vec = data.update_vector_;
    if (pose_update_sum > 0) {
      if (data.differential_) {
        _twist_var_counts[StateMemberVx] += pose_update_vec[StateMemberX];
        _twist_var_counts[StateMemberVy] += pose_update_vec[StateMemberY];
        _twist_var_counts[StateMemberVz] += pose_update_vec[StateMemberZ];
        _twist_var_counts[StateMemberVroll] +=
          pose_update_vec[StateMemberRoll];
        _twist_var_counts[StateMemberVpitch] +=
          pose_update_vec[StateMemberPitch];
        _twist_var_counts[StateMemberVyaw] += pose_update_vec[StateMemberYaw];
      } else {
        _abs_pose_var_counts[StateMemberX] += pose_update_vec[StateMemberX];
        _abs_pose_var_counts[StateMemberY] += pose_update_vec[StateMemberY];
        _abs_pose_var_counts[StateMemberZ] += pose_update_vec[StateMemberZ];
        _abs_pose_var_counts[StateMemberRoll] +=
          pose_update_vec[StateMemberRoll];
        _abs_pose_var_counts[StateMemberPitch] +=
          pose_update_vec[StateMemberPitch];
        _abs_pose_var_counts[StateMemberYaw] +=
          pose_update_vec[StateMemberYaw];
      }
    }

  }

  for (const CallbackData& data : twist_callback_data)
  {
    int twist_update_sum =
      std::accumulate(data.update_vector_.begin(), data.update_vector_.end(), 0);

    const std::vector<bool>& twist_update_vec = data.update_vector_;
    if (twist_update_sum > 0) {
      _twist_var_counts[StateMemberVx] += twist_update_vec[StateMemberVx];
      _twist_var_counts[StateMemberVy] += twist_update_vec[StateMemberVx];
      _twist_var_counts[StateMemberVz] += twist_update_vec[StateMemberVz];
      _twist_var_counts[StateMemberVroll] +=
        twist_update_vec[StateMemberVroll];
      _twist_var_counts[StateMemberVpitch] +=
        twist_update_vec[StateMemberVpitch];
      _twist_var_counts[StateMemberVyaw] += twist_update_vec[StateMemberVyaw];
    }
  }

}


bool RosFilterBase::validate_filter_output(nav_msgs::msg::Odometry * message)
{
  return !std::isnan(message->pose.pose.position.x) &&
         !std::isinf(message->pose.pose.position.x) &&
         !std::isnan(message->pose.pose.position.y) &&
         !std::isinf(message->pose.pose.position.y) &&
         !std::isnan(message->pose.pose.position.z) &&
         !std::isinf(message->pose.pose.position.z) &&
         !std::isnan(message->pose.pose.orientation.x) &&
         !std::isinf(message->pose.pose.orientation.x) &&
         !std::isnan(message->pose.pose.orientation.y) &&
         !std::isinf(message->pose.pose.orientation.y) &&
         !std::isnan(message->pose.pose.orientation.z) &&
         !std::isinf(message->pose.pose.orientation.z) &&
         !std::isnan(message->pose.pose.orientation.w) &&
         !std::isinf(message->pose.pose.orientation.w) &&
         !std::isnan(message->twist.twist.linear.x) &&
         !std::isinf(message->twist.twist.linear.x) &&
         !std::isnan(message->twist.twist.linear.y) &&
         !std::isinf(message->twist.twist.linear.y) &&
         !std::isnan(message->twist.twist.linear.z) &&
         !std::isinf(message->twist.twist.linear.z) &&
         !std::isnan(message->twist.twist.angular.x) &&
         !std::isinf(message->twist.twist.angular.x) &&
         !std::isnan(message->twist.twist.angular.y) &&
         !std::isinf(message->twist.twist.angular.y) &&
         !std::isnan(message->twist.twist.angular.z) &&
         !std::isinf(message->twist.twist.angular.z);
}


void RosFilterBase::clear_measurement_queue()
{
  // Clear the measurement queue.
  // This prevents us from immediately undoing our reset.
  while (!measurement_queue_.empty() && rclcpp::ok()) {
    measurement_queue_.pop();
  }
}

bool RosFilterBase::revert_to(const TimestampNs time)
{
  RF_DEBUG("\n----- RosFilter::revert_to -----\n");
  RF_DEBUG(
    "\nRequested time was " << std::setprecision(20) <<
      nanoseconds_to_seconds(time) << "\n")

  // size_t history_size = filter_state_history_.size();

  // Walk back through the queue until we reach a filter state whose time stamp
  // is less than or equal to the requested time. Since every saved state after
  // that time will be overwritten/corrected, we can pop from the queue. If the
  // history is insufficiently short, we just take the oldest state we have.
  FilterStatePtr last_history_state;
  while (!filter_state_history_.empty() &&
    filter_state_history_.back()->_last_measurement_time > time)
  {
    last_history_state = filter_state_history_.back();
    filter_state_history_.pop_back();
  }

  // If the state history is not empty at this point, it means that our history
  // was large enough, and we should revert to the state at the back of the
  // history deque.
  bool ret_val = false;
  if (!filter_state_history_.empty()) {
    ret_val = true;
    last_history_state = filter_state_history_.back();
  } else {
    RF_DEBUG(
      "Insufficient history to revert to time " <<
        nanoseconds_to_seconds(time) << "\n");

    if (last_history_state) {
      RF_DEBUG(
        "Will revert to oldest state at " <<
          nanoseconds_to_seconds(last_history_state->_latest_control_time) <<
          ".\n");

      // ROS_WARN_STREAM_DELAYED_THROTTLE(history_length_, "Could not revert "
      //   "to state with time " << std::setprecision(20) << time <<
      //   ". Instead reverted to state with time " <<
      //   lastHistoryState->lastMeasurementTime_ << ". History size was " <<
      //   history_size);
    }
  }

  // If we have a valid reversion state, revert
  if (last_history_state) {
    // Reset filter to the latest state from the queue.
    const FilterStatePtr & state = last_history_state;
    set_rpp_filter_state(state->_state);
    set_rpp_filter_covariance(state->_estimate_error_covariance);
    set_rpp_filter_last_measurement_time(state->_last_measurement_time);

    RF_DEBUG(
      "Reverted to state with time " <<
        nanoseconds_to_seconds(state->_last_measurement_time) << "\n");

    // Repeat for measurements, but push every measurement onto the measurement
    // queue as we go
    int restored_measurements = 0;
    while (!measurement_history_.empty() &&
      measurement_history_.back()->time_ > time)
    {
      // Don't need to restore measurements that predate our earliest state time
      if (state->_last_measurement_time <= measurement_history_.back()->time_) {
        measurement_queue_.push(measurement_history_.back());
        restored_measurements++;
      }

      measurement_history_.pop_back();
    }

    RF_DEBUG(
      "Restored " << restored_measurements << " to measurement queue."
        "\n");
  }

  RF_DEBUG("\n----- /RosFilter::revert_to\n");

  return ret_val;
}

void RosFilterBase::load_filter_params()
{

  // Determine if we'll be printing diagnostic information
  this->print_diagnostics_ = this->declare_parameter("print_diagnostics", false);

  // Check for custom gravitational acceleration value
  this->gravitational_acceleration_ = this->declare_parameter(
    "gravitational_acceleration",
    this->gravitational_acceleration_);

  // Grab the debug param. If true, the node will produce a LOT of output.
  bool debug = this->declare_parameter("debug", false);
  std::string debug_out_file = "rpp_localization_debug.txt";
  if (debug) {
    try {
      debug_out_file = this->declare_parameter("debug_out_file", debug_out_file);
      this->_debug_stream.open(debug_out_file.c_str());

      // Make sure we succeeded
      if (this->_debug_stream.is_open()) {
        this->set_rpp_filter_debug(debug, &this->_debug_stream);
      } else {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "RosFilter::load_params() - unable to create debug output file " << debug_out_file);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "RosFilter::load_params() - unable to create debug output file " << debug_out_file <<
          ". Error was " << e.what());
    }
  }

  // These params specify the name of the robot's body frame (typically
  // base_link) and odometry frame (typically odom)
  this->map_frame_id_ = this->declare_parameter("map_frame", std::string("map"));
  this->odom_frame_id_ = this->declare_parameter("odom_frame", std::string("odom"));
  this->base_link_frame_id_ = this->declare_parameter(
    "base_link_frame",
    std::string("base_link"));
  this->base_link_output_frame_id_ = this->declare_parameter(
    "base_link_frame_output",
    this->base_link_frame_id_);

  /*
   * These parameters are designed to enforce compliance with REP-105:
   * http://www.ros.org/reps/rep-0105.html
   * When fusing absolute position data from sensors such as GPS, the state
   * estimate can undergo discrete jumps. According to REP-105, we want three
   * coordinate frames: map, odom, and base_link. The map frame can have
   * discontinuities, but is the frame with the most accurate position estimate
   * for the robot and should not suffer from drift. The odom frame drifts over
   * time, but is guaranteed to be continuous and is accurate enough for local
   * planning and navigation. The base_link frame is affixed to the robot. The
   * intention is that some odometry source broadcasts the odom->base_link
   * transform. The localization software should broadcast map->base_link.
   * However, tf does not allow multiple parents for a coordinate frame, so
   * we must *compute* map->base_link, but then use the existing odom->base_link
   * transform to compute *and broadcast* map->odom.
   *
   * The state estimation nodes in rpp_localization therefore have two
   * "modes." If your world_frame parameter value matches the odom_frame
   * parameter value, then rpp_localization will assume someone else is
   * broadcasting a transform from odom_frame->base_link_frame, and it will
   * compute the map_frame->odom_frame transform. Otherwise, it will simply
   * compute the odom_frame->base_link_frame transform.
   *
   * The default is the latter behavior (broadcast of odom->base_link).
   */
  this->world_frame_id_ = this->declare_parameter("world_frame", this->odom_frame_id_);

  if (this->map_frame_id_ == this->odom_frame_id_ ||
    this->odom_frame_id_ == this->base_link_frame_id_ ||
    this->map_frame_id_ == this->base_link_frame_id_ ||
    this->odom_frame_id_ == this->base_link_output_frame_id_ ||
    this->map_frame_id_ == this->base_link_output_frame_id_)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid frame configuration! The values for map_frame, "
      "odom_frame, and base_link_frame must be unique. If using a base_link_frame_output "
      "values, it must not match the map_frame or odom_frame.");
  }

  // Try to resolve tf_prefix
  std::string tf_prefix = "";
  std::string tf_prefix_path = "";
  this->declare_parameter("tf_prefix", rclcpp::PARAMETER_STRING);
  if (this->get_parameter("tf_prefix", tf_prefix_path)) {
    // Append the tf prefix in a tf2-friendly manner
    filter_utilities::append_prefix(tf_prefix, this->map_frame_id_);
    filter_utilities::append_prefix(tf_prefix, this->odom_frame_id_);
    filter_utilities::append_prefix(tf_prefix, this->base_link_frame_id_);
    filter_utilities::append_prefix(tf_prefix, this->base_link_output_frame_id_);
    filter_utilities::append_prefix(tf_prefix, this->world_frame_id_);
  }


  // Transform future dating
  double offset_tmp = this->declare_parameter("transform_time_offset", 0.0);
  this->tf_time_offset_ = rclcpp::Duration::from_seconds(offset_tmp);

  // Transform timeout
  double timeout_tmp = this->declare_parameter("transform_timeout", 0.0);
  this->tf_timeout_ = rclcpp::Duration::from_seconds(timeout_tmp);

  // Update frequency and sensor timeout
  this->frequency_ = this->declare_parameter("frequency", 30.0);

  this->predict_to_current_time_ = rclcpp::Node::declare_parameter<bool>("predict_to_current_time", false);

  this->_sensor_timeout =
    rclcpp::Duration::from_seconds(rclcpp::Node::declare_parameter("sensor_timeout", 1.0 / this->frequency_));
  this->set_rpp_filter_sensor_timeout(this->_sensor_timeout);

  // Determine if we're in 2D mode
  this->two_d_mode_ = rclcpp::Node::declare_parameter("two_d_mode", false);

  // Smoothing window size
  this->smooth_lagged_data_ = rclcpp::Node::declare_parameter("smooth_lagged_data", false);
  double history_length_double = rclcpp::Node::declare_parameter("history_length", 0.0);

  if (!this->smooth_lagged_data_ && std::abs(history_length_double) > 0) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Filter history interval of " << history_length_double <<
        " specified, but smooth_lagged_data is set to false. Lagged data will not be smoothed.");
  }

  if (this->smooth_lagged_data_ && history_length_double < 0) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Negative history interval of " << history_length_double << " specified. Absolute value "
        "will be assumed.");
  }

  this->history_length_ = rclcpp::Duration::from_seconds(std::abs(history_length_double));

  // Whether we reset filter on jump back in time
  this->reset_on_time_jump_ = rclcpp::Node::declare_parameter("reset_on_time_jump", false);

  // Debugging writes to file
  RF_DEBUG(
    std::boolalpha <<
      "tf_prefix is " << tf_prefix <<
      "\nmap_frame is " << this->map_frame_id_ <<
      "\nodom_frame is " << this->odom_frame_id_ <<
      "\nbase_link_frame is " << this->base_link_frame_id_ <<
      "\nbase_link_output_frame is " << this->base_link_output_frame_id_ <<
      "\nworld_frame is " << this->world_frame_id_ <<
      "\ntransform_time_offset is " << ros::to_seconds(this->tf_time_offset_) <<
      "\ntransform_timeout is " << ros::to_seconds(this->tf_timeout_) <<
      "\nfrequency is " << this->frequency_ <<
      "\nsensor_timeout is " << ros::to_seconds(this->rpp_filter_sensor_timeout()) <<
      "\ntwo_d_mode is " << (this->two_d_mode_ ? "true" : "false") <<
      "\nsmooth_lagged_data is " << (this->smooth_lagged_data_ ? "true" : "false") <<
      "\nhistory_length is " << ros::to_seconds(this->history_length_) <<
      "\ninitial state is " << this->rpp_filter_state() <<
      "\nprint_diagnostics is " << this->print_diagnostics_ << "\n");
}





namespace
{

constexpr double k_initial_covariance = 1e-9;

struct ScriptReference
{
  std::string library;
  std::string name;
};

ScriptReference parse_script_reference(const std::string & reference)
{
  const auto separator = reference.find("::");
  if (separator == std::string::npos || separator == 0 ||
    separator + 2 >= reference.size() ||
    reference.find("::", separator + 2) != std::string::npos)
  {
    throw std::invalid_argument(
            "script must use the library::script_name format");
  }

  return {
    reference.substr(0, separator),
    reference.substr(separator + 2)};
}

void require_valid_state(const Eigen::VectorXd & state)
{
  if (state.size() != STATE_SIZE || !state.allFinite())
  {
    throw std::invalid_argument("state must contain 15 finite values");
  }
}

void require_valid_covariance(const Eigen::MatrixXd & covariance)
{
  if (covariance.rows() != STATE_SIZE || covariance.cols() != STATE_SIZE ||
    !covariance.allFinite())
  {
    throw std::invalid_argument("covariance must be a finite 15 by 15 matrix");
  }

  for (Eigen::Index index = 0; index < covariance.rows(); ++index)
  {
    if (covariance(index, index) < 0.0)
    {
      throw std::invalid_argument("covariance diagonal entries must be non-negative");
    }
  }
}

template<typename Status>
void require_ok(const Status & status, const std::string & operation)
{
  if (status.code() != 0)
  {
    throw std::runtime_error(operation + ": " + status.message());
  }
}

LocalizationFilter15::Estimate15 make_estimate(
  const StateVector & state,
  const CovarianceMatrix & covariance,
  const TimestampNs reference_time)
{
  require_valid_state(state);
  require_valid_covariance(covariance);

  LocalizationFilter15::Estimate15 estimate;
  auto state_values = estimate.state().values();
  state_values.resize(STATE_SIZE);
  for (Eigen::Index index = 0; index < STATE_SIZE; ++index)
  {
    state_values[static_cast<std::size_t>(index)] = state(index);
  }

  auto covariance_values = estimate.covariance().values();
  covariance_values.resize(STATE_SIZE * STATE_SIZE);
  for (Eigen::Index row = 0; row < STATE_SIZE; ++row)
  {
    for (Eigen::Index column = 0; column < STATE_SIZE; ++column)
    {
      covariance_values[static_cast<std::size_t>(row * STATE_SIZE + column)] =
        covariance(row, column);
    }
  }
  estimate.referenceTimeNs() = reference_time;
  return estimate;
}

LocalizationFilter15::Measurement15 make_measurement(const Measurement & measurement)
{
  require_valid_state(measurement.measurement_);
  if (measurement.covariance_.rows() != STATE_SIZE ||
    measurement.covariance_.cols() != STATE_SIZE || !measurement.covariance_.allFinite())
  {
    throw std::invalid_argument("measurement covariance must be a finite 15 by 15 matrix");
  }
  if (measurement.update_vector_.size() != static_cast<std::size_t>(STATE_SIZE))
  {
    throw std::invalid_argument("measurement update vector must contain 15 values");
  }

  LocalizationFilter15::Measurement15 output;
  auto state_values = output.state().values();
  state_values.resize(STATE_SIZE);
  auto covariance_values = output.covariance().values();
  covariance_values.resize(STATE_SIZE * STATE_SIZE);
  auto update_mask = output.updateMask();
  update_mask.resize(STATE_SIZE);

  for (Eigen::Index row = 0; row < STATE_SIZE; ++row)
  {
    const auto index = static_cast<std::size_t>(row);
    state_values[index] = measurement.measurement_(row);
    update_mask[index] = measurement.update_vector_[index];
    for (Eigen::Index column = 0; column < STATE_SIZE; ++column)
    {
      covariance_values[static_cast<std::size_t>(row * STATE_SIZE + column)] =
        measurement.covariance_(row, column);
    }
  }
  output.referenceTimeNs() = measurement.time_;
  output.mahalanobisThreshold() = measurement.mahalanobis_thresh_;
  output.sourceName() = measurement.topic_name_;
  return output;
}

LocalizationFilter15::LocalizationPredictInput15 make_prediction_input(
  const ControlCommand & control,
  const std::vector<bool> & control_update_vector,
  const bool use_control,
  const TimestampNs reference_time,
  const DurationNs delta)
{
  if (control.control.size() != TWIST_SIZE ||
    control_update_vector.size() != static_cast<std::size_t>(TWIST_SIZE))
  {
    throw std::invalid_argument("control and control_config must each contain six values");
  }
  if (use_control && !control.control.allFinite())
  {
    throw std::invalid_argument("control values must be finite");
  }

  LocalizationFilter15::LocalizationPredictInput15 input;
  auto rpp_control = input.control();
  rpp_control.present() = use_control;
  rpp_control.stampNs() = control.stamp;
  auto values = rpp_control.values();
  auto enabled = rpp_control.enabled();
  values.resize(TWIST_SIZE);
  enabled.resize(TWIST_SIZE);
  for (Eigen::Index index = 0; index < TWIST_SIZE; ++index)
  {
    const auto control_index = static_cast<std::size_t>(index);
    values[control_index] = control.control(index);
    enabled[control_index] = control_update_vector[control_index];
  }
  input.referenceTimeNs() = reference_time;
  input.deltaNs() = delta;
  return input;
}

void apply_estimate(
  const LocalizationFilter15::Estimate15::Const & estimate,
  StateVector & state,
  CovarianceMatrix & covariance)
{
  const auto state_values = estimate.state().values();
  const auto covariance_values = estimate.covariance().values();
  if (state_values.size() != STATE_SIZE ||
    covariance_values.size() != static_cast<std::size_t>(STATE_SIZE * STATE_SIZE))
  {
    throw std::runtime_error("RPP filter returned an estimate with an invalid shape");
  }

  state.resize(STATE_SIZE);
  covariance.resize(STATE_SIZE, STATE_SIZE);
  for (Eigen::Index row = 0; row < STATE_SIZE; ++row)
  {
    state(row) = state_values[static_cast<std::size_t>(row)];
    for (Eigen::Index column = 0; column < STATE_SIZE; ++column)
    {
      covariance(row, column) =
        covariance_values[static_cast<std::size_t>(row * STATE_SIZE + column)];
    }
  }
  require_valid_state(state);
  require_valid_covariance(covariance);
}

}  // namespace

void RosFilterBase::initialize_rpp_filter()
{
  reset_rpp_filter();
  rpp_filter_.reset();
  rpp_script_.reset();
  rpp_context_.reset();
  rpp_use_control_ = declare_parameter("use_control", false);
  rpp_control_update_vector_ = declare_parameter(
    "control_config", std::vector<bool>(TWIST_SIZE, false));
  if (rpp_control_update_vector_.size() != static_cast<std::size_t>(TWIST_SIZE))
  {
    throw std::invalid_argument("control_config must contain six values");
  }

  ros_filter_utilities::load_covariance_parameter(
    *this, "initial_estimate_covariance", rpp_covariance_);
  require_valid_covariance(rpp_covariance_);

  const auto script_reference = declare_parameter<std::string>(
    "script", "rpp_localization::localization");
  const auto configuration = declare_parameter<std::string>(
    "configuration", default_configuration_);
  const auto script = parse_script_reference(script_reference);
  // The composition is read from the script's own package unless another
  // workspace that links the script supplies it.
  const auto workspace = declare_parameter<std::string>("rpp_workspace", "");

  rpp::RppDataManager data_manager(
    rpp::RPP_HOME,
    workspace.empty() ?
    ament_index_cpp::get_package_share_path(script.library).string() : workspace);
  rpp::ComponentContextBuilder context_builder(data_manager);
  const std::optional<std::string> selected_configuration = configuration.empty() ?
    std::nullopt : std::optional<std::string>(configuration);
  rpp_context_ = std::make_unique<rpp::ComponentContext>(
    context_builder.build_script_from_library(
      script.library, script.name, selected_configuration));
  rpp_script_ = std::make_unique<LocalizationScript>(*rpp_context_);
  rpp_script_->initialize();
  rpp_filter_ = rpp_script_->filter();
  if (!rpp_filter_)
  {
    throw std::runtime_error("RPP composition did not provide a LocalizationFilter15");
  }
}

void RosFilterBase::reset_rpp_filter()
{
  rpp_state_.setZero(STATE_SIZE);
  rpp_covariance_.setIdentity(STATE_SIZE, STATE_SIZE);
  rpp_covariance_ *= k_initial_covariance;
  rpp_control_.stamp = 0;
  rpp_control_.control.setZero(TWIST_SIZE);
  rpp_last_measurement_time_ = 0;
  rpp_filter_initialized_ = false;
  rpp_remote_reset_required_ = false;

  if (rpp_filter_)
  {
    const auto status = rpp_filter_->reset(
      std::move(make_estimate(rpp_state_, rpp_covariance_, rpp_last_measurement_time_)));
    require_ok(status, "RPP filter reset failed");
  }
}

void RosFilterBase::correct_rpp_filter(const Measurement & measurement)
{
  if (!rpp_filter_initialized_)
  {
    require_valid_state(measurement.measurement_);
    if (measurement.covariance_.rows() != STATE_SIZE ||
      measurement.covariance_.cols() != STATE_SIZE || !measurement.covariance_.allFinite() ||
      measurement.update_vector_.size() != static_cast<std::size_t>(STATE_SIZE))
    {
      throw std::invalid_argument("first measurement must have a finite 15-state payload");
    }

    for (Eigen::Index row = 0; row < STATE_SIZE; ++row)
    {
      if (!measurement.update_vector_[static_cast<std::size_t>(row)])
      {
        continue;
      }
      rpp_state_(row) = measurement.measurement_(row);
      for (Eigen::Index column = 0; column < STATE_SIZE; ++column)
      {
        if (!measurement.update_vector_[static_cast<std::size_t>(column)])
        {
          continue;
        }
        const double value = measurement.covariance_(row, column);
        if (std::abs(value) <= validation::kMinimumMeasurementCovariance)
        {
          continue;
        }
        rpp_covariance_(row, column) = row == column ? std::abs(value) : value;
      }
    }
    require_valid_state(rpp_state_);
    require_valid_covariance(rpp_covariance_);
    rpp_last_measurement_time_ = measurement.time_;
    const auto status = rpp_filter_->initialize(
      std::move(make_estimate(rpp_state_, rpp_covariance_, rpp_last_measurement_time_)));
    require_ok(status, "RPP filter initialization failed");
    rpp_filter_initialized_ = true;
    rpp_remote_reset_required_ = false;
    return;
  }

  if (rpp_remote_reset_required_)
  {
    const auto status = rpp_filter_->reset(
      std::move(make_estimate(rpp_state_, rpp_covariance_, rpp_last_measurement_time_)));
    require_ok(status, "RPP filter reset failed");
    rpp_remote_reset_required_ = false;
  }

  const auto result = rpp_filter_->correct(std::move(make_measurement(measurement)));
  require_ok(result.status(), "RPP filter correction failed");
  apply_estimate(result.estimate(), rpp_state_, rpp_covariance_);
}

void RosFilterBase::predict_rpp_filter(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  if (!rpp_filter_initialized_)
  {
    return;
  }
  if (delta < 0)
  {
    throw std::invalid_argument("prediction delta must be non-negative");
  }
  if (rpp_remote_reset_required_)
  {
    const auto status = rpp_filter_->reset(
      std::move(make_estimate(rpp_state_, rpp_covariance_, rpp_last_measurement_time_)));
    require_ok(status, "RPP filter reset failed");
    rpp_remote_reset_required_ = false;
  }

  const auto result = rpp_filter_->predict(std::move(make_prediction_input(
    rpp_control_, rpp_control_update_vector_, rpp_use_control_, reference_time, delta)));
  require_ok(result.status(), "RPP filter prediction failed");
  apply_estimate(result.estimate(), rpp_state_, rpp_covariance_);
}

void RosFilterBase::process_rpp_measurement(const Measurement & measurement)
{
  if (!rpp_filter_initialized_)
  {
    correct_rpp_filter(measurement);
    return;
  }

  const DurationNs delta = measurement.time_ - rpp_last_measurement_time_;
  if (delta > 0)
  {
    rclcpp::Duration ros_delta = ros::to_ros_duration(delta);
    validate_rpp_filter_delta(ros_delta);
    predict_rpp_filter(measurement.time_, ros_delta.nanoseconds());
  }
  correct_rpp_filter(measurement);
  if (delta >= 0)
  {
    rpp_last_measurement_time_ = measurement.time_;
  }
}

bool RosFilterBase::rpp_filter_debug() const noexcept
{
  return rpp_debug_;
}

bool RosFilterBase::rpp_filter_initialized() const noexcept
{
  return rpp_filter_initialized_;
}

bool RosFilterBase::rpp_filter_uses_control() const noexcept
{
  return rpp_use_control_;
}

const StateVector & RosFilterBase::rpp_filter_state() const noexcept
{
  return rpp_state_;
}

const CovarianceMatrix & RosFilterBase::rpp_filter_covariance() const noexcept
{
  return rpp_covariance_;
}

const ControlCommand & RosFilterBase::rpp_filter_control() const noexcept
{
  return rpp_control_;
}

const std::vector<bool> & RosFilterBase::rpp_filter_control_update_vector() const noexcept
{
  return rpp_control_update_vector_;
}

TimestampNs RosFilterBase::rpp_filter_last_measurement_time() const noexcept
{
  return rpp_last_measurement_time_;
}

const rclcpp::Duration & RosFilterBase::rpp_filter_sensor_timeout() const noexcept
{
  return rpp_sensor_timeout_;
}

void RosFilterBase::set_rpp_filter_control(const ControlCommand & control)
{
  if (control.control.size() != TWIST_SIZE || !control.control.allFinite())
  {
    throw std::invalid_argument("control must contain six finite values");
  }
  rpp_control_ = control;
}

void RosFilterBase::set_rpp_filter_debug(const bool debug, std::ostream * output)
{
  rpp_debug_ = debug && output != nullptr;
}

void RosFilterBase::set_rpp_filter_last_measurement_time(const TimestampNs time) noexcept
{
  rpp_last_measurement_time_ = time;
}

void RosFilterBase::set_rpp_filter_sensor_timeout(const rclcpp::Duration & timeout)
{
  rpp_sensor_timeout_ = timeout;
}

void RosFilterBase::set_rpp_filter_state(const Eigen::VectorXd & state)
{
  require_valid_state(state);
  rpp_state_ = state;
  rpp_remote_reset_required_ = rpp_filter_initialized_;
}

void RosFilterBase::set_rpp_filter_covariance(const Eigen::MatrixXd & covariance)
{
  require_valid_covariance(covariance);
  rpp_covariance_ = covariance;
  rpp_remote_reset_required_ = rpp_filter_initialized_;
}

void RosFilterBase::validate_rpp_filter_delta(rclcpp::Duration & delta) const
{
  if (delta.nanoseconds() < 0)
  {
    delta = rclcpp::Duration::from_nanoseconds(0);
  }
}

}  // namespace rpp_localization
