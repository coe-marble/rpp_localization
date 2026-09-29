
#include "rpp_localization/ros/rosbag_filter.hpp"

// #include "rpp_localization/models/python_model.hpp"

using namespace rpp_localization;

namespace rpp_localization
{

template <typename T>
RosBagFilter<T>::RosBagFilter(const rclcpp::NodeOptions& options, RosBagMsgProvider& provider, double frequency)
  : RosFilterBase<T>(options, false),
    _filter_param_namespace("rosbag_filter"),
    _static_transform_param_namespace(_filter_param_namespace + ".transforms"),
    _provider(provider),
    _frequency(frequency),
    _total_time(0, 0, rcl_clock_type_e::RCL_ROS_TIME ),
    _update_time(1. / _frequency)
{
  using namespace std::placeholders;

  _provider.set_on_message_callback(std::bind(&RosBagFilter::on_new_msg, this, _1, _2, _3));
}

template <typename T>
RosBagFilter<T>::~RosBagFilter() {}

template <typename T>
void RosBagFilter<T>::init()
{
  RosFilterBase<T>::init();
  load_params();
}

template <typename T>
void RosBagFilter<T>::load_params()
{
  this->load_filter_params();
  auto shared_this = RosFilterBase<T>::shared_from_this();

  this->load_static_transforms();

  std::vector<CallbackData> pose_callback_data_v;
  std::vector<CallbackData> twist_callback_data_v;
  std::vector<CallbackData> acc_callback_data_v;

  ros_filter_utilities::handle_odom_params(*shared_this,
    nullptr,
    pose_callback_data_v,
    twist_callback_data_v
  );

  ros_filter_utilities::handle_pose_params(*shared_this,
    nullptr,
    pose_callback_data_v
  );

  ros_filter_utilities::handle_twist_params(*shared_this,
    nullptr,
    twist_callback_data_v
  );

  auto control_update_vector = this->filter_.get_control_update_vector();
  ros_filter_utilities::handle_imu_params(*shared_this,
    nullptr, control_update_vector,
    this->remove_gravitational_acceleration_,
    pose_callback_data_v,
    twist_callback_data_v,
    acc_callback_data_v
  );

  for (int i = 0; i < pose_callback_data_v.size(); ++i)
  {
    CallbackData& data = pose_callback_data_v[i];
    pose_callback_data.insert(std::pair<std::string, CallbackData>(data.topic_, std::move(data)));
  }
  for (int i = 0; i < twist_callback_data_v.size(); ++i)
  {
    CallbackData& data = twist_callback_data_v[i];
    twist_callback_data.insert(std::pair<std::string, CallbackData>(data.topic_, std::move(data)));
  }
  for (int i = 0; i < acc_callback_data_v.size(); ++i)
  {
    CallbackData& data = acc_callback_data_v[i];
    acc_callback_data.insert(std::pair<std::string, CallbackData>(data.topic_, std::move(data)));
  }

}

template <typename T>
void RosBagFilter<T>::load_static_transforms()
{
  std::string tf_names(".tf");

  int idx = 0;
  std::string frame_id, child_frame_id;
  while (true)
  {
    std::stringstream ss;
    ss << tf_names << idx++;
    std::string tf_param_name = _static_transform_param_namespace + ss.str();
    frame_id = this->declare_parameter(tf_param_name + ".frame_id", "");
    child_frame_id = this->declare_parameter(tf_param_name + ".child_frame_id", "");

    if (child_frame_id == "")
      break;

    this->declare_parameter(tf_param_name + ".params", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> tf_params_flat;
    if (this->get_parameter(tf_param_name + ".params", tf_params_flat))
    {
      geometry_msgs::msg::TransformStamped transform;
      tf2::Quaternion quat;

      transform.child_frame_id = child_frame_id;
      transform.header.frame_id = frame_id;
      transform.transform.translation.x = tf_params_flat[0];
      transform.transform.translation.y = tf_params_flat[1];
      transform.transform.translation.z = tf_params_flat[2];
      quat.setRPY(tf_params_flat[5], tf_params_flat[4], tf_params_flat[3]);

      transform.transform.rotation.x = quat.x();
      transform.transform.rotation.y = quat.y();
      transform.transform.rotation.z = quat.z();
      transform.transform.rotation.w = quat.w();
      this->_tf_buffer->set_transform(transform, "rosbag_filter", true);
    }
  }

}

template <typename T>
FilterResult RosBagFilter<T>::filter()
{
  FilterResult result;
  if (!_provider.is_initialized())
  {
    RCLCPP_ERROR(this->get_logger(), "Ros bag filter not correctly initialized");
    return result;
  }

  double start_time = _provider.get_start_time();
  double duration = _provider.get_duration();
  _total_time = rclcpp::Time(start_time, _total_time.get_clock_type());

  RCLCPP_INFO(this->get_logger(), "Starting filter. Total messages: %ld", _provider.get_num_msgs());
  int i = 0, debug_every = 1000;
  while (_provider.has_next())
  {
    _total_time += rclcpp::Duration::from_seconds(_update_time);
    _provider.get_next(_total_time);
    update();
    result.states.push_back(this->filter_.get_state());
    result.covariances.push_back(this->filter_.get_estimate_error_covariance());
    result.timestamps.push_back(_total_time.seconds());
    if (++i % debug_every == 0)
    {
      double percent = (double)(_total_time.nanoseconds() - start_time) / duration * 100;
      RCLCPP_INFO(this->get_logger(), "Processed %.2f%%", percent);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Finished filtering");
  return result;
}

template <typename T>
bool RosBagFilter<T>::on_new_msg(const SerializedMessage* msg, const TopicMetadata& topic, Serializator& serializator)
{
  // if not startswith /, add it
  std::string topic_name = topic.name;
  if (topic_name[0] != '/')
    topic_name = "/" + topic_name;
  if (topic.type == "sensor_msgs/msg/Imu")
  {
    auto imu = serializator.deserialize_imu(msg);
    if (pose_callback_data.count(topic_name) == 0
      || twist_callback_data.count(topic_name) == 0
      || acc_callback_data.count(topic_name) == 0)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Callback data not set for imu topic '%s'. Skipping messages...", topic_name.c_str());
      return false;
    }
    auto imu_ptr = std::shared_ptr<sensor_msgs::msg::Imu>(&imu, [](auto*) { }); // no ownership
    auto pose_data = pose_callback_data.find(topic_name)->second;
    auto twist_data = twist_callback_data.find(topic_name)->second;
    auto acc_data = acc_callback_data.find(topic_name)->second;
    this->imuCallback(imu_ptr, topic.name, pose_data, twist_data, acc_data);
  }
  else if (topic.type == "geometry_msgs/msg/PoseWithCovarianceStamped")
  {
    auto pose = serializator.deserialize_pose_with_covariance_stamped(msg);
    if (pose_callback_data.count(topic_name) == 0)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Callback data not set for pose topic '%s'. Skipping messages...", topic_name.c_str());
      return false;
    }

    auto pose_data = pose_callback_data.find(topic_name)->second;
    auto pose_ptr = std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>(&pose, [](auto*) { });
    this->poseCallback(pose_ptr, pose_data, this->world_frame_id_, this->base_link_frame_id_, false);
  }
  else if (topic.type == "geometry_msgs/msg/TwistWithCovarianceStamped")
  {
    auto twist = serializator.deserialize_twist_with_covariance_stamped(msg);
    if (twist_callback_data.count(topic_name) == 0)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Callback data not set for twist topic '%s'. Skipping messages...", topic_name.c_str());
      return false;
    }

    auto twist_data = twist_callback_data.find(topic_name)->second;
    auto twist_ptr = std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>(&twist, [](auto*) { });
    this->twistCallback(twist_ptr, twist_data, this->base_link_frame_id_);
  }
  else if (topic.type == "nav_msgs/msg/Odometry")
  {
    auto odom = serializator.deserialize_odometry(msg);
    if (pose_callback_data.count(topic_name) == 0
      || twist_callback_data.count(topic_name) == 0)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Callback data not set for odometry topic '%s'. Skipping messages...", topic_name.c_str());
      return false;
    }

    auto pose_data = pose_callback_data.find(topic_name)->second;
    auto twist_data = twist_callback_data.find(topic_name)->second;
    auto odom_ptr = std::shared_ptr<nav_msgs::msg::Odometry>(&odom, [](auto*) { });
    this->odometryCallback(odom_ptr, topic.name, pose_data, twist_data);
  }
  return true;

}

template <typename T>
void RosBagFilter<T>::update()
{

  if (this->toggled_on_) {
    // Now we'll integrate any measurements we've received if requested,
    // and update angular acceleration.
    this->integrateMeasurements(_total_time);
    this->differentiateMeasurements(_total_time);
  } else {
    // Clear out measurements since we're not currently processing new entries
    this->clearMeasurementQueue();

    // Reset last measurement time so we don't get a large time delta on toggle
    if (this->filter_.get_initialized_status()) {
      this->filter_.set_last_measurement_time(ros::toTimestampNs(this->now()));
    }
  }

  // Get latest state and publish it
  auto filtered_position = std::make_unique<nav_msgs::msg::Odometry>();

  bool corrected_data = false;

  if (this->getFilteredOdometryMessage(filtered_position.get())) {
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
    if (!this->validateFilterOutput(filtered_position.get())) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Critical Error, NaNs were detected in the output state of the filter. "
        "This was likely due to poorly coniditioned process, noise, or sensor "
        "covariances.");
    }

  }

  // Clear out expired history data
  if (this->smooth_lagged_data_) {
    this->clearExpiredHistory(
      this->filter_.get_last_measurement_time() -
      ros::toDurationNs(this->history_length_));
  }
}
} // rpp_localization

template class rpp_localization::RosBagFilter<rpp_localization::Ekf<rpp_localization::ConstantAccelerationModel>>;
template class rpp_localization::RosBagFilter<rpp_localization::Ukf<rpp_localization::ConstantAccelerationModel>>;
template class rpp_localization::RosBagFilter<rpp_localization::InEkf<rpp_localization::InEKF::InertialProcess>>;
