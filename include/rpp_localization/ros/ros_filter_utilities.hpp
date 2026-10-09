#ifndef RPP_LOCALIZATION__ROS_FILTER_UTILITIES_HPP_
#define RPP_LOCALIZATION__ROS_FILTER_UTILITIES_HPP_

#include <ostream>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <functional>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/buffer_core.hpp"

#include "rpp_localization/core/filter_common.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define RF_DEBUG(msg) \
  if (this->rpp_filter_debug()) { \
    this->_debug_stream << msg; \
  }

// Handy methods for debug output
std::ostream & operator<<(std::ostream & os, const tf2::Vector3 & vec);
std::ostream & operator<<(std::ostream & os, const tf2::Quaternion & quat);
std::ostream & operator<<(std::ostream & os, const tf2::Transform & trans);
std::ostream & operator<<(std::ostream & os, const std::vector<double> & vec);
std::ostream & operator<<(std::ostream & os, const std::vector<bool> & vec);

namespace rpp_localization
{


struct CallbackData
{
  CallbackData(
    const std::string & topic_name,
    const std::string & topic,
    const std::vector<bool> & update_vector, const int update_sum,
    const bool differential, const bool relative,
    const bool pose_use_child_frame,
    const double rejection_threshold)
  : topic_name_(topic_name), topic_(topic), update_vector_(update_vector),
    update_sum_(update_sum), differential_(differential),
    relative_(relative), pose_use_child_frame_(pose_use_child_frame),
    rejection_threshold_(rejection_threshold) {}

  std::string topic_name_;
  std::string topic_;
  std::vector<bool> update_vector_;
  int update_sum_;
  bool differential_;
  bool relative_;
  bool pose_use_child_frame_;
  double rejection_threshold_;
};

namespace ros_filter_utilities
{

double get_yaw(const tf2::Quaternion quat);

//! @brief Utility method for converting quaternion to RPY
//! @param[in] quat - The quaternion to convert
//! @param[out] roll - The converted roll
//! @param[out] pitch - The converted pitch
//! @param[out] yaw - The converted yaw
//!
void quat_to_rpy(
  const tf2::Quaternion & quat, double & roll, double & pitch,
  double & yaw);

//! @brief Converts our Eigen state vector into a TF transform/pose
//! @param[in] state - The state to convert
//! @param[out] stateTF - The converted state
//!
void state_to_tf(const Eigen::VectorXd & state, tf2::Transform & stateTF);

//! @brief Converts a TF transform/pose into our Eigen state vector
//! @param[in] stateTF - The state to convert
//! @param[out] state - The converted state
//!
void tf_to_state(const tf2::Transform & stateTF, Eigen::VectorXd & state);

void load_covariance_parameter(rclcpp::Node& node, const std::string & parameter, Eigen::MatrixXd & covariance);


  //! @brief Loads fusion settings from the config file
  //! @param[in] topic_name - The name of the topic for which to load settings
  //! @return The boolean vector of update settings for each variable for this
  //! topic
  //!
std::vector<bool> load_update_config(rclcpp::Node& node, const std::string & topic_name);

void handle_odom_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& pose_callback_data_v,
  std::vector<CallbackData>& twist_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&)>& on_registered_topic = nullptr
);

void handle_pose_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& pose_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&)>& on_registered_topic = nullptr
);

void handle_twist_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& twist_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&)>& on_registered_topic = nullptr
);

/// control_drives_acceleration says whether a measured acceleration replaces
/// the control on its axis; control_update_vector is updated accordingly.
void handle_imu_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<bool>& control_update_vector,
  bool control_drives_acceleration,
  std::map<std::string, bool>& remove_gravitational_acceleration,
  std::vector<CallbackData>& pose_callback_data_v,
  std::vector<CallbackData>& twist_callback_data_v,
  std::vector<CallbackData>& acc_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&, const CallbackData&)>& on_registered_topic = nullptr
);


void warn_if_misconfigured(
  rclcpp::Node& node,
  bool two_d_mode,
  std::vector<std::string>& state_variable_names,
  std::map<StateMembers, int>& abs_pose_var_counts,
  std::map<StateMembers, int>& twist_var_counts
);

}  // namespace ros_filter_utilities
}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__ROS_FILTER_UTILITIES_HPP_
