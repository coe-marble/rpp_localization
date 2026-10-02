#include "rpp_localization/ros/ros_filter_utilities.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/buffer_core.hpp"


using namespace rpp_localization;

#define THROTTLE(clock, duration, thing) do { \
    static rclcpp::Time _last_output_time ## __LINE__(0, 0, (clock)->get_clock_type()); \
    auto _now = (clock)->now(); \
    if (_now - _last_output_time ## __LINE__ > (duration)) { \
      _last_output_time ## __LINE__ = _now; \
      thing; \
    } \
} while (0)

std::ostream & operator<<(std::ostream & os, const tf2::Vector3 & vec)
{
  os << "(" << std::setprecision(20) << vec.getX() << " " << vec.getY() << " " <<
    vec.getZ() << ")\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const tf2::Quaternion & quat)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3 or_tmp(quat);
  or_tmp.getRPY(roll, pitch, yaw);

  os << "(" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw <<
    ")\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const tf2::Transform & trans)
{
  os << "Origin: " << trans.getOrigin() <<
    "Rotation (RPY): " << trans.getRotation();

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<double> & vec)
{
  os << "(" << std::setprecision(20);

  for (size_t i = 0; i < vec.size(); ++i) {
    os << vec[i] << " ";
  }

  os << ")\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<bool> & vec)
{
  os << "(" << std::boolalpha;

  for (size_t i = 0; i < vec.size(); ++i) {
    os << vec[i] << " ";
  }

  os << ")\n";

  return os;
}

namespace rpp_localization
{
namespace ros_filter_utilities
{

double get_yaw(const tf2::Quaternion quat)
{
  tf2::Matrix3x3 mat(quat);

  double dummy;
  double yaw;
  mat.getRPY(dummy, dummy, yaw);

  return yaw;
}


void quat_to_rpy(
  const tf2::Quaternion & quat, double & roll, double & pitch,
  double & yaw)
{
  tf2::Matrix3x3 or_tmp(quat);
  or_tmp.getRPY(roll, pitch, yaw);
}

void state_to_tf(const Eigen::VectorXd & state, tf2::Transform & state_tf)
{
  state_tf.setOrigin(
    tf2::Vector3(
      state(StateMemberX), state(StateMemberY),
      state(StateMemberZ)));
  tf2::Quaternion quat;
  quat.setRPY(
    state(StateMemberRoll), state(StateMemberPitch),
    state(StateMemberYaw));

  state_tf.setRotation(quat);
}

void tf_to_state(const tf2::Transform & state_tf, Eigen::VectorXd & state)
{
  state(StateMemberX) = state_tf.getOrigin().getX();
  state(StateMemberY) = state_tf.getOrigin().getY();
  state(StateMemberZ) = state_tf.getOrigin().getZ();
  quat_to_rpy(
    state_tf.getRotation(), state(StateMemberRoll),
    state(StateMemberPitch), state(StateMemberYaw));
}



void load_covariance_parameter(rclcpp::Node& node, const std::string & parameter, Eigen::MatrixXd & covariance)
{
  covariance.setZero();
  std::vector<double> covar_flat;

  if (!node.has_parameter(parameter))
  {
    node.declare_parameter(parameter, rclcpp::PARAMETER_DOUBLE_ARRAY);
  }
  if (node.get_parameter(parameter, covar_flat)) {
    if (covar_flat.size() == STATE_SIZE) {
      RCLCPP_INFO_STREAM(
        node.get_logger(), "Detected a " << parameter << " parameter with "
          "length " << STATE_SIZE << ". Assuming diagonal values specified.");
      covariance.diagonal() = Eigen::VectorXd::Map(covar_flat.data(), STATE_SIZE);
    } else if (covariance.size() == STATE_SIZE * STATE_SIZE) {
      covariance = Eigen::MatrixXd::Map(covar_flat.data(), STATE_SIZE, STATE_SIZE);
    } else {
      std::string error = "Invalid " + parameter + " specified. Expected a length of " +
        std::to_string(STATE_SIZE) + " or " + std::to_string(STATE_SIZE * STATE_SIZE) +
        ", received length " + std::to_string(covar_flat.size());
      RCLCPP_FATAL_STREAM(node.get_logger(), error);
      throw std::invalid_argument(error);
    }
  }
}


void handle_odom_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& pose_callback_data_v,
  std::vector<CallbackData>& twist_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&)>& on_registered_topic
)
{
  size_t topic_ind = 0;
  bool more_params = false;
  do {
    // Build the string in the form of "odomX", where X is the odom topic
    // number, then check if we have any parameters with that value. Users need
    // to make sure they don't have gaps in their configs (e.g., odom0 and then
    // odom2)
    std::stringstream ss;
    ss << "odom" << topic_ind++;
    std::string odom_topic_name = ss.str();
    std::string odom_topic;
    node.declare_parameter(odom_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node.get_parameter(odom_topic_name, parameter)) {
      more_params = true;
      odom_topic = node.get_node_base_interface()->
          resolve_topic_or_service_name(parameter.as_string(), false);

    } else {
      more_params = false;
    }

    if (more_params) {
      // Determine if we want to integrate this sensor differentially
      bool differential = node.declare_parameter(
        odom_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = node.declare_parameter(odom_topic_name + std::string("_relative"), false);

      if (relative && differential) {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Both " << odom_topic_name << "_differential" << " and " << odom_topic_name <<
            "_relative were set to true. Using differential mode.");

        relative = false;
      }

      // Consider odometry transformation from the child_frame_id instead of the base_link_frame_id
      bool pose_use_child_frame = node.declare_parameter(
        odom_topic_name + std::string("_pose_use_child_frame"), false);

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = node.declare_parameter(
        odom_topic_name +
        std::string("_pose_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Check for twist rejection threshold
      double twist_mahalanobis_thresh = node.declare_parameter(
        odom_topic_name +
        std::string("_twist_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = node.declare_parameter(
        odom_topic_name +
        std::string("_queue_size"), 10);

      // Now pull in its boolean update vector configuration. Create separate
      // vectors for pose and twist data, and then zero out the opposite values
      // in each vector (no pose data in the twist update vector and
      // vice-versa).
      std::vector<bool> update_vec = load_update_config(node, odom_topic_name);
      std::vector<bool> pose_update_vec = update_vec;
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::vector<bool> twist_update_vec = update_vec;
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);

      CallbackData pose_callback_data(
        odom_topic_name + "_pose", odom_topic, pose_update_vec, pose_update_sum,
        differential, relative, pose_use_child_frame, pose_mahalanobis_thresh);
      pose_callback_data_v.push_back(pose_callback_data);

      const CallbackData twist_callback_data(
        odom_topic_name + "_twist", odom_topic, twist_update_vec, twist_update_sum, false,
        false, false, twist_mahalanobis_thresh);
      twist_callback_data_v.push_back(twist_callback_data);

      // Store the odometry topic subscribers so they don't go out of scope.
      if (pose_update_sum + twist_update_sum > 0) {

        if (on_registered_topic != nullptr)
        {
          on_registered_topic(odom_topic, odom_topic_name, queue_size, pose_callback_data, twist_callback_data);
        }
      } else {
        std::stringstream stream;
        stream << odom_topic << " is listed as an input topic, but all update "
          "variables are false";

        RCLCPP_ERROR(node.get_logger(), stream.str().c_str());

        // node->add_diagnostic(
        //   diagnostic_msgs::msg::DiagnosticStatus::WARN,
        //   odom_topic + "_configuration", stream.str(), true);
      }

      if (debug_stream != nullptr)
      {
        *debug_stream << "Registerd topic " <<
            odom_topic << " (" << odom_topic_name << ")\n\t" <<
            odom_topic_name << "_differential is " <<
            (differential ? "true" : "false") << "\n\t" << odom_topic_name <<
            "_pose_rejection_threshold is " << pose_mahalanobis_thresh <<
            "\n\t" << odom_topic_name << "_twist_rejection_threshold is " <<
            twist_mahalanobis_thresh << "\n\t" << odom_topic_name <<
            " pose update vector is " << pose_update_vec << "\t" <<
            odom_topic_name << " twist update vector is " <<
            twist_update_vec;
      }
    }
  } while (more_params);
}


void handle_pose_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& pose_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&)>& on_registered_topic
)
{
  size_t topic_ind = 0;
  bool more_params = false;
  do {
    std::stringstream ss;
    ss << "pose" << topic_ind++;
    std::string pose_topic_name = ss.str();
    std::string pose_topic;
    node.declare_parameter(pose_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node.get_parameter(pose_topic_name, parameter)) {
      more_params = true;
      pose_topic = node.get_node_base_interface()->
          resolve_topic_or_service_name(parameter.as_string(), false);
    } else {
      more_params = false;
    }

    if (more_params) {
      bool differential = node.declare_parameter(
        pose_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = node.declare_parameter(
        pose_topic_name + std::string("_relative"),
        false);

      if (relative && differential) {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Both " << pose_topic_name << "_differential" << " and " << pose_topic_name <<
            "_relative were set to true. Using differential mode.");

        relative = false;
      }

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = node.declare_parameter(
        pose_topic_name +
        std::string("_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = node.declare_parameter(
        pose_topic_name +
        std::string("_queue_size"), 10);

      // Pull in the sensor's config, zero out values that are invalid for the
      // pose type
      std::vector<bool> pose_update_vec = load_update_config(node, pose_topic_name);
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_A_OFFSET,
        pose_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
        0);


      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);

      if (pose_update_sum > 0)
      {
        const CallbackData callback_data(pose_topic_name, pose_topic, pose_update_vec,
          pose_update_sum, differential,
          relative, false, pose_mahalanobis_thresh);

        pose_callback_data_v.push_back(callback_data);

        if (on_registered_topic != nullptr)
        {
          on_registered_topic(pose_topic, pose_topic_name, queue_size, callback_data);
        }

      }
      else
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Warning: " << pose_topic << " is listed as an input topic, but all pose update "
            "variables are false");
      }

      if (debug_stream != nullptr)
      {
        *debug_stream << "Registered topic " <<
          pose_topic << " (" << pose_topic_name << ")\n\t" <<
          pose_topic_name << "_differential is " <<
          (differential ? "true" : "false") << "\n\t" << pose_topic_name <<
          "_rejection_threshold is " << pose_mahalanobis_thresh <<
          "\n\t" << pose_topic_name << " update vector is " <<
          pose_update_vec;
      }
    }
  } while (more_params);
}


void handle_twist_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<CallbackData>& twist_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&)>& on_registered_topic
)
{
  size_t topic_ind = 0;
  bool more_params = false;
  do {
    std::stringstream ss;
    ss << "twist" << topic_ind++;
    std::string twist_topic_name = ss.str();
    std::string twist_topic;
    node.declare_parameter(twist_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node.get_parameter(twist_topic_name, parameter)) {
      more_params = true;
      twist_topic = node.get_node_base_interface()->
          resolve_topic_or_service_name(parameter.as_string(), false);
    } else {
      more_params = false;
    }

    if (more_params) {
      // Check for twist rejection threshold
      double twist_mahalanobis_thresh = node.declare_parameter(
        twist_topic_name +
        std::string("_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = node.declare_parameter(
        twist_topic_name +
        std::string("_queue_size"), 10);

      // Pull in the sensor's config, zero out values that are invalid for the
      // twist type
      std::vector<bool> twist_update_vec = load_update_config(node, twist_topic_name);
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);

      if (twist_update_sum > 0) {
        const CallbackData callback_data(twist_topic_name, twist_topic, twist_update_vec,
          twist_update_sum, false, false, false,
          twist_mahalanobis_thresh);

        twist_callback_data_v.push_back(callback_data);
        if (on_registered_topic != nullptr)
        {
          on_registered_topic(twist_topic, twist_topic_name, queue_size, callback_data);
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Warning: " << twist_topic << " is listed as an input topic, but all twist update "
            "variables are false");
      }

      if (debug_stream != nullptr)
      {
        *debug_stream <<
          "Registered topic " << twist_topic << " (" << twist_topic_name << ")\n\t" <<
            twist_topic_name << "_rejection_threshold is " << twist_mahalanobis_thresh <<
            "\n\t" << twist_topic_name << " update vector is " << twist_update_vec;
      }
    }
  } while (more_params);
}



void handle_imu_params(rclcpp::Node& node, std::ofstream* debug_stream,
  std::vector<bool>& control_update_vector,
  std::map<std::string, bool>& remove_gravitational_acceleration,
  std::vector<CallbackData>& pose_callback_data_v,
  std::vector<CallbackData>& twist_callback_data_v,
  std::vector<CallbackData>& acc_callback_data_v,
  const std::function<void(const std::string&, const std::string&, int, const CallbackData&, const CallbackData&, const CallbackData&)>& on_registered_topic
)
{
  // Repeat for IMU
  size_t topic_ind = 0;
  bool more_params = false;
  do {
    std::stringstream ss;
    ss << "imu" << topic_ind++;
    std::string imu_topic_name = ss.str();
    std::string imu_topic;
    node.declare_parameter(imu_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (node.get_parameter(imu_topic_name, parameter)) {
      more_params = true;
      imu_topic = node.get_node_base_interface()->
          resolve_topic_or_service_name(parameter.as_string(), false);
    } else {
      more_params = false;
    }

    if (more_params) {
      bool differential = node.declare_parameter(
        imu_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = node.declare_parameter(imu_topic_name + std::string("_relative"), false);

      if (relative && differential) {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Both " << imu_topic_name << "_differential" << " and " << imu_topic_name << "_relative "
            "were set to true. Using differential mode.");

        relative = false;
      }

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = node.declare_parameter(
        imu_topic_name +
        std::string("_pose_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Check for angular velocity rejection threshold
      std::string imu_twist_rejection_name =
        imu_topic_name + std::string("_twist_rejection_threshold");
      double twist_mahalanobis_thresh = node.declare_parameter(
        imu_twist_rejection_name,
        std::numeric_limits<double>::max());

      // Check for acceleration rejection threshold
      double accel_mahalanobis_thresh = node.declare_parameter(
        imu_topic_name +
        std::string("_linear_acceleration_rejection_threshold"),
        std::numeric_limits<double>::max());

      bool remove_grav_acc = node.declare_parameter(
        imu_topic_name +
        "_remove_gravitational_acceleration",
        false);
      remove_gravitational_acceleration[imu_topic_name + "_acceleration"] =
        remove_grav_acc;

      // Set optional custom queue size
      int queue_size = node.declare_parameter(
        imu_topic_name +
        std::string("_queue_size"), 10);

      // Now pull in its boolean update vector configuration and differential
      // update configuration (as this contains pose information)
      std::vector<bool> update_vec = load_update_config(node, imu_topic_name);

      // sanity checks for update config settings
      std::vector<int> position_update_vec(update_vec.begin() + POSITION_OFFSET,
        update_vec.begin() + POSITION_OFFSET + POSITION_SIZE);
      int position_update_sum = std::accumulate(
        position_update_vec.begin(),
        position_update_vec.end(), 0);
      if (position_update_sum > 0)
      {
        RCLCPP_WARN_STREAM(
          node.get_logger(),
          "Warning: Some position entries in parameter " << imu_topic_name << "_config are set to "
            "true, but the sensor_msgs/Imu message contains no positional data");
      }
      std::vector<int> linear_velocity_update_vec(
        update_vec.begin() + POSITION_V_OFFSET,
        update_vec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE);
      int linear_velocity_update_sum = std::accumulate(
        linear_velocity_update_vec.begin(), linear_velocity_update_vec.end(),
        0);
      if (linear_velocity_update_sum > 0) {
        RCLCPP_WARN_STREAM(
          node.get_logger(),
          "Warning: Some position entries in parameter " << imu_topic_name << "_config are set to "
            "true, but the sensor_msgs/Imu message contains no linear velocity data");
      }

      std::vector<bool> pose_update_vec = update_vec;
      // IMU message contains no information about position, filter everything
      // except orientation
      std::fill(
        pose_update_vec.begin() + POSITION_OFFSET,
        pose_update_vec.begin() + POSITION_OFFSET + POSITION_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_A_OFFSET,
        pose_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

      std::vector<bool> twist_update_vec = update_vec;
      // IMU message contains no information about linear speeds, filter
      // everything except angular velocity
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
      std::fill(
        twist_update_vec.begin() + POSITION_V_OFFSET,
        twist_update_vec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE, 0);
      std::fill(
        twist_update_vec.begin() + POSITION_A_OFFSET,
        twist_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

      std::vector<bool> accel_update_vec = update_vec;
      std::fill(
        accel_update_vec.begin() + POSITION_OFFSET,
        accel_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
      std::fill(
        accel_update_vec.begin() + POSITION_V_OFFSET,
        accel_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);

      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);
      int accelUpdateSum =
        std::accumulate(accel_update_vec.begin(), accel_update_vec.end(), 0);



      // Check if we're using control input for any of the acceleration
      // variables; turn off if so
      if (control_update_vector[ControlMemberVx] &&
        static_cast<bool>(accel_update_vec[StateMemberAx]))
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "X acceleration is being measured from IMU; X velocity control input is disabled");
        control_update_vector[ControlMemberVx] = 0;
      }
      if (control_update_vector[ControlMemberVy] &&
        static_cast<bool>(accel_update_vec[StateMemberAy]))
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Y acceleration is being measured from IMU; Y velocity control input is disabled");
        control_update_vector[ControlMemberVy] = 0;
      }
      if (control_update_vector[ControlMemberVz] &&
        static_cast<bool>(accel_update_vec[StateMemberAz]))
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Z acceleration is being measured from IMU; Z velocity control input is disabled");
        control_update_vector[ControlMemberVz] = 0;
      }

      if (pose_update_sum + twist_update_sum + accelUpdateSum > 0) {
        const CallbackData pose_callback_data(
          imu_topic_name + "_pose", imu_topic, pose_update_vec, pose_update_sum,
          differential, relative, false, pose_mahalanobis_thresh);
        const CallbackData twist_callback_data(
          imu_topic_name + "_twist", imu_topic, twist_update_vec, twist_update_sum,
          differential, relative, false, twist_mahalanobis_thresh);
        const CallbackData accel_callback_data(
          imu_topic_name + "_acceleration", imu_topic, accel_update_vec, accelUpdateSum,
          differential, relative, false, accel_mahalanobis_thresh);

        pose_callback_data_v.push_back(pose_callback_data);
        twist_callback_data_v.push_back(twist_callback_data);
        acc_callback_data_v.push_back(accel_callback_data);
        if (on_registered_topic != nullptr)
        {
          on_registered_topic(imu_topic, imu_topic_name, queue_size, pose_callback_data, twist_callback_data, accel_callback_data);
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          node.get_logger(),
          "Warning: " << imu_topic << " is listed as an input topic, but all its update variables "
            "are false");
      }

      if (debug_stream != nullptr)
      {
        *debug_stream <<
        "Registered topic " <<
          imu_topic << " (" << imu_topic_name << ")\n\t" <<
          imu_topic_name << "_differential is " <<
          (differential ? "true" : "false") << "\n\t" << imu_topic_name <<
          "_pose_rejection_threshold is " << pose_mahalanobis_thresh <<
          "\n\t" << imu_topic_name << "_twist_rejection_threshold is " <<
          twist_mahalanobis_thresh << "\n\t" << imu_topic_name <<
          "_linear_acceleration_rejection_threshold is " <<
          accel_mahalanobis_thresh << "\n\t" << imu_topic_name <<
          "_remove_gravitational_acceleration is " <<
          (remove_grav_acc ? "true" : "false") << "\n\t" <<
          imu_topic_name << " pose update vector is " << pose_update_vec <<
          "\t" << imu_topic_name << " twist update vector is " <<
          twist_update_vec << "\t" << imu_topic_name <<
          " acceleration update vector is " << accel_update_vec;
      }
    }
  } while (more_params);

}

std::vector<bool> load_update_config(rclcpp::Node& node, const std::string & topic_name)
{
  std::vector<bool> update_vector(STATE_SIZE, 0);
  const std::string topic_config_name = topic_name + "_config";

  update_vector = node.declare_parameter(topic_config_name, update_vector);

  return update_vector;
}


  /* Warn users about:
   *    1. Multiple non-differential input sources
   *    2. No absolute *or* velocity measurements for pose variables
   */
void warn_if_misconfigured(
  rclcpp::Node& node,
  bool two_d_mode,
  std::vector<std::string>& state_variable_names,
  std::map<StateMembers, int>& abs_pose_var_counts,
  std::map<StateMembers, int>& twist_var_counts
)
{
  for (int state_var = StateMemberX; state_var <= StateMemberYaw; ++state_var) {
    if (abs_pose_var_counts[static_cast<StateMembers>(state_var)] > 1) {
      std::stringstream stream;
      stream << abs_pose_var_counts[static_cast<StateMembers>(state_var -
        POSITION_OFFSET)] << " absolute pose inputs detected for " <<
        state_variable_names[state_var] <<
        ". This may result in oscillations. Please ensure that your"
        "variances for each measured variable are set appropriately.";

      // this->add_diagnostic(
      //   diagnostic_msgs::msg::DiagnosticStatus::WARN,
      //   state_variable_names[state_var] + "_configuration",
      //   stream.str(), true);
      RCLCPP_WARN(node.get_logger(), stream.str().c_str());
    } else if (abs_pose_var_counts[static_cast<StateMembers>(state_var)] == 0) {
      if ((static_cast<StateMembers>(state_var) == StateMemberX &&
        twist_var_counts[static_cast<StateMembers>(StateMemberVx)] == 0) ||
        (static_cast<StateMembers>(state_var) == StateMemberY &&
        twist_var_counts[static_cast<StateMembers>(StateMemberVy)] == 0) ||
        (static_cast<StateMembers>(state_var) == StateMemberZ &&
        twist_var_counts[static_cast<StateMembers>(StateMemberVz)] == 0 &&
        two_d_mode == false) ||
        (static_cast<StateMembers>(state_var) == StateMemberRoll &&
        twist_var_counts[static_cast<StateMembers>(StateMemberVroll)] == 0 &&
        two_d_mode == false) || (static_cast<StateMembers>(state_var) ==
        StateMemberPitch &&
        twist_var_counts[static_cast<StateMembers>(StateMemberVpitch)] == 0 &&
        two_d_mode == false) || (static_cast<StateMembers>(state_var) ==
        StateMemberYaw && twist_var_counts[static_cast<StateMembers>(StateMemberVyaw)] ==
        0))
      {
        std::stringstream stream;
        stream << "Neither " << state_variable_names[state_var] << " nor its "
          "velocity is being measured. This will result in unbounded"
          "error growth and erratic filter behavior.";

        // this->add_diagnostic(
        //   diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        //   this->state_variable_names[state_var] + "_configuration",
        //   stream.str(), true);

        RCLCPP_WARN(node.get_logger(), stream.str().c_str());
      }
    }
  }
}




}  // namespace ros_filter_utilities
}  // namespace rpp_localization
