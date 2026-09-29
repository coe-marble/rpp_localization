#pragma once

#include <string>
#include <functional>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "rosbag2_storage/topic_metadata.hpp"
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>


#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rpp_localization
{

#define MAKE_FUNCTION_NAME(name, prefix) name ## prefix


#define ENABLE_ROS_MSG(A, name) \
\
A MAKE_FUNCTION_NAME(deserialize_, name)(const rclcpp::SerializedMessage* ser_msg)\
{ \
    if (!_serializators.count(#A)) \
    { \
      _serializators.insert(std::make_pair(#A, new rclcpp::Serialization<A>())); \
    } \
    rclcpp::Serialization<A>* serializer = dynamic_cast<rclcpp::Serialization<A>*>(_serializators.at(#A)); \
    A msg; \
    serializer->deserialize_message(ser_msg, & msg); \
    return msg; \
} \

typedef rosbag2_storage::TopicMetadata TopicMetadata;
typedef rclcpp::SerializedMessage SerializedMessage;

class Serializator
{
  public:
    ENABLE_ROS_MSG(sensor_msgs::msg::Imu, imu)
    ENABLE_ROS_MSG(geometry_msgs::msg::PoseWithCovarianceStamped, pose_with_covariance_stamped)
    ENABLE_ROS_MSG(geometry_msgs::msg::TwistWithCovarianceStamped, twist_with_covariance_stamped)
    ENABLE_ROS_MSG(nav_msgs::msg::Odometry, odometry)

    std::vector<std::string> get_types()
    {
      std::vector<std::string> keys;
      std::transform(_serializators.begin(), _serializators.end(), std::back_inserter(keys),
        [](auto const& x) { return x.first; } );
      return keys;
    }
  private:
    std::map<std::string, rclcpp::SerializationBase*> _serializators;
};

class RosBagMsgProvider
{
public:
  /**
   * @brief  Constructor for the Ukf class
   *
   * @param[in] args - Generic argument container. It is assumed that args[0]
   * constains the alpha parameter, args[1] contains the kappa parameter, and
   * args[2] contains the beta parameter.
   */

  RosBagMsgProvider() = delete;

  RosBagMsgProvider(std::string bag_path, std::string storage_id = "sqlite3");

  /**
   * @brief  Destructor for the Ukf class
   */
  ~RosBagMsgProvider();

  void set_on_message_callback(std::function<bool(const SerializedMessage*, const TopicMetadata&, Serializator&)> func);

  void start();

  uint64_t get_num_msgs() const;
  uint64_t get_duration() const;
  bool has_next(rclcpp::Time future = rclcpp::Time());
  void get_next(rclcpp::Time future = rclcpp::Time());

  rcutils_time_point_value_t get_start_time();

  bool is_initialized() { return _initialized; }

private:

  Serializator _serializator;
  std::function<bool(const SerializedMessage*, const TopicMetadata&, Serializator&)> _func;
  std::string _bag_path;
  std::string _storage_id;
  std::vector<std::string> _topic_names;
  std::vector<rosbag2_storage::TopicMetadata> _topics;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> _newest_msg;
  rosbag2_cpp::readers::SequentialReader _reader;
  rcutils_time_point_value_t _start_time;
  rclcpp::Time _zero_time;
  bool _initialized;



};

} // rpp_localization
