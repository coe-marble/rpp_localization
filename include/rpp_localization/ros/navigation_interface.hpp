#pragma once

#include <rpp_localization/estimation/prepared_queue.hpp>
#include <rpp_ros_interfaces/msg/estimator_estimate.hpp>
#include <rpp_ros_interfaces/msg/numeric_interface.hpp>
#include <rpp_ros_interfaces/msg/numeric_sample.hpp>
#include <rpp_ros_interfaces/srv/set_pose.hpp>
#include <rpp_interfaces/field_layout.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <array>
#include <limits>
#include <map>
#include <optional>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace rpp_localization
{
class EstimatorScriptNode;

// Optional navigation facade. ROS decoding/publication runs on the executor;
// only prediction_policy() runs on the numerical worker. They share immutable
// routes and exchange commands through their own prepared SPSC queue.
class NavigationInterface
{
public:
  explicit NavigationInterface(EstimatorScriptNode& node);
  void accept_layout();
  void reset();  // Management thread, with the worker paused.
  void process_measurements(std::uint64_t time);  // ROS executor, after clock-reset handling.
  void publish(const rpp_ros_interfaces::msg::EstimatorEstimate& estimate);

private:
  static constexpr std::size_t missing = std::numeric_limits<std::size_t>::max();
  struct Binding
  {
    Binding() {values.fill(missing); tangent.fill(missing);}
    std::array<std::size_t, 15> values, tangent;
    std::optional<std::size_t> quaternion;
    std::uint16_t mask{0};
  };
  struct Port
  {
    std::uint32_t index{0};
    Binding binding;
    std::vector<std::size_t> covariance_roles;
    std::vector<double> values, covariance;
  };
  struct Control
  {
    std::array<double, 6> values{};
    std::uint64_t time{0};
  };
  struct PoseHistory
  {
    bool anchored{false}, previous{false};
    tf2::Transform anchor, pose;
    Eigen::Matrix<double, 6, 6> covariance;
    std::uint64_t time{0};
  };
  using SensorMessage = std::variant<
    nav_msgs::msg::Odometry::ConstSharedPtr,
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr,
    geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr,
    sensor_msgs::msg::Imu::ConstSharedPtr>;
  struct PendingMeasurement
  {
    std::uint64_t time;
    SensorMessage message;
  };
  using Covariance = Eigen::Matrix<double, 15, 15>;
  void queue_measurement(SensorMessage message);
  Binding bind(const rpp_interfaces::FieldLayout& layout, bool complete) const;
  std::optional<tf2::Transform> transform(const std::string& target, const std::string& source,
    const builtin_interfaces::msg::Time& stamp);
  void odometry(const nav_msgs::msg::Odometry& message);
  void pose(const geometry_msgs::msg::PoseWithCovarianceStamped& message, bool from_odometry = false);
  void set_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& message);
  void twist(const geometry_msgs::msg::TwistWithCovarianceStamped& message, std::uint16_t selection);
  void imu(const sensor_msgs::msg::Imu& message);
  void control(const geometry_msgs::msg::Twist& message);
  void submit(std::uint16_t requested, const std::array<double, 15>& values,
    const Covariance& covariance, const builtin_interfaces::msg::Time& stamp,
    const std::array<double, 4>& quaternion);
  std::uint8_t prediction_policy(std::uint64_t end, std::uint64_t dt,
    const rpp_ros_interfaces::msg::EstimatorEstimate& state,
    rpp::List<double>& input, rpp::List<double>& q);

  EstimatorScriptNode& node_;
  std::string resource_, world_, body_, imu_world_, pose_child_;
  std::array<PoseHistory, 2> pose_history_;
  std::array<bool, 2> relative_pose_{}, differential_pose_{};
  bool two_d_{false};
  std::uint16_t observed_coordinates_{0}, planar_constraints_{0};
  Binding state_;
  std::map<std::uint16_t, Port> ports_;
  std::vector<PendingMeasurement> pending_measurements_;
  std::array<bool, 6> control_axes_{};
  std::array<double, 6> acceleration_limits_{}, acceleration_gains_{}, deceleration_limits_{}, deceleration_gains_{};
  std::array<std::size_t, 9> control_route_{};
  estimation::PreparedQueue<Control> controls_;
  Control worker_control_;
  std::uint64_t control_timeout_{0}, last_published_{0};
  bool use_control_{false}, dynamic_noise_{false}, remove_gravity_{false}, published_{false};
  bool permit_corrected_{false};
  bool publish_control_state_{false};
  std::size_t control_state_dimensions_{2};
  rpp_ros_interfaces::msg::NumericSample control_state_;
  std::optional<rpp_interfaces::FieldLayout> control_state_layout_;
  rclcpp::Publisher<rpp_ros_interfaces::msg::NumericInterface>::SharedPtr control_layout_publisher_;
  rclcpp::Publisher<rpp_ros_interfaces::msg::NumericSample>::SharedPtr control_state_publisher_;
  std::uint16_t pose_selection_{0x003f}, twist_selection_{0x0fc0}, acceleration_selection_{0x7000};
  std::uint16_t odometry_selection_{0x0fff}, imu_selection_{0x7e38};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::array<double, 3> previous_gyro_{};
  Eigen::Matrix3d previous_gyro_covariance_{Eigen::Matrix3d::Zero()};
  std::optional<std::uint64_t> previous_gyro_time_;
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transforms_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_subscription_;
  rclcpp::Service<rpp_ros_interfaces::srv::SetPose>::SharedPtr set_pose_service_;
};
}  // namespace rpp_localization
