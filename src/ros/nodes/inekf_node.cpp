#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_cpp/context_builder.hpp"
#include "rpp_cpp/data_manager.hpp"
#include "rpp_cpp/rpp_paths.hpp"
#include "rpp_localization/inekf/inertial_payload.hpp"
#include "rpp_localization/inekf/so3.hpp"
#include "rpp_localization/ros/invariant_localization_script.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace rpp_localization
{

/// Drives an InvariantFilter composition from ROS topics.
///
/// Every IMU message is one prediction. Poses are fused as positions and
/// twists as body-frame velocities, each at the lever arm of its sensor
/// frame. The filter body is the IMU; the published odometry is converted
/// to the base link through the fixed transform between the two. Sensor
/// frames are assumed to be aligned with the base link.
class InvariantLocalizationNode : public rclcpp::Node
{
public:
  InvariantLocalizationNode()
  : Node("inekf_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    world_frame_id_ = declare_parameter("world_frame", std::string("odom"));
    base_link_frame_id_ = declare_parameter("base_link_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    use_orientation_ = declare_parameter("imu0_use_orientation", true);
    orientation_variance_ = declare_parameter("imu0_orientation_variance", 1e-4);
    initial_covariance_ = declare_parameter(
      "initial_estimate_covariance",
      std::vector<double>{
        1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
        1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});
    if (initial_covariance_.size() != 15) {
      throw std::invalid_argument(
              "initial_estimate_covariance must list the 15 tangent-space variances");
    }

    load_filter();

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry/filtered", rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    const auto imu_topic = declare_parameter("imu0", std::string("imu/data"));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(*message);});

    for (std::size_t index = 0;; ++index) {
      const auto name = "pose" + std::to_string(index);
      const auto topic = declare_parameter(name, std::string());
      if (topic.empty()) {
        break;
      }
      const auto threshold = rejection_threshold(name);
      // A pose is stamped with the world frame, so its sensor frame is a
      // parameter. Empty means the sensor sits at the base link origin.
      const auto sensor_frame = declare_parameter(name + "_frame", std::string());
      pose_subscriptions_.push_back(
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, rclcpp::SensorDataQoS(),
          [this, name, threshold, sensor_frame](
            const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
            on_pose(*message, name, sensor_frame, threshold);
          }));
    }

    for (std::size_t index = 0;; ++index) {
      const auto name = "twist" + std::to_string(index);
      const auto topic = declare_parameter(name, std::string());
      if (topic.empty()) {
        break;
      }
      const auto threshold = rejection_threshold(name);
      twist_subscriptions_.push_back(
        create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          topic, rclcpp::SensorDataQoS(),
          [this, name, threshold](
            const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr message) {
            on_twist(*message, name, threshold);
          }));
    }
  }

private:
  using Vector6 = Eigen::Matrix<double, 6, 1>;

  static constexpr std::uint16_t k_position = 0;
  static constexpr std::uint16_t k_body_velocity = 1;
  static constexpr std::uint16_t k_attitude = 2;

  void load_filter()
  {
    const auto reference = declare_parameter(
      "script", std::string("rpp_localization::invariant_localization"));
    const auto configuration = declare_parameter("configuration", std::string("inekf"));
    const auto workspace = declare_parameter("rpp_workspace", std::string());

    const auto separator = reference.find("::");
    if (separator == std::string::npos || separator == 0 ||
      separator + 2 >= reference.size())
    {
      throw std::invalid_argument("script must use the library::script_name format");
    }
    const auto library = reference.substr(0, separator);
    const auto script_name = reference.substr(separator + 2);

    rpp::RppDataManager data_manager(
      rpp::RPP_HOME,
      workspace.empty() ? ament_index_cpp::get_package_share_path(library).string() : workspace);
    rpp::ComponentContextBuilder context_builder(data_manager);
    const std::optional<std::string> selected_configuration = configuration.empty() ?
      std::nullopt : std::optional<std::string>(configuration);
    context_ = std::make_unique<rpp::ComponentContext>(
      context_builder.build_script_from_library(library, script_name, selected_configuration));
    script_ = std::make_unique<InvariantLocalizationScript>(*context_);
    script_->initialize();
    filter_ = script_->filter();
  }

  double rejection_threshold(const std::string& name)
  {
    const double threshold = declare_parameter(
      name + "_rejection_threshold", std::numeric_limits<double>::infinity());
    if (std::isnan(threshold) || threshold <= 0.0) {
      throw std::invalid_argument(name + "_rejection_threshold must be positive");
    }
    return threshold;
  }

  /// Position of a frame relative to the base link, in the base link frame.
  std::optional<Eigen::Vector3d> frame_offset(const std::string& frame_id)
  {
    if (frame_id.empty() || frame_id == base_link_frame_id_) {
      return Eigen::Vector3d::Zero();
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_link_frame_id_, frame_id, tf2::TimePointZero);
      const auto& translation = transform.transform.translation;
      return Eigen::Vector3d(translation.x, translation.y, translation.z);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No transform from %s to %s: %s", base_link_frame_id_.c_str(), frame_id.c_str(),
        error.what());
      return std::nullopt;
    }
  }

  /// Position of a sensor frame relative to the IMU, which is the filter body.
  std::optional<Eigen::Vector3d> lever_arm(const std::string& frame_id)
  {
    const auto offset = frame_offset(frame_id);
    if (!offset || !imu_offset_) {
      return std::nullopt;
    }
    return *offset - *imu_offset_;
  }

  void on_imu(const sensor_msgs::msg::Imu& message)
  {
    if (!imu_offset_) {
      imu_offset_ = frame_offset(message.header.frame_id);
      if (!imu_offset_) {
        return;
      }
    }
    Vector6 sample;
    sample <<
      message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z;
    if (!sample.allFinite()) {
      return;
    }

    const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    const bool has_orientation = message.orientation_covariance[0] >= 0.0;
    const Eigen::Quaterniond orientation(
      message.orientation.w, message.orientation.x, message.orientation.y,
      message.orientation.z);

    if (!initialized_) {
      initialize(
        has_orientation && orientation.norm() > 0.5 ?
        orientation.normalized().toRotationMatrix() : Eigen::Matrix3d::Identity().eval(),
        stamp);
      return;
    }
    if (stamp <= reference_time_) {
      return;
    }

    InvariantFilter::InvariantPredictInput input;
    plugins::detail::write_imu(input.imu(), sample);
    input.referenceTimeNs() = stamp;
    input.deltaNs() = stamp - reference_time_;
    if (!accept(filter_->predict(std::move(input)), "imu0")) {
      return;
    }
    reference_time_ = stamp;

    if (use_orientation_ && has_orientation && orientation.norm() > 0.5) {
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (int axis = 0; axis < 3; ++axis) {
        const double variance = message.orientation_covariance[4 * axis];
        covariance(axis, axis) = variance > 0.0 ? variance : orientation_variance_;
      }
      InvariantFilter::InvariantMeasurement measurement;
      measurement.kind() = k_attitude;
      plugins::detail::write_row_major(
        measurement.values(), orientation.normalized().toRotationMatrix());
      fill_measurement(
        measurement, covariance, Eigen::Vector3d::Zero(), stamp,
        std::numeric_limits<double>::infinity(), "imu0_attitude");
      accept(filter_->correct(std::move(measurement)), "imu0_attitude");
    }
    publish();
  }

  void on_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& message,
    const std::string& name,
    const std::string& sensor_frame,
    const double threshold)
  {
    if (!initialized_) {
      return;
    }
    if (!message.header.frame_id.empty() && message.header.frame_id != world_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s must be given in %s, not %s", name.c_str(),
        world_frame_id_.c_str(), message.header.frame_id.c_str());
      return;
    }
    const auto arm = lever_arm(sensor_frame);
    if (!arm) {
      return;
    }

    const auto& position = message.pose.pose.position;
    InvariantFilter::InvariantMeasurement measurement;
    measurement.kind() = k_position;
    plugins::detail::write_values(
      measurement.values(), Eigen::Vector3d(position.x, position.y, position.z));
    fill_measurement(
      measurement, linear_covariance(message.pose.covariance), *arm,
      rclcpp::Time(message.header.stamp).nanoseconds(), threshold, name);
    accept(filter_->correct(std::move(measurement)), name);
  }

  void on_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped& message,
    const std::string& name,
    const double threshold)
  {
    if (!initialized_) {
      return;
    }
    const auto arm = lever_arm(message.header.frame_id);
    if (!arm) {
      return;
    }

    const auto& linear = message.twist.twist.linear;
    InvariantFilter::InvariantMeasurement measurement;
    measurement.kind() = k_body_velocity;
    plugins::detail::write_values(
      measurement.values(), Eigen::Vector3d(linear.x, linear.y, linear.z));
    fill_measurement(
      measurement, linear_covariance(message.twist.covariance), *arm,
      rclcpp::Time(message.header.stamp).nanoseconds(), threshold, name);
    accept(filter_->correct(std::move(measurement)), name);
  }

  static Eigen::Matrix3d linear_covariance(const std::array<double, 36>& covariance)
  {
    Eigen::Matrix3d linear;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        linear(row, column) = covariance[6 * row + column];
      }
    }
    return linear;
  }

  /// The filter keeps no history, so a measurement that is newer than the
  /// last IMU sample is applied to the current estimate.
  void fill_measurement(
    InvariantFilter::InvariantMeasurement& measurement,
    const Eigen::Matrix3d& covariance,
    const Eigen::Vector3d& arm,
    const std::int64_t stamp,
    const double threshold,
    const std::string& name) const
  {
    plugins::detail::write_row_major(measurement.covariance(), covariance);
    plugins::detail::write_values(measurement.leverArm(), arm);
    measurement.referenceTimeNs() = std::min(stamp, reference_time_);
    measurement.mahalanobisThreshold() = threshold;
    measurement.sourceName() = name;
  }

  void initialize(const Eigen::Matrix3d& rotation, const std::int64_t stamp)
  {
    InEKF::InertialLieState::MatrixState matrix = InEKF::InertialLieState::MatrixState::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    // The base link starts at the world origin, so the IMU starts at its
    // offset from the base link.
    matrix.block<3, 1>(0, 4) = rotation * (*imu_offset_);
    InEKF::InertialLieState::MatrixCov covariance = InEKF::InertialLieState::MatrixCov::Zero();
    for (int index = 0; index < 15; ++index) {
      covariance(index, index) = initial_covariance_[static_cast<std::size_t>(index)];
    }

    InvariantFilter::InertialEstimate estimate;
    plugins::detail::write_inertial_state(
      estimate.state(),
      InEKF::InertialLieState(matrix, covariance, InEKF::InertialLieState::VectorAug::Zero()));
    plugins::detail::write_values(estimate.angularVelocity(), Eigen::Vector3d::Zero().eval());
    estimate.referenceTimeNs() = stamp;

    const auto status = filter_->initialize(std::move(estimate));
    if (status.code() != plugins::detail::k_inertial_ok) {
      throw std::runtime_error("invariant filter initialization failed: " + std::string(status.message()));
    }
    reference_time_ = stamp;
    initialized_ = true;
  }

  /// Keeps the estimate of a successful call and reports a failed one.
  template<typename Result>
  bool accept(const Result& result, const std::string& name)
  {
    const auto status = result.status();
    if (status.code() != plugins::detail::k_inertial_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s was not applied: %s", name.c_str(),
        std::string(status.message()).c_str());
      return false;
    }
    try {
      const auto estimate = result.estimate();
      state_ = plugins::detail::read_inertial_state(estimate.state());
      angular_velocity_ =
        plugins::detail::read_vector3(estimate.angularVelocity(), "angularVelocity");
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s returned an invalid estimate: %s",
        name.c_str(), error.what());
      return false;
    }
    return true;
  }

  void publish()
  {
    const Eigen::Matrix3d rotation = state_.R()();
    const Eigen::Vector3d position = state_[1] - rotation * (*imu_offset_);
    const Eigen::Vector3d body_velocity =
      rotation.transpose() * state_[0] - angular_velocity_.cross(*imu_offset_);
    const Eigen::Quaterniond orientation(rotation);
    const auto stamp = rclcpp::Time(reference_time_, get_clock()->get_clock_type());

    // World position and rotation errors of the base link in terms of the
    // invariant error.
    Eigen::Matrix<double, 6, 15> pose_map = Eigen::Matrix<double, 6, 15>::Zero();
    pose_map.block<3, 3>(0, InEKF::k_inertial_position) = Eigen::Matrix3d::Identity();
    pose_map.block<3, 3>(0, InEKF::k_inertial_rotation) = -InEKF::SO3<>::wedge(position);
    pose_map.block<3, 3>(3, InEKF::k_inertial_rotation) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix<double, 6, 6> pose_covariance =
      pose_map * state_.cov() * pose_map.transpose();
    const Eigen::Matrix3d velocity_covariance =
      rotation.transpose() *
      state_.cov().block<3, 3>(InEKF::k_inertial_velocity, InEKF::k_inertial_velocity) *
      rotation;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_id_;
    odometry.child_frame_id = base_link_frame_id_;
    odometry.pose.pose.position.x = position.x();
    odometry.pose.pose.position.y = position.y();
    odometry.pose.pose.position.z = position.z();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = body_velocity.x();
    odometry.twist.twist.linear.y = body_velocity.y();
    odometry.twist.twist.linear.z = body_velocity.z();
    odometry.twist.twist.angular.x = angular_velocity_.x();
    odometry.twist.twist.angular.y = angular_velocity_.y();
    odometry.twist.twist.angular.z = angular_velocity_.z();
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        odometry.pose.covariance[6 * row + column] = pose_covariance(row, column);
      }
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        odometry.twist.covariance[6 * row + column] = velocity_covariance(row, column);
      }
    }
    odometry_publisher_->publish(odometry);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_link_frame_id_;
      transform.transform.translation.x = position.x();
      transform.transform.translation.y = position.y();
      transform.transform.translation.z = position.z();
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::string world_frame_id_;
  std::string base_link_frame_id_;
  bool publish_tf_{true};
  bool use_orientation_{true};
  double orientation_variance_{1e-4};
  std::vector<double> initial_covariance_;

  std::unique_ptr<rpp::ComponentContext> context_;
  std::unique_ptr<InvariantLocalizationScript> script_;
  std::shared_ptr<InvariantFilter> filter_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
  pose_subscriptions_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr>
  twist_subscriptions_;

  bool initialized_{false};
  std::int64_t reference_time_{0};
  std::optional<Eigen::Vector3d> imu_offset_;
  InEKF::InertialLieState state_;
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
};

}  // namespace rpp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpp_localization::InvariantLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
