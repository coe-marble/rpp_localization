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
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_cpp/context_builder.hpp"
#include "rpp_cpp/data_manager.hpp"
#include "rpp_cpp/rpp_paths.hpp"
#include "rpp_localization/inekf/inertial_payload.hpp"
#include "rpp_localization/inekf/invariant_ekf.hpp"
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
/// Every IMU message is one prediction. Poses are fused as positions, and
/// twists and the twist of odometry messages as body-frame velocities, each
/// at the lever arm of its sensor frame. Sensor readings are rotated into
/// the base link axes with the fixed transform of their frame. The filter
/// body sits at the IMU with the axes of the base link; the published
/// odometry is moved to the base link origin.
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
    // Smallest variance accepted on a fused position axis, in m^2.
    minimum_position_variance_ = declare_parameter("minimum_position_variance", 1e-4);
    if (std::isnan(minimum_position_variance_) || minimum_position_variance_ < 0.0) {
      throw std::invalid_argument("minimum_position_variance must not be negative");
    }
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
      // Axes of the position that are fused, as x, y, z.
      const auto position_axes = declare_parameter(
        name + "_position_axes", std::vector<bool>{true, true, true});
      if (position_axes.size() != 3) {
        throw std::invalid_argument(name + "_position_axes must contain 3 values");
      }
      pose_subscriptions_.push_back(
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, rclcpp::SensorDataQoS(),
          [this, name, threshold, sensor_frame, position_axes](
            const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
            on_position(
              message->pose.pose.position, message->pose.covariance,
              position_axes, message->header.frame_id, sensor_frame,
              message->header.stamp, name, threshold);
          }));
    }

    for (std::size_t index = 0;; ++index) {
      const auto name = "odom" + std::to_string(index);
      const auto topic = declare_parameter(name, std::string());
      if (topic.empty()) {
        break;
      }
      const auto threshold = rejection_threshold(name);
      // The velocity of an odometry message is always fused. Its position
      // is fused only on the axes selected here, as x, y, z.
      const auto position_axes = declare_parameter(
        name + "_position_axes", std::vector<bool>{false, false, false});
      if (position_axes.size() != 3) {
        throw std::invalid_argument(name + "_position_axes must contain 3 values");
      }
      odometry_subscriptions_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          topic, rclcpp::SensorDataQoS(),
          [this, name, threshold, position_axes](
            const nav_msgs::msg::Odometry::ConstSharedPtr message) {
            on_body_velocity(
              message->twist.twist.linear, message->twist.covariance,
              message->child_frame_id, message->header.stamp, name, threshold);
            on_position(
              message->pose.pose.position, message->pose.covariance, position_axes,
              message->header.frame_id, message->child_frame_id, message->header.stamp,
              name + "_position", threshold);
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
            on_body_velocity(
              message->twist.twist.linear, message->twist.covariance,
              message->header.frame_id, message->header.stamp, name, threshold);
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

  /// Fixed pose of a sensor frame in the base link frame.
  struct Mounting
  {
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  };

  std::optional<Mounting> mounting(const std::string& frame_id)
  {
    if (frame_id.empty() || frame_id == base_link_frame_id_) {
      return Mounting{};
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_link_frame_id_, frame_id, tf2::TimePointZero).transform;
      Mounting result;
      result.offset = Eigen::Vector3d(
        transform.translation.x, transform.translation.y, transform.translation.z);
      result.rotation = Eigen::Quaterniond(
        transform.rotation.w, transform.rotation.x, transform.rotation.y,
        transform.rotation.z).normalized().toRotationMatrix();
      return result;
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No transform from %s to %s: %s", base_link_frame_id_.c_str(), frame_id.c_str(),
        error.what());
      return std::nullopt;
    }
  }

  void on_imu(const sensor_msgs::msg::Imu& message)
  {
    if (!imu_offset_) {
      const auto imu_mounting = mounting(message.header.frame_id);
      if (!imu_mounting) {
        return;
      }
      imu_offset_ = imu_mounting->offset;
      imu_rotation_ = imu_mounting->rotation;
    }
    Vector6 sample;
    sample <<
      imu_rotation_ * Eigen::Vector3d(
      message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z),
      imu_rotation_ * Eigen::Vector3d(
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z);
    if (!sample.allFinite()) {
      return;
    }

    const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    const Eigen::Quaterniond orientation(
      message.orientation.w, message.orientation.x, message.orientation.y,
      message.orientation.z);
    const bool has_orientation =
      message.orientation_covariance[0] >= 0.0 && orientation.norm() > 0.5;
    // The message gives the attitude of the IMU frame; the filter estimates
    // that of the base link.
    const Eigen::Matrix3d attitude = has_orientation ?
      (orientation.normalized().toRotationMatrix() * imu_rotation_.transpose()).eval() :
      Eigen::Matrix3d::Identity().eval();

    if (!initialized_) {
      initialize(attitude, stamp);
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

    if (use_orientation_ && has_orientation) {
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (int axis = 0; axis < 3; ++axis) {
        const double variance = message.orientation_covariance[4 * axis];
        covariance(axis, axis) = variance > 0.0 ? variance : orientation_variance_;
      }
      InvariantFilter::InvariantMeasurement measurement;
      measurement.kind() = k_attitude;
      plugins::detail::write_row_major(measurement.values(), attitude);
      fill_measurement(
        measurement, covariance, Eigen::Vector3d::Zero(), stamp,
        std::numeric_limits<double>::infinity(), "imu0_attitude");
      accept(filter_->correct(std::move(measurement)), "imu0_attitude");
    }
    publish();
  }

  /// Fuses the selected axes of a world position of the point where
  /// `sensor_frame` is mounted.
  void on_position(
    const geometry_msgs::msg::Point& position,
    const std::array<double, 36>& covariance,
    const std::vector<bool>& axes,
    const std::string& frame_id,
    const std::string& sensor_frame,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& name,
    const double threshold)
  {
    if (!initialized_ || (!axes[0] && !axes[1] && !axes[2])) {
      return;
    }
    if (!frame_id.empty() && frame_id != world_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s must be given in %s, not %s", name.c_str(),
        world_frame_id_.c_str(), frame_id.c_str());
      return;
    }
    const auto sensor = mounting(sensor_frame);
    if (!sensor || !imu_offset_) {
      return;
    }

    // An axis that is not fused is marked as not measured, and the filter
    // leaves it out of the update.
    Eigen::Matrix3d position_covariance = linear_covariance(covariance);
    for (int axis = 0; axis < 3; ++axis) {
      if (!axes[static_cast<std::size_t>(axis)]) {
        position_covariance.row(axis).setZero();
        position_covariance.col(axis).setZero();
        position_covariance(axis, axis) = InEKF::k_unmeasured_axis_variance;
      } else if (position_covariance(axis, axis) < minimum_position_variance_) {
        // A message without a covariance would otherwise be taken as exact.
        position_covariance(axis, axis) = minimum_position_variance_;
      }
    }

    const Eigen::Vector3d measured(position.x, position.y, position.z);
    if (!adopt_first_position(measured, axes, sensor->offset - *imu_offset_, name)) {
      return;
    }

    InvariantFilter::InvariantMeasurement measurement;
    measurement.kind() = k_position;
    plugins::detail::write_values(
      measurement.values(), measured);
    fill_measurement(
      measurement, position_covariance, sensor->offset - *imu_offset_,
      rclcpp::Time(stamp).nanoseconds(), threshold, name);
    accept(filter_->correct(std::move(measurement)), name);
  }

  /// The filter starts at the world origin. The first measurement of a world
  /// axis places the estimate there, since a gated correction could not
  /// cover the distance. The world covariance stays as it was.
  bool adopt_first_position(
    const Eigen::Vector3d& measured,
    const std::vector<bool>& axes,
    const Eigen::Vector3d& arm,
    const std::string& name)
  {
    const Eigen::Vector3d imu_position = measured - state_.R()() * arm;
    Eigen::Vector3d shift = Eigen::Vector3d::Zero();
    bool adopted = false;
    for (int axis = 0; axis < 3; ++axis) {
      const auto index = static_cast<std::size_t>(axis);
      if (axes[index] && !position_known_[index]) {
        shift[axis] = imu_position[axis] - state_[1][axis];
        position_known_[index] = true;
        adopted = true;
      }
    }
    if (!adopted) {
      return true;
    }

    InEKF::InertialLieState::MatrixState matrix = state_();
    matrix.block<3, 1>(0, 4) += shift;
    InEKF::InertialLieState::MatrixCov jacobian = InEKF::InertialLieState::MatrixCov::Identity();
    jacobian.block<3, 3>(InEKF::k_inertial_position, InEKF::k_inertial_rotation) =
      InEKF::SO3<>::wedge(shift);
    const InEKF::InertialLieState::MatrixCov covariance =
      jacobian * state_.cov() * jacobian.transpose();

    InvariantFilter::InertialEstimate estimate;
    plugins::detail::write_inertial_state(
      estimate.state(), InEKF::InertialLieState(matrix, covariance, state_.aug()));
    plugins::detail::write_values(estimate.angularVelocity(), angular_velocity_);
    estimate.referenceTimeNs() = reference_time_;
    const auto status = filter_->initialize(std::move(estimate));
    if (status.code() != plugins::detail::k_inertial_ok) {
      RCLCPP_WARN(
        get_logger(), "%s could not place the estimate: %s", name.c_str(),
        std::string(status.message()).c_str());
      return false;
    }
    state_ = InEKF::InertialLieState(matrix, covariance, state_.aug());
    return true;
  }

  /// Fuses a linear velocity measured in `frame_id` as the body velocity
  /// of the point where that frame is mounted.
  void on_body_velocity(
    const geometry_msgs::msg::Vector3& linear,
    const std::array<double, 36>& covariance,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& name,
    const double threshold)
  {
    if (!initialized_) {
      return;
    }
    const auto sensor = mounting(frame_id);
    if (!sensor || !imu_offset_) {
      return;
    }

    InvariantFilter::InvariantMeasurement measurement;
    measurement.kind() = k_body_velocity;
    plugins::detail::write_values(
      measurement.values(),
      (sensor->rotation * Eigen::Vector3d(linear.x, linear.y, linear.z)).eval());
    fill_measurement(
      measurement,
      sensor->rotation * linear_covariance(covariance) * sensor->rotation.transpose(),
      sensor->offset - *imu_offset_, rclcpp::Time(stamp).nanoseconds(), threshold, name);
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
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odometry_subscriptions_;

  double minimum_position_variance_{0.0};
  bool initialized_{false};
  std::array<bool, 3> position_known_{false, false, false};
  std::int64_t reference_time_{0};
  std::optional<Eigen::Vector3d> imu_offset_;
  Eigen::Matrix3d imu_rotation_ = Eigen::Matrix3d::Identity();
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
