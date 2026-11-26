#include <rpp_localization/ros/navigation_interface.hpp>
#include <rpp_localization/ros/estimator_script_node.hpp>
#include <rpp_localization/ros/covariance_helpers.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>

namespace rpp_localization
{
namespace h = ros_helpers;
namespace c = rpp_interfaces::contract;
namespace
{
constexpr std::uint16_t pose_mask = 0x003f, twist_mask = 0x0fc0, acceleration_mask = 0x7000;
std::array<double, 15> absent()
{std::array<double, 15> values; values.fill(std::numeric_limits<double>::quiet_NaN()); return values;}
bool orientation(const geometry_msgs::msg::Quaternion& quaternion,
  std::array<double, 15>& values, std::array<double, 4>& raw)
{
  raw = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  double norm = 0.0; for (const auto value : raw) {norm += value * value;}
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {return false;}
  tf2::Matrix3x3(tf2::Quaternion(raw[0], raw[1], raw[2], raw[3])).getRPY(values[3], values[4], values[5]);
  return true;
}
int axis(const std::string& name)
{
  const auto suffix = name.substr(name.find_last_of('.') == std::string::npos ? 0 : name.find_last_of('.') + 1);
  if (suffix == "roll" || suffix == "vroll" || suffix == "aroll" || suffix == "wx") {return 0;}
  if (suffix == "pitch" || suffix == "vpitch" || suffix == "apitch" || suffix == "wy") {return 1;}
  if (suffix == "yaw" || suffix == "vyaw" || suffix == "ayaw" || suffix == "wz") {return 2;}
  if (!suffix.empty() && suffix.back() == 'x') {return 0;}
  if (!suffix.empty() && suffix.back() == 'y') {return 1;}
  if (!suffix.empty() && suffix.back() == 'z') {return 2;}
  return -1;
}
Eigen::Matrix3d rotation(const tf2::Transform& transform)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {for (int column = 0; column < 3; ++column) {result(row, column) = transform.getBasis()[row][column];}}
  return result;
}
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}
std::uint16_t rotate_selection(std::uint16_t selection, std::size_t offset, const tf2::Transform& frame)
{
  std::uint16_t result = 0;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      if ((selection & (1u << (offset + column))) && std::abs(frame.getBasis()[row][column]) > 1e-6) {
        result |= 1u << (offset + row);
      }
    }
  }
  return result;
}
}

NavigationInterface::NavigationInterface(EstimatorScriptNode& node) : node_(node)
{
  resource_ = h::parameter<std::string>(node_, "navigation.resource", "base");
  use_control_ = h::parameter(node_, "navigation.use_control", false);
  dynamic_noise_ = h::parameter(node_, "navigation.dynamic_process_noise", false);
  remove_gravity_ = h::parameter(node_, "navigation.remove_gravity", false);
  permit_corrected_ = h::parameter(node_, "navigation.permit_corrected_publication", false);
  two_d_ = h::parameter(node_, "navigation.two_d_mode", false);
  relative_pose_ = {h::parameter(node_, "navigation.pose_relative", false), h::parameter(node_, "navigation.odometry_relative", false)};
  differential_pose_ = {h::parameter(node_, "navigation.pose_differential", false), h::parameter(node_, "navigation.odometry_differential", false)};
  for (std::size_t source = 0; source < 2; ++source) {
    if (relative_pose_[source] && differential_pose_[source]) {throw std::invalid_argument("Pose input cannot be both relative and differential");}
  }
  publish_control_state_ = h::parameter(node_, "navigation.publish_control_state", false);
  const auto dimensions = h::parameter<std::int64_t>(node_, "navigation.control_state_dimensions", 2);
  if (dimensions != 2 && dimensions != 3) {throw std::invalid_argument("navigation.control_state_dimensions must be 2 or 3");}
  control_state_dimensions_ = static_cast<std::size_t>(dimensions);
  if (publish_control_state_) {
    control_layout_publisher_ = node_.create_publisher<rpp_ros_interfaces::msg::NumericInterface>("~/navigation/layout", rclcpp::QoS(1).transient_local());
    control_state_publisher_ = node_.create_publisher<rpp_ros_interfaces::msg::NumericSample>("~/navigation/state", rclcpp::QoS(10));
  }
  const auto timeout = h::parameter(node_, "navigation.control_timeout_sec", 0.2);
  if (!std::isfinite(timeout) || timeout < 0 || timeout > 1e9) {throw std::invalid_argument("Invalid navigation.control_timeout_sec");}
  control_timeout_ = static_cast<std::uint64_t>(timeout * 1e9);
  const auto enabled = h::parameter(node_, "navigation.control_config", std::vector<bool>(6, false));
  if (enabled.size() != 6) {throw std::invalid_argument("navigation.control_config must contain six flags");}
  for (std::size_t index = 0; index < 6; ++index) {control_axes_[index] = enabled[index];}
  const auto load = [&](const std::string& name, std::array<double, 6>& output) {
    const auto values = h::parameter(node_, "navigation." + name, std::vector<double>(6, 1.0));
    if (values.size() != 6) {throw std::invalid_argument(name + " must contain six values");}
    for (std::size_t index = 0; index < 6; ++index) {
      if (!std::isfinite(values[index]) || values[index] < 0) {throw std::invalid_argument(name + " must be finite and nonnegative");}
      output[index] = values[index];
    }
  };
  load("acceleration_limits", acceleration_limits_); load("acceleration_gains", acceleration_gains_);
  load("deceleration_limits", deceleration_limits_); load("deceleration_gains", deceleration_gains_);
  controls_.prepare(16, [] {return Control{};});
  pending_measurements_.reserve(node_.queue_capacity_);
  accept_layout();
  pose_child_ = h::parameter<std::string>(node_, "navigation.pose_child_frame", body_);
  imu_world_ = h::parameter<std::string>(node_, "navigation.imu_world_frame", world_);
  const auto selection = [&](const std::string& name, std::size_t width, std::size_t offset) {
    const auto flags = h::parameter(node_, "navigation." + name, std::vector<bool>(width, true));
    if (flags.size() != width) {throw std::invalid_argument(name + " has an invalid width");}
    std::uint16_t mask = 0;
    for (std::size_t index = 0; index < width; ++index) {if (flags[index]) {mask |= 1u << (offset + index);}}
    return mask;
  };
  pose_selection_ = selection("pose_axes", 6, 0); twist_selection_ = selection("twist_axes", 6, 6);
  acceleration_selection_ = selection("acceleration_axes", 3, 12);
  const auto source_selection = [&](const std::string& name, std::uint16_t supported) {
    std::vector<bool> defaults(15, false);
    for (std::size_t axis = 0; axis < 15; ++axis) {defaults[axis] = supported & (1u << axis);}
    const auto flags = h::parameter(node_, "navigation." + name, defaults);
    if (flags.size() != 15) {throw std::invalid_argument(name + " must contain fifteen flags");}
    std::uint16_t result = 0;
    for (std::size_t axis = 0; axis < 15; ++axis) {
      if (flags[axis]) {
        if (!(supported & (1u << axis))) {throw std::invalid_argument(name + " selects an unsupported sensor coordinate");}
        result |= 1u << axis;
      }
    }
    return result;
  };
  odometry_selection_ = source_selection("odometry_axes", pose_mask | twist_mask);
  imu_selection_ = source_selection("imu_axes", 0x0038 | 0x0e00 | acceleration_mask);
  imu_selection_ &= static_cast<std::uint16_t>(~acceleration_mask) | acceleration_selection_;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node_, false);
  odometry_publisher_ = node_.create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::QoS(10));
  if (h::parameter(node_, "navigation.publish_acceleration", false)) {
    acceleration_publisher_ = node_.create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("accel/filtered", rclcpp::QoS(10));
  }
  if (h::parameter(node_, "navigation.publish_tf", false)) {transforms_ = std::make_unique<tf2_ros::TransformBroadcaster>(&node_);}
  const auto odometry_topic = h::parameter<std::string>(node_, "navigation.odometry_topic", "odometry/input");
  const auto pose_topic = h::parameter<std::string>(node_, "navigation.pose_topic", "pose/input");
  const auto twist_topic = h::parameter<std::string>(node_, "navigation.twist_topic", "twist/input");
  const auto imu_topic = h::parameter<std::string>(node_, "navigation.imu_topic", "imu/data");
  const auto sensor_qos = [&](const std::string& name) {
    const auto depth = h::parameter<std::int64_t>(node_, "navigation." + name + "_queue_size", 10);
    if (depth < 1 || depth > 4096) {throw std::invalid_argument("Invalid navigation sensor queue size");}
    auto qos = rclcpp::SensorDataQoS(); qos.keep_last(static_cast<std::size_t>(depth)); return qos;
  };
  if (!odometry_topic.empty()) {
    odometry_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, sensor_qos("odometry"),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr value) {queue_measurement(std::move(value));});
  }
  if (!pose_topic.empty()) {
    pose_subscription_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, sensor_qos("pose"),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr value) {queue_measurement(std::move(value));});
  }
  if (!twist_topic.empty()) {
    twist_subscription_ = node_.create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      twist_topic, sensor_qos("twist"),
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr value) {queue_measurement(std::move(value));});
  }
  if (!imu_topic.empty()) {
    imu_subscription_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos("imu"),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr value) {queue_measurement(std::move(value));});
  }
  if (use_control_) {control_subscription_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::ConstSharedPtr value) {control(*value);});}
  set_pose_subscription_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "set_pose", rclcpp::QoS(1), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr value) {set_pose(*value);});
  set_pose_service_ = node_.create_service<rpp_ros_interfaces::srv::SetPose>("set_pose",
    [this](rpp_ros_interfaces::srv::SetPose::Request::SharedPtr request, rpp_ros_interfaces::srv::SetPose::Response::SharedPtr) {set_pose(request->pose);});
  if (use_control_ || dynamic_noise_) {
    node_.set_prediction_policy([this](auto time, auto dt, const auto& state, auto& input, auto& q) {
      return prediction_policy(time, dt, state, input, q);
    });
  }
}

NavigationInterface::Binding NavigationInterface::bind(const rpp_interfaces::FieldLayout& layout, bool complete) const
{
  Binding result;
  for (std::size_t index = 0; index < layout.fields().size(); ++index) {
    const auto& field = layout.fields()[index]; const auto range = layout.ranges()[index];
    if (field.resource != resource_) {if (complete) {throw std::invalid_argument("Navigation measurement mixes resources");} continue;}
    int first = field.quantity == c::QUANTITY_LINEAR_POSITION ? 0 : field.quantity == c::QUANTITY_ANGULAR_POSITION ? 3 :
      field.quantity == c::QUANTITY_LINEAR_VELOCITY ? 6 : field.quantity == c::QUANTITY_ANGULAR_VELOCITY ? 9 :
      field.quantity == c::QUANTITY_LINEAR_ACCELERATION ? 12 : -1;
    if (field.quantity == c::QUANTITY_ORIENTATION && field.kind == c::COORDINATE_QUATERNION) {
      if (result.mask & 0x0038) {throw std::invalid_argument("Repeated navigation orientation");}
      result.quaternion = range.offset;
      for (std::size_t component = 0; component < 3; ++component) {result.tangent[3 + component] = range.tangent_offset + component;}
      result.mask |= 0x0038; continue;
    }
    const bool linear = first == 0 || first == 6 || first == 12;
    if (first < 0 || (range.size != 1 && range.size != 3 && !(linear && range.size == 2))) {
      if (complete) {throw std::invalid_argument("Unsupported navigation measurement field");} continue;
    }
    const auto suffix = field.name.substr(field.name.find_last_of('.') == std::string::npos ? 0 : field.name.find_last_of('.') + 1);
    const bool planar_angle = range.size == 1 && ((first == 3 && suffix == "orientation") ||
      (first == 9 && suffix == "angularVelocity"));
    const int start = range.size > 1 ? 0 : planar_angle ? 2 : axis(field.name);
    if (start < 0) {if (complete) {throw std::invalid_argument("Navigation scalar field needs an axis suffix");} continue;}
    for (std::size_t component = 0; component < range.size; ++component) {
      const auto role = static_cast<std::size_t>(first + start) + component;
      if (result.mask & (1u << role)) {throw std::invalid_argument("Repeated navigation coordinate");}
      result.values[role] = range.offset + component; result.tangent[role] = range.tangent_offset + component;
      result.mask |= 1u << role;
    }
  }
  return result;
}

void NavigationInterface::accept_layout()
{
  state_ = bind(node_.state_layout(), false);
  if ((state_.mask & 0x0023) != 0x0023) {throw std::invalid_argument("Navigation output needs at least x/y position and yaw for the selected resource");}
  world_.clear(); body_.clear();
  for (const auto& field : node_.state_layout().fields()) {
    if (field.resource != resource_) {continue;}
    if (field.quantity == c::QUANTITY_LINEAR_POSITION || field.quantity == c::QUANTITY_ANGULAR_POSITION || field.quantity == c::QUANTITY_ORIENTATION) {
      if (!world_.empty() && world_ != field.frameId) {throw std::invalid_argument("Navigation pose fields use different world frames");}
      world_ = field.frameId;
      if (!body_.empty() && body_ != field.childFrameId) {throw std::invalid_argument("Navigation pose fields use different body frames");}
      body_ = field.childFrameId;
    }
  }
  if (world_.empty() || body_.empty() || world_ == body_) {throw std::invalid_argument("Navigation needs distinct nonempty world and body frames");}
  for (const auto& field : node_.state_layout().fields()) {
    if (field.resource == resource_ && (field.quantity == c::QUANTITY_LINEAR_VELOCITY ||
      field.quantity == c::QUANTITY_ANGULAR_VELOCITY || field.quantity == c::QUANTITY_LINEAR_ACCELERATION) && field.frameId != body_) {
      throw std::invalid_argument("Navigation velocity and acceleration must use the body frame");
    }
  }
  ports_.clear(); observed_coordinates_ = 0;
  for (std::size_t index = 0; index < node_.measurement_layouts().size(); ++index) {
    const auto& layout = node_.measurement_layouts()[index];
    try {
      Port port; port.index = static_cast<std::uint32_t>(index); port.binding = bind(layout, true);
      if (!port.binding.mask) {continue;}
      for (const auto& field : layout.fields()) {
        const bool pose_field = field.quantity == c::QUANTITY_LINEAR_POSITION || field.quantity == c::QUANTITY_ANGULAR_POSITION || field.quantity == c::QUANTITY_ORIENTATION;
        if (field.frameId != (pose_field ? world_ : body_) || field.childFrameId != body_) {
          throw std::invalid_argument("Navigation observation frame does not match its state role");
        }
      }
      port.values.resize(layout.size()); port.covariance.resize(layout.tangent_size() * layout.tangent_size());
      port.covariance_roles.resize(layout.tangent_size(), missing);
      for (std::size_t role = 0; role < 15; ++role) {
        if (port.binding.tangent[role] != missing) {port.covariance_roles[port.binding.tangent[role]] = role;}
      }
      if (std::find(port.covariance_roles.begin(), port.covariance_roles.end(), missing) != port.covariance_roles.end()) {continue;}
      observed_coordinates_ |= port.binding.mask;
      ports_.try_emplace(port.binding.mask, std::move(port));
    } catch (const std::invalid_argument&) {continue;}  // Other observation models remain generic-only ports.
  }
  if (use_control_) {
    const std::array<const char*, 9> names{"control.ax", "control.ay", "control.az", "control.aroll", "control.apitch",
      "control.ayaw", "control.overrideAx", "control.overrideAy", "control.overrideAz"};
    if (node_.input_layout().size() != 9) {
      throw std::invalid_argument("Navigation control requires the prepared-control input layout");
    }
    for (std::size_t axis = 0; axis < 6; ++axis) {
      if (control_axes_[axis] && state_.values[6 + axis] == missing) {throw std::invalid_argument("Enabled navigation control axis has no measured velocity");}
    }
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto field = rpp_interfaces::find_field(node_.input_layout(), resource_ + "." + names[index]);
      if (!field || node_.input_layout().ranges()[*field].size != 1) {throw std::invalid_argument("Missing prepared navigation input");}
      const auto& description = node_.input_layout().fields()[*field];
      const auto quantity = index < 3 ? c::QUANTITY_LINEAR_ACCELERATION : index < 6 ? c::QUANTITY_ANGULAR_ACCELERATION : c::QUANTITY_NORMALIZED;
      if (description.kind != c::COORDINATE_EUCLIDEAN || description.quantity != quantity ||
        description.resource != resource_ || description.frameId != body_ || description.childFrameId != body_) {
        throw std::invalid_argument("Prepared navigation input has incompatible units or frames");
      }
      control_route_[index] = node_.input_layout().ranges()[*field].offset;
    }
  }
  if (dynamic_noise_ && (state_.mask & 0x7fff) != 0x7fff) {throw std::invalid_argument("Dynamic navigation noise needs the full navigation coordinate set");}
  if (dynamic_noise_ && !use_control_ && !node_.input_layout().fields().empty()) {
    throw std::invalid_argument("Dynamic navigation noise with external inputs needs an explicit prediction policy");
  }
  constexpr std::uint16_t planar = (1u << 2) | (1u << 3) | (1u << 4) | (1u << 8) | (1u << 9) | (1u << 10) | (1u << 14);
  planar_constraints_ = two_d_ ? planar & observed_coordinates_ : 0;
  if (two_d_ && planar_constraints_ && ports_.find(planar_constraints_) == ports_.end()) {
    throw std::invalid_argument("Planar navigation needs the prepared planar measurement layouts (ekf_2d/ukf_2d examples)");
  }
  odometry_.header.frame_id = world_; odometry_.child_frame_id = body_; acceleration_.header.frame_id = body_;
  if (publish_control_state_) {
    const auto required_velocity = control_state_dimensions_ == 2 ? 0x08c0 : twist_mask;
    if ((state_.mask & required_velocity) != required_velocity) {
      throw std::invalid_argument("Control projection requires the corresponding body velocities");
    }
    if (control_state_dimensions_ == 3 && (state_.mask & pose_mask) != pose_mask) {
      throw std::invalid_argument("Spatial control projection requires a complete spatial pose");
    }
    const auto linear = static_cast<std::uint32_t>(control_state_dimensions_);
    const auto angular = control_state_dimensions_ == 2 ? 1u : 3u;
    const bool planar = control_state_dimensions_ == 2;
    const rpp_interfaces::FieldLayout layout({
      {resource_ + ".position", c::COORDINATE_EUCLIDEAN, c::QUANTITY_LINEAR_POSITION, linear, world_, body_, resource_},
      {resource_ + ".orientation", planar ? c::COORDINATE_CONTINUOUS_ANGLE : c::COORDINATE_QUATERNION,
        planar ? c::QUANTITY_ANGULAR_POSITION : c::QUANTITY_ORIENTATION, planar ? 1u : 4u, world_, body_, resource_},
      {resource_ + ".linearVelocity", c::COORDINATE_EUCLIDEAN, c::QUANTITY_LINEAR_VELOCITY, linear, body_, body_, resource_},
      {resource_ + ".angularVelocity", c::COORDINATE_EUCLIDEAN, c::QUANTITY_ANGULAR_VELOCITY, angular, body_, body_, resource_}});
    if (control_state_layout_ && (control_state_layout_->size() != layout.size() ||
      !rpp_interfaces::validate_required_fields(layout, *control_state_layout_))) {
      throw std::invalid_argument("Cannot redefine the published navigation control interface");
    }
    control_state_layout_ = layout;
    rpp_ros_interfaces::msg::NumericInterface description; description.name = resource_; description.layout = h::describe(layout);
    control_state_.interface_name = resource_; control_state_.values.resize(layout.size());
    control_layout_publisher_->publish(description);
  }
  reset();
}

void NavigationInterface::reset()
{
  pending_measurements_.clear();
  controls_.reset(); worker_control_ = {}; published_ = false; last_published_ = 0; previous_gyro_time_.reset();
  for (auto& source : pose_history_) {source.anchored = source.previous = false;}
}

void NavigationInterface::queue_measurement(SensorMessage message)
{
  if (!node_.configured_ || !node_.enabled_) {return;}
  const auto time = std::visit([](const auto& value) {return h::timestamp(value->header.stamp);}, message);
  if (!time) {node_.status(c::CODE_INVALID_INPUT, "Invalid navigation measurement timestamp"); return;}
  if (pending_measurements_.size() == node_.queue_capacity_) {
    node_.status(c::CODE_BUDGET_EXCEEDED, "Navigation measurement queue is full"); return;
  }
  // Keep message ownership, without copying payloads. Equal stamps retain arrival order.
  const auto position = std::upper_bound(pending_measurements_.begin(), pending_measurements_.end(), *time,
    [](auto stamp, const auto& pending) {return stamp < pending.time;});
  pending_measurements_.insert(position, PendingMeasurement{*time, std::move(message)});
}

void NavigationInterface::process_measurements(std::uint64_t time)
{
  if (!node_.configured_ || !node_.enabled_) {pending_measurements_.clear(); return;}
  const auto ready_end = std::upper_bound(pending_measurements_.begin(), pending_measurements_.end(), time,
    [](auto stamp, const auto& pending) {return stamp < pending.time;});
  for (auto pending = pending_measurements_.begin(); pending != ready_end; ++pending) {
    std::visit([this](const auto& message) {
      using Message = std::decay_t<decltype(*message)>;
      if constexpr (std::is_same_v<Message, nav_msgs::msg::Odometry>) {odometry(*message);}
      else if constexpr (std::is_same_v<Message, geometry_msgs::msg::PoseWithCovarianceStamped>) {pose(*message);}
      else if constexpr (std::is_same_v<Message, geometry_msgs::msg::TwistWithCovarianceStamped>) {twist(*message, twist_selection_);}
      else {imu(*message);}
    }, pending->message);
  }
  // Future messages stay queued: /clock and sensor delivery are independent.
  pending_measurements_.erase(pending_measurements_.begin(), ready_end);
}

std::optional<tf2::Transform> NavigationInterface::transform(const std::string& target,
  const std::string& source, const builtin_interfaces::msg::Time& stamp)
{
  if (source.empty() || !h::timestamp(stamp)) {return {};}
  if (target == source) {return tf2::Transform::getIdentity();}
  try {
    tf2::Transform result;
    tf2::fromMsg(tf_buffer_->lookupTransform(target, source, rclcpp::Time(stamp, RCL_ROS_TIME)).transform, result);
    return result;
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "Navigation transform unavailable: %s", error.what());
    return {};
  }
}

void NavigationInterface::publish(const rpp_ros_interfaces::msg::EstimatorEstimate& estimate)
{
  const auto stamp = h::timestamp(estimate.stamp); if (!stamp || (published_ && !permit_corrected_ && *stamp <= last_published_)) {return;}
  const auto n = node_.state_layout().tangent_size();
  if (estimate.state.size() != node_.state_layout().size() || estimate.covariance.size() != n * n) {return;}
  auto& pose = odometry_.pose; auto& twist = odometry_.twist;
  pose.pose.position.x = estimate.state[state_.values[0]]; pose.pose.position.y = estimate.state[state_.values[1]];
  pose.pose.position.z = state_.values[2] == missing ? 0.0 : estimate.state[state_.values[2]];
  if (state_.quaternion) {
    const auto offset = *state_.quaternion;
    pose.pose.orientation.x = estimate.state[offset]; pose.pose.orientation.y = estimate.state[offset + 1];
    pose.pose.orientation.z = estimate.state[offset + 2]; pose.pose.orientation.w = estimate.state[offset + 3];
  } else {
    tf2::Quaternion quaternion;
    quaternion.setRPY(state_.values[3] == missing ? 0.0 : estimate.state[state_.values[3]],
      state_.values[4] == missing ? 0.0 : estimate.state[state_.values[4]], estimate.state[state_.values[5]]);
    pose.pose.orientation.x = quaternion.x(); pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z(); pose.pose.orientation.w = quaternion.w();
  }
  std::array<double*, 6> velocity{&twist.twist.linear.x, &twist.twist.linear.y, &twist.twist.linear.z,
    &twist.twist.angular.x, &twist.twist.angular.y, &twist.twist.angular.z};
  for (std::size_t axis = 0; axis < 6; ++axis) {*velocity[axis] = state_.values[6 + axis] == missing ? 0.0 : estimate.state[state_.values[6 + axis]];}
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      const auto pose_row = state_.tangent[row], pose_column = state_.tangent[column];
      pose.covariance[row * 6 + column] = pose_row == missing || pose_column == missing ?
        (row == column ? (two_d_ ? 1e-6 : 1e6) : 0.0) : estimate.covariance[pose_row * n + pose_column];
      const auto r = state_.tangent[6 + row], k = state_.tangent[6 + column];
      twist.covariance[row * 6 + column] = r == missing || k == missing ? (row == column ? 1e6 : 0.0) : estimate.covariance[r * n + k];
    }
  }
  odometry_.header.stamp = estimate.stamp; odometry_publisher_->publish(odometry_);
  if (publish_control_state_) {
    control_state_.stamp = estimate.stamp;
    if (control_state_dimensions_ == 2) {
      double roll, pitch, yaw;
      const auto& q = pose.pose.orientation;
      tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
      const std::array<double, 6> values{pose.pose.position.x, pose.pose.position.y, yaw,
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z};
      std::copy(values.begin(), values.end(), control_state_.values.begin());
    } else {
      const auto& q = pose.pose.orientation;
      const std::array<double, 13> values{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        q.x, q.y, q.z, q.w, twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z};
      std::copy(values.begin(), values.end(), control_state_.values.begin());
    }
    control_state_publisher_->publish(control_state_);
  }
  if (transforms_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry_.header; transform.child_frame_id = body_;
    transform.transform.translation.x = pose.pose.position.x; transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z; transform.transform.rotation = pose.pose.orientation;
    transforms_->sendTransform(transform);
  }
  if (acceleration_publisher_ && (state_.mask & acceleration_mask) == acceleration_mask) {
    acceleration_.header.stamp = estimate.stamp;
    acceleration_.accel.accel.linear.x = estimate.state[state_.values[12]];
    acceleration_.accel.accel.linear.y = estimate.state[state_.values[13]];
    acceleration_.accel.accel.linear.z = estimate.state[state_.values[14]];
    acceleration_.accel.covariance.fill(0.0);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        acceleration_.accel.covariance[row * 6 + column] = estimate.covariance[state_.tangent[12 + row] * n + state_.tangent[12 + column]];
      }
    }
    for (std::size_t axis = 3; axis < 6; ++axis) {acceleration_.accel.covariance[axis * 6 + axis] = 1e6;}
    acceleration_publisher_->publish(acceleration_);
  }
  published_ = true; last_published_ = *stamp;
}

void NavigationInterface::submit(std::uint16_t requested, const std::array<double, 15>& values,
  const Covariance& covariance, const builtin_interfaces::msg::Time& stamp, const std::array<double, 4>& quaternion)
{
  const auto time = h::timestamp(stamp); const auto current = node_.now().nanoseconds();
  if (!time || current < 0 || *time > static_cast<std::uint64_t>(current)) {return;}
  requested &= observed_coordinates_;
  for (std::size_t role = 0; role < values.size(); ++role) {
    if (!std::isfinite(values[role]) || !std::isfinite(covariance(role, role))) {requested &= ~(1u << role);}
  }
  const auto forced = planar_constraints_;
  requested |= forced;
  if (!requested) {return;}
  const auto found = ports_.find(requested);
  if (found == ports_.end()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "No configured navigation measurement port for mask %u", static_cast<unsigned>(requested)); return;
  }
  auto& port = found->second;
  for (std::size_t role = 0; role < values.size(); ++role) {
    if (port.binding.values[role] != missing) {port.values[port.binding.values[role]] = two_d_ && (forced & (1u << role)) ? 0.0 : values[role];}
  }
  if (port.binding.quaternion) {
    auto output = quaternion;
    if (two_d_) {
      tf2::Quaternion planar; planar.setRPY(0.0, 0.0, values[5]);
      output = {planar.x(), planar.y(), planar.z(), planar.w()};
    }
    for (std::size_t axis = 0; axis < 4; ++axis) {port.values[*port.binding.quaternion + axis] = output[axis];}
  }
  for (std::size_t row = 0; row < port.covariance_roles.size(); ++row) {
    for (std::size_t column = 0; column < port.covariance_roles.size(); ++column) {
      const auto value = covariance(port.covariance_roles[row], port.covariance_roles[column]);
      const bool constrained = two_d_ && ((forced & (1u << port.covariance_roles[row])) || (forced & (1u << port.covariance_roles[column])));
      port.covariance[row * port.covariance_roles.size() + column] = constrained ? (row == column ? 1e-6 : 0.0) :
        row == column ? navigation_helpers::normalize_measurement_variance(value) : value;
    }
  }
  const auto code = node_.submit_measurement(port.index, port.values, port.covariance, *time);
  if (!h::success(code)) {node_.status(code, "Navigation measurement was not queued");}
}

void NavigationInterface::pose(const geometry_msgs::msg::PoseWithCovarianceStamped& message, bool from_odometry)
{
  const auto selection = from_odometry ? odometry_selection_ & pose_mask : pose_selection_;
  if (!selection) {return;}
  const auto frame = transform(world_, message.header.frame_id, message.header.stamp); if (!frame) {return;}
  const auto selected = rotate_selection(selection, 0, *frame) | rotate_selection(selection, 3, *frame);
  auto values = absent(); Covariance covariance = Covariance::Zero(); std::array<double, 4> quaternion{};
  const tf2::Vector3 source_position(message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z);
  const auto position = message.header.frame_id == world_ ? source_position : *frame * source_position;
  values[0] = position.x(); values[1] = position.y(); values[2] = position.z();
  if (orientation(message.pose.pose.orientation, values, quaternion)) {
    const auto q = frame->getRotation() * tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    geometry_msgs::msg::Quaternion output; output.x = q.x(); output.y = q.y(); output.z = q.z(); output.w = q.w();
    orientation(output, values, quaternion);
  }
  const auto rotated = message.header.frame_id == world_ ? message.pose.covariance : tf2::transformCovariance(message.pose.covariance, *frame);
  for (std::size_t row = 0; row < 6; ++row) {for (std::size_t column = 0; column < 6; ++column) {covariance(row, column) = rotated[row * 6 + column];}}
  const auto source = from_odometry ? 1u : 0u;
  if (!from_odometry && pose_child_ != body_) {
    const auto child = transform(pose_child_, body_, message.header.stamp);
    if (!child || !std::isfinite(values[3])) {return;}
    tf2::Transform body_pose(tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]), tf2::Vector3(values[0], values[1], values[2]));
    const auto offset = body_pose.getBasis() * child->getOrigin(); body_pose *= *child;
    values[0] = body_pose.getOrigin().x(); values[1] = body_pose.getOrigin().y(); values[2] = body_pose.getOrigin().z();
    const auto q = body_pose.getRotation(); geometry_msgs::msg::Quaternion q_message;
    q_message.x = q.x(); q_message.y = q.y(); q_message.z = q.z(); q_message.w = q.w(); orientation(q_message, values, quaternion);
    Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Identity();
    jacobian.topRightCorner<3, 3>() = -skew(Eigen::Vector3d(offset.x(), offset.y(), offset.z()));
    covariance.topLeftCorner<6, 6>() = (jacobian * covariance.topLeftCorner<6, 6>() * jacobian.transpose()).eval();
  }
  if (relative_pose_[source] || differential_pose_[source]) {
    const auto stamp = h::timestamp(message.header.stamp); if (!stamp) {return;}
    for (std::size_t coordinate = 0; coordinate < 6; ++coordinate) {if (!std::isfinite(values[coordinate])) {return;}}
    auto& history = pose_history_[source];
    tf2::Transform measured(tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]), tf2::Vector3(values[0], values[1], values[2]));
    if (relative_pose_[source]) {
      if (!history.anchored) {history.anchor = measured; history.anchored = true;}
      const auto relative = history.anchor.inverse() * measured;
      values[0] = relative.getOrigin().x(); values[1] = relative.getOrigin().y(); values[2] = relative.getOrigin().z();
      const auto q = relative.getRotation(); geometry_msgs::msg::Quaternion q_message;
      q_message.x = q.x(); q_message.y = q.y(); q_message.z = q.z(); q_message.w = q.w(); orientation(q_message, values, quaternion);
      const auto r = rotation(history.anchor.inverse());
      Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
      jacobian.topLeftCorner<3, 3>() = r; jacobian.bottomRightCorner<3, 3>() = r;
      covariance.topLeftCorner<6, 6>() = (jacobian * covariance.topLeftCorner<6, 6>() * jacobian.transpose()).eval();
    } else {
      if (history.previous && *stamp <= history.time) {return;}
      if (!history.previous) {history.pose = measured; history.covariance = covariance.topLeftCorner<6, 6>(); history.time = *stamp; history.previous = true; return;}
      const double dt = static_cast<double>(*stamp - history.time) * 1e-9;
      const auto inverse_rotation = measured.getBasis().transpose();
      const auto velocity = inverse_rotation * ((measured.getOrigin() - history.pose.getOrigin()) / dt);
      auto delta = history.pose.getRotation().inverse() * measured.getRotation();
      if (delta.w() < 0.0) {delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());}
      const double length = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y() + delta.z() * delta.z());
      const double factor = length < 1e-12 ? 2.0 / dt : 2.0 * std::atan2(length, delta.w()) / (length * dt);
      auto differential = absent(); differential[6] = velocity.x(); differential[7] = velocity.y(); differential[8] = velocity.z();
      differential[9] = factor * delta.x(); differential[10] = factor * delta.y(); differential[11] = factor * delta.z();
      Covariance output = Covariance::Zero();
      Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
      const auto r = rotation(measured.inverse()); jacobian.topLeftCorner<3, 3>() = r; jacobian.bottomRightCorner<3, 3>() = r;
      output.block<6, 6>(6, 6) = jacobian * (covariance.topLeftCorner<6, 6>() + history.covariance) * jacobian.transpose() / (dt * dt);
      history.pose = measured; history.covariance = covariance.topLeftCorner<6, 6>(); history.time = *stamp;
      submit(static_cast<std::uint16_t>(selected << 6), differential, output, message.header.stamp, {});
      return;
    }
  }
  submit(selected, values, covariance, message.header.stamp, quaternion);
}
void NavigationInterface::twist(const geometry_msgs::msg::TwistWithCovarianceStamped& message, std::uint16_t selection)
{
  if (!selection) {return;}
  const auto frame = transform(body_, message.header.frame_id, message.header.stamp); if (!frame) {return;}
  const auto selected = rotate_selection(selection, 6, *frame) | rotate_selection(selection, 9, *frame);
  auto values = absent(); Covariance covariance = Covariance::Zero();
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero(), input_covariance;
  const auto r = rotation(*frame); const auto t = frame->getOrigin();
  jacobian.topLeftCorner<3, 3>() = r; jacobian.bottomRightCorner<3, 3>() = r;
  jacobian.topRightCorner<3, 3>() = skew(Eigen::Vector3d(t.x(), t.y(), t.z())) * r;
  Eigen::Matrix<double, 6, 1> input;
  input << message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z,
    message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z;
  const Eigen::Matrix<double, 6, 1> output = message.header.frame_id == body_ ? input : (jacobian * input).eval();
  for (std::size_t row = 0; row < 6; ++row) {
    values[6 + row] = output(row);
    for (std::size_t column = 0; column < 6; ++column) {input_covariance(row, column) = message.twist.covariance[row * 6 + column];}
  }
  if (message.header.frame_id == body_) {covariance.block<6, 6>(6, 6) = input_covariance;}
  else {covariance.block<6, 6>(6, 6) = jacobian * input_covariance * jacobian.transpose();}
  submit(selected, values, covariance, message.header.stamp, {});
}
void NavigationInterface::odometry(const nav_msgs::msg::Odometry& message)
{
  const auto submit_pose = [&] {
    if (!(odometry_selection_ & pose_mask)) {return;}
    geometry_msgs::msg::PoseWithCovarianceStamped pose_message;
    pose_message.header = message.header; pose_message.pose = message.pose;
    if (message.child_frame_id != body_) {
      auto values = absent(); std::array<double, 4> quaternion{};
      if (!orientation(message.pose.pose.orientation, values, quaternion)) {return;}
      const auto child = transform(message.child_frame_id, body_, message.header.stamp);
      if (!child) {return;}
      tf2::Transform measured; tf2::fromMsg(message.pose.pose, measured);
      const auto offset = measured.getBasis() * child->getOrigin();
      measured *= *child; tf2::toMsg(measured, pose_message.pose.pose);
      Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Identity(), covariance;
      jacobian.topRightCorner<3, 3>() = -skew(Eigen::Vector3d(offset.x(), offset.y(), offset.z()));
      for (std::size_t row = 0; row < 6; ++row) {for (std::size_t column = 0; column < 6; ++column) {covariance(row, column) = message.pose.covariance[row * 6 + column];}}
      const Eigen::Matrix<double, 6, 6> transformed = jacobian * covariance * jacobian.transpose();
      for (std::size_t row = 0; row < 6; ++row) {for (std::size_t column = 0; column < 6; ++column) {pose_message.pose.covariance[row * 6 + column] = transformed(row, column);}}
    }
    pose(pose_message, true);
  };
  submit_pose();  // An unavailable pose transform must not discard valid twist.
  geometry_msgs::msg::TwistWithCovarianceStamped twist_message;
  twist_message.header = message.header; twist_message.header.frame_id = message.child_frame_id; twist_message.twist = message.twist;
  twist(twist_message, odometry_selection_ & twist_mask);
}
void NavigationInterface::imu(const sensor_msgs::msg::Imu& message)
{
  if (!imu_selection_) {return;}
  const auto frame = transform(body_, message.header.frame_id, message.header.stamp); if (!frame) {return;}
  const auto selected = rotate_selection(imu_selection_, 3, *frame) | rotate_selection(imu_selection_, 9, *frame) |
    rotate_selection(imu_selection_, 12, *frame);
  auto values = absent(); Covariance covariance = Covariance::Zero(); std::array<double, 4> quaternion{};
  bool valid_orientation = message.orientation_covariance[0] != -1 && orientation(message.orientation, values, quaternion);
  if (valid_orientation) {
    const auto world = transform(world_, imu_world_, message.header.stamp);
    if (!world) {valid_orientation = false;}
    else {
      const auto q = world->getRotation() * tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]) * frame->getRotation().inverse();
      geometry_msgs::msg::Quaternion output; output.x = q.x(); output.y = q.y(); output.z = q.z(); output.w = q.w();
      valid_orientation = orientation(output, values, quaternion);
    }
  }
  const auto r = rotation(*frame);
  const Eigen::Vector3d source_omega(message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
  const Eigen::Vector3d source_acceleration(message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z);
  const Eigen::Vector3d omega = message.header.frame_id == body_ ? source_omega : (r * source_omega).eval();
  Eigen::Vector3d acceleration = message.header.frame_id == body_ ? source_acceleration : (r * source_acceleration).eval();
  Eigen::Matrix3d angular_covariance, acceleration_covariance, orientation_covariance;
  for (std::size_t row = 0; row < 3; ++row) {for (std::size_t column = 0; column < 3; ++column) {
      angular_covariance(row, column) = message.angular_velocity_covariance[row * 3 + column];
      acceleration_covariance(row, column) = message.linear_acceleration_covariance[row * 3 + column];
      orientation_covariance(row, column) = message.orientation_covariance[row * 3 + column];
    }}
  if (message.header.frame_id != body_) {
    angular_covariance = (r * angular_covariance * r.transpose()).eval();
    acceleration_covariance = (r * acceleration_covariance * r.transpose()).eval();
    orientation_covariance = (r * orientation_covariance * r.transpose()).eval();
  }
  const auto t = frame->getOrigin(); const Eigen::Vector3d offset(t.x(), t.y(), t.z());
  const auto time = h::timestamp(message.header.stamp);
  if (offset.squaredNorm() > 1e-18) {
    if (time && previous_gyro_time_ && *time > *previous_gyro_time_ && omega.allFinite() && message.angular_velocity_covariance[0] != -1) {
      const double dt = static_cast<double>(*time - *previous_gyro_time_) * 1e-9;
      const Eigen::Vector3d previous(previous_gyro_[0], previous_gyro_[1], previous_gyro_[2]);
      const Eigen::Vector3d alpha = (omega - previous) / dt;
      acceleration -= skew(alpha) * offset + skew(omega) * (skew(omega) * offset);
      const Eigen::Matrix3d centripetal = omega.dot(offset) * Eigen::Matrix3d::Identity() + omega * offset.transpose() - 2.0 * offset * omega.transpose();
      const Eigen::Matrix3d current_jacobian = skew(offset) / dt - centripetal;
      const Eigen::Matrix3d previous_jacobian = -skew(offset) / dt;
      acceleration_covariance += current_jacobian * angular_covariance * current_jacobian.transpose() +
        previous_jacobian * previous_gyro_covariance_ * previous_jacobian.transpose();
    } else {
      // Angular acceleration is required to move an off-center accelerometer.
      // Establish its derivative history before fusing that acceleration.
      acceleration.setConstant(std::numeric_limits<double>::quiet_NaN());
    }
  }
  if (time && (!previous_gyro_time_ || *time > *previous_gyro_time_) && omega.allFinite() && message.angular_velocity_covariance[0] != -1) {
    previous_gyro_ = {omega.x(), omega.y(), omega.z()}; previous_gyro_time_ = *time; previous_gyro_covariance_ = angular_covariance;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {values[9 + axis] = omega(axis); values[12 + axis] = acceleration(axis);}
  if (remove_gravity_) {
    if (!valid_orientation) {values[12] = values[13] = values[14] = std::numeric_limits<double>::quiet_NaN();}
    else {
      const auto gravity = tf2::quatRotate(tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).inverse(), tf2::Vector3(0, 0, 9.80665));
      values[12] -= gravity.x(); values[13] -= gravity.y(); values[14] -= gravity.z();
    }
  }
  for (std::size_t row = 0; row < 3; ++row) {for (std::size_t column = 0; column < 3; ++column) {
      covariance(3 + row, 3 + column) = orientation_covariance(row, column);
      covariance(9 + row, 9 + column) = angular_covariance(row, column);
      covariance(12 + row, 12 + column) = acceleration_covariance(row, column);
    }}
  if (valid_orientation && (selected & 0x0038)) {submit(selected & 0x0038, values, covariance, message.header.stamp, quaternion);}
  if (message.angular_velocity_covariance[0] != -1 && (selected & 0x0e00)) {submit(selected & 0x0e00, values, covariance, message.header.stamp, {});}
  if (message.linear_acceleration_covariance[0] != -1 && (selected & acceleration_mask)) {submit(selected & acceleration_mask, values, covariance, message.header.stamp, {});}
}
void NavigationInterface::control(const geometry_msgs::msg::Twist& message)
{
  const auto time = node_.now().nanoseconds(); if (time < 0) {return;}
  Control command; command.time = static_cast<std::uint64_t>(time);
  command.values = {message.linear.x, message.linear.y, message.linear.z, message.angular.x, message.angular.y, message.angular.z};
  for (const auto value : command.values) {if (!std::isfinite(value)) {return;}}
  if (auto* output = controls_.write()) {*output = command; controls_.commit();}
}

void NavigationInterface::set_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& message)
{
  EstimatorScriptNode::ManagementPause pause(node_);
  if (!node_.configured_) {node_.status(c::CODE_NOT_READY); return;}
  const auto frame = transform(world_, message.header.frame_id, message.header.stamp); if (!frame) {return;}
  auto values = absent(); std::array<double, 4> quaternion{};
  if (!orientation(message.pose.pose.orientation, values, quaternion)) {node_.status(c::CODE_INVALID_INPUT, "Set-pose orientation must be a unit quaternion"); return;}
  const auto q = frame->getRotation() * tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  const auto position = *frame * tf2::Vector3(message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z);
  values[0] = position.x(); values[1] = position.y(); values[2] = position.z();
  geometry_msgs::msg::Quaternion q_message; q_message.x = q.x(); q_message.y = q.y(); q_message.z = q.z(); q_message.w = q.w();
  orientation(q_message, values, quaternion);
  rpp_ros_interfaces::srv::ResetEstimate::Request request;
  request.stamp = message.header.stamp;
  const auto stamp = h::timestamp(request.stamp);
  if (stamp && *stamp == 0) {request.stamp = node_.now();}
  request.state.resize(node_.initial_state_.size()); request.covariance.resize(node_.initial_covariance_.size());
  for (std::size_t index = 0; index < request.state.size(); ++index) {request.state[index] = node_.initial_state_[index];}
  for (std::size_t index = 0; index < request.covariance.size(); ++index) {request.covariance[index] = node_.initial_covariance_[index];}
  for (std::size_t role = 0; role < 6; ++role) {
    if (state_.values[role] != missing) {request.state[state_.values[role]] = values[role];}
  }
  if (state_.quaternion) {
    if (two_d_) {
      tf2::Quaternion planar; planar.setRPY(0.0, 0.0, values[5]);
      quaternion = {planar.x(), planar.y(), planar.z(), planar.w()};
    }
    for (std::size_t axis = 0; axis < 4; ++axis) {request.state[*state_.quaternion + axis] = quaternion[axis];}
  }
  const auto covariance = tf2::transformCovariance(message.pose.covariance, *frame);
  const auto n = node_.state_layout().tangent_size();
  for (std::size_t role = 0; role < 6; ++role) {
    if (state_.tangent[role] == missing) {continue;}
    for (std::size_t other = 0; other < n; ++other) {
      request.covariance[state_.tangent[role] * n + other] = 0.0; request.covariance[other * n + state_.tangent[role]] = 0.0;
    }}
  for (std::size_t row = 0; row < 6; ++row) {for (std::size_t column = 0; column < 6; ++column) {
      if (state_.tangent[row] == missing || state_.tangent[column] == missing) {continue;}
      const auto value = covariance[row * 6 + column];
      request.covariance[state_.tangent[row] * n + state_.tangent[column]] = row == column ?
        navigation_helpers::normalize_measurement_variance(value) : value;
    }}
  if (two_d_) {
    constexpr std::array<std::size_t, 7> constrained{2, 3, 4, 8, 9, 10, 14};
    for (const auto role : constrained) {
      if (state_.values[role] != missing) {request.state[state_.values[role]] = 0.0;}
      if (state_.tangent[role] == missing) {continue;}
      for (std::size_t other = 0; other < n; ++other) {
        request.covariance[state_.tangent[role] * n + other] = 0.0; request.covariance[other * n + state_.tangent[role]] = 0.0;
      }
      request.covariance[state_.tangent[role] * n + state_.tangent[role]] = 1e-6;
    }
  }
  request.activate = node_.script_->is_active();
  rpp_ros_interfaces::srv::ResetEstimate::Response response;
  node_.reset(request, response, true);
}

std::uint8_t NavigationInterface::prediction_policy(std::uint64_t end, std::uint64_t dt,
  const rpp_ros_interfaces::msg::EstimatorEstimate& state, rpp::List<double>& input, rpp::List<double>& q)
{
  for (std::size_t count = 0; count < 16; ++count) {
    const auto* command = controls_.front(); if (!command) {break;}
    worker_control_ = *command; controls_.pop();
  }
  const double seconds = static_cast<double>(dt) * 1e-9;
  std::array<double, 6> effective{};
  if (use_control_) {
    const bool timed_out = worker_control_.time > end || end - worker_control_.time >= control_timeout_;
    for (std::size_t index = 0; index < 6; ++index) {
      if (control_axes_[index]) {
        const auto velocity = state.state[state_.values[6 + index]];
        const auto target = timed_out ? 0.0 : worker_control_.values[index];
        const auto error = target - velocity;
        const bool same_sign = std::abs(error) <= std::abs(target) + 0.01;
        const bool decelerating = std::abs(same_sign ? target : 0.0) < std::abs(velocity);
        const auto gain = decelerating ? deceleration_gains_[index] : acceleration_gains_[index];
        const auto limit = decelerating ? deceleration_limits_[index] : acceleration_limits_[index];
        effective[index] = std::clamp(gain * error, -limit, limit);
      }
      input[control_route_[index]] = effective[index];
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {input[control_route_[6 + axis]] = control_axes_[axis] ? 1.0 : 0.0;}
  }
  if (dynamic_noise_) {
    double speed_squared = 0.0;
    for (std::size_t axis = 0; axis < 6; ++axis) {
      const double acceleration = axis < 3 ? (use_control_ && control_axes_[axis] ? effective[axis] :
        state.state[state_.values[12 + axis]]) : effective[axis];
      const double velocity = state.state[state_.values[6 + axis]] + seconds * acceleration;
      speed_squared += velocity * velocity;
    }
    const auto n = node_.state_layout().tangent_size(); const auto speed = std::sqrt(speed_squared);
    // Apply S Q S^T, including pose/velocity cross terms, to preserve PSD.
    for (std::size_t pose = 0; pose < 6; ++pose) {
      for (std::size_t other = 0; other < n; ++other) {
        const auto row = state_.tangent[pose] * n + other; q[row] = static_cast<double>(q[row]) * speed;
      }
    }
    for (std::size_t pose = 0; pose < 6; ++pose) {
      for (std::size_t other = 0; other < n; ++other) {
        const auto column = other * n + state_.tangent[pose]; q[column] = static_cast<double>(q[column]) * speed;
      }
    }
  }
  return c::CODE_OK;
}
}  // namespace rpp_localization
