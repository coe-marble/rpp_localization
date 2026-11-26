#pragma once

#include <rpp_interfaces/field_routing.hpp>
#include <rpp_interfaces/signal_router.hpp>
#include <rpp_localization/estimation/rpp_support.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rpp_ros_interfaces/msg/numeric_layout.hpp>

#include <cmath>
#include <limits>
#include <optional>

namespace rpp_localization::ros_helpers
{
inline bool success(std::uint8_t code)
{return code == rpp_interfaces::contract::CODE_OK || code == rpp_interfaces::contract::CODE_LIMITED;}

inline std::optional<std::uint64_t> timestamp(const builtin_interfaces::msg::Time& value)
{
  if (value.sec < 0 || value.nanosec >= 1'000'000'000) {return {};}
  return static_cast<std::uint64_t>(value.sec) * 1'000'000'000 + value.nanosec;
}
inline std::optional<builtin_interfaces::msg::Time> timestamp(std::uint64_t value)
{
  if (value / 1'000'000'000 > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {return {};}
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(value / 1'000'000'000);
  result.nanosec = static_cast<std::uint32_t>(value % 1'000'000'000);
  return result;
}
inline std::size_t covariance_size(const rpp_interfaces::FieldLayout& layout)
{
  const auto n = layout.tangent_size();
  if (layout.size() == 0 || n == 0) {throw std::invalid_argument("Estimator state and observation layouts must be nonempty");}
  if (n && n > rpp_interfaces::SignalRouter::max_values / n) {
    throw std::invalid_argument("Covariance exceeds prepared buffer capacity");
  }
  return n * n;
}
inline rpp_ros_interfaces::msg::NumericLayout describe(const rpp_interfaces::FieldLayout& layout)
{
  rpp_ros_interfaces::msg::NumericLayout message;
  message.value_count = static_cast<std::uint32_t>(layout.size());
  for (std::size_t index = 0; index < layout.fields().size(); ++index) {
    const auto& field = layout.fields()[index];
    const auto range = layout.ranges()[index];
    rpp_ros_interfaces::msg::NumericField output;
    output.name = field.name; output.coordinate_kind = field.kind; output.quantity = field.quantity;
    output.offset = static_cast<std::uint32_t>(range.offset); output.width = static_cast<std::uint32_t>(range.size);
    output.frame_id = field.frameId; output.child_frame_id = field.childFrameId; output.resource = field.resource;
    message.fields.push_back(std::move(output));
  }
  return message;
}
template<class Values>
inline bool copy(const Values& source, rpp::List<double>& output)
{
  if (source.size() != output.size()) {return false;}
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (!std::isfinite(static_cast<double>(source[index]))) {return false;}
  }
  for (std::size_t index = 0; index < source.size(); ++index) {output[index] = source[index];}
  return true;
}
inline void covariance_parameter(const std::vector<double>& values, std::size_t n,
  rpp::List<double>& output, double default_diagonal)
{
  output.resize(n * n);
  if (!values.empty() && values.size() != n && values.size() != n * n) {
    throw std::invalid_argument("Covariance parameter must be a diagonal or a row-major square matrix");
  }
  for (std::size_t row = 0; row < n; ++row) {
    for (std::size_t column = 0; column < n; ++column) {
      const double value = values.empty() ? (row == column ? default_diagonal : 0.0) :
        values.size() == n ? (row == column ? values[row] : 0.0) : values[row * n + column];
      if (!std::isfinite(value) || (row == column && value < 0.0)) {
        throw std::invalid_argument("Covariance parameter must be finite with a nonnegative diagonal");
      }
      output[row * n + column] = value;
    }
  }
  // Startup only: reject indefinite or asymmetric noise before execution.
  estimation::Matrix matrix(n, n);
  estimation::shared::read_matrix(output, matrix);
  Eigen::LDLT<estimation::Matrix> factor;
  if (!estimation::shared::valid_covariance(matrix, factor)) {
    throw std::invalid_argument("Covariance parameter must be symmetric and positive semidefinite");
  }
}
template<class Value>
Value parameter(rclcpp::Node& node, const std::string& name, const Value& value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<Value>(name, value, descriptor);
}
}  // namespace rpp_localization::ros_helpers
