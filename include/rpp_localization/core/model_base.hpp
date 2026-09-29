#pragma once

#include "rpp_localization/core/measurement.hpp"

#include <Eigen/Dense>

#include <memory>
#include <ostream>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace rpp_localization
{

// Compatibility base for legacy navigation models. It intentionally keeps the
// ROS lifecycle and time interfaces during the dependency-extraction phase.
class ModelBase
{
public:
  explicit ModelBase(const int state_dim)
  : _use_control(false),
    _compute_jacobian(true),
    _compute_covariance(true),
    _debug(false),
    _debug_stream(nullptr)
  {
    if (state_dim <= 0)
    {
      throw std::invalid_argument("state_space_dim must be >0");
    }

    _state_dim = state_dim;
    _state.resize(state_dim);
    _state.setZero();
    _state_covariance.resize(state_dim, state_dim);
    _state_covariance.setZero();
    _cov_dontcare.resize(state_dim, state_dim);
    _cov_dontcare.setZero();
  }

  virtual ~ModelBase() = default;

  virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;

  void step(const rclcpp::Time& reference_time, const double delta_sec)
  {
    step(_state, _state_covariance, reference_time, delta_sec);
  }

  void step(
    Eigen::VectorXd& state,
    const rclcpp::Time& reference_time,
    const double delta_sec)
  {
    step(state, _cov_dontcare, reference_time, delta_sec);
  }

  virtual void step(
    Eigen::VectorXd& state,
    Eigen::MatrixXd& state_covariance,
    const rclcpp::Time& reference_time,
    double delta_sec) = 0;

  [[nodiscard]] bool use_control() const
  {
    return _use_control;
  }

  [[nodiscard]] const ControlCommand& get_control() const
  {
    return _control;
  }

  void set_control(const ControlCommand& control)
  {
    _control = control;
  }

  [[nodiscard]] const Eigen::VectorXd& get_state() const
  {
    return _state;
  }

  [[nodiscard]] Eigen::VectorXd& get_state_unsafe()
  {
    return _state;
  }

  void set_state(const Eigen::VectorXd& state)
  {
    _state = state;
  }

  void set_state(Eigen::VectorXd&& state)
  {
    _state = std::move(state);
  }

  [[nodiscard]] bool computes_covariance() const
  {
    return _compute_covariance;
  }

  void set_computes_covariance(const bool compute)
  {
    _compute_covariance = compute;
  }

  void set_state_covariance(const Eigen::MatrixXd& covariance)
  {
    _state_covariance = covariance;
  }

  [[nodiscard]] const Eigen::MatrixXd& get_state_covariance() const
  {
    return _state_covariance;
  }

  [[nodiscard]] Eigen::MatrixXd& get_state_covariance_unsafe()
  {
    return _state_covariance;
  }

  [[nodiscard]] bool computes_jacobian() const
  {
    return _compute_jacobian;
  }

  void set_computes_jacobian(const bool compute)
  {
    _compute_jacobian = compute;
  }

  [[nodiscard]] const Eigen::MatrixXd& get_state_jacobian() const
  {
    throw std::logic_error("Method not implemented");
  }

  [[nodiscard]] bool get_debug() const
  {
    return _debug;
  }

  void set_debug(const bool debug, std::ostream* output_stream)
  {
    if (debug && output_stream != nullptr)
    {
      _debug_stream = output_stream;
      _debug = true;
      return;
    }

    _debug = false;
  }

protected:
  int _state_dim;

  bool _debug;
  std::ostream* _debug_stream;
  bool _use_control;
  bool _compute_jacobian;
  bool _compute_covariance;
  ControlCommand _control;
  Eigen::VectorXd _state;
  Eigen::MatrixXd _state_covariance;
  Eigen::MatrixXd _cov_dontcare;
};

}  // namespace rpp_localization
