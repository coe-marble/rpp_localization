#pragma once

#include "rpp_localization/core/filter_common.hpp"

#include <ostream>
#include <stdexcept>
#include <utility>

namespace rpp_localization
{

// Core contract for injected navigation prediction models. ROS lifecycle and time adaptation belong
// to the model and adapter layers, not this core interface.
class ModelBase
{
public:
  explicit ModelBase(const int state_dim)
  : _state_dim(state_dim),
    _debug(false),
    _debug_stream(nullptr),
    _use_control(false),
    _compute_jacobian(true),
    _compute_covariance(true)
  {
    if (state_dim <= 0)
    {
      throw std::invalid_argument("state_space_dim must be >0");
    }

    _state.resize(state_dim);
    _state.setZero();
    _state_covariance.resize(state_dim, state_dim);
    _state_covariance.setZero();
    _cov_dontcare.resize(state_dim, state_dim);
    _cov_dontcare.setZero();
  }

  virtual ~ModelBase() = default;

  virtual void predict(
    StateVector& state,
    CovarianceMatrix& state_covariance,
    TimestampNs reference_time,
    DurationNs delta) = 0;

  void predict(const TimestampNs reference_time, const DurationNs delta)
  {
    predict(_state, _state_covariance, reference_time, delta);
  }

  void predict(
    StateVector& state,
    const TimestampNs reference_time,
    const DurationNs delta)
  {
    predict(state, _cov_dontcare, reference_time, delta);
  }

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
