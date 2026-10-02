#include "rpp_localization/models/navigation_model_base.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rpp_localization
{

NavigationModelBase::NavigationModelBase(const int state_dim)
: ModelBase(state_dim),
  initialized_(false),
  control_timeout_(0),
  use_dynamic_process_noise_covariance_(false),
  acceleration_gains_(TWIST_SIZE, 0.0),
  acceleration_limits_(TWIST_SIZE, 0.0),
  deceleration_gains_(TWIST_SIZE, 0.0),
  deceleration_limits_(TWIST_SIZE, 0.0),
  control_update_vector_(TWIST_SIZE, false),
  control_acceleration_(TWIST_SIZE),
  covariance_epsilon_(STATE_SIZE, STATE_SIZE),
  dynamic_process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  identity_(STATE_SIZE, STATE_SIZE),
  process_noise_covariance_(STATE_SIZE, STATE_SIZE)
{
  reset();
}

NavigationModelBase::~NavigationModelBase() = default;

void NavigationModelBase::reset()
{
  initialized_ = false;
  control_acceleration_.setZero();

  identity_.setIdentity();
  covariance_epsilon_.setIdentity();
  covariance_epsilon_ *= 0.001;

  process_noise_covariance_.setZero();
  process_noise_covariance_(StateMemberX, StateMemberX) = 0.05;
  process_noise_covariance_(StateMemberY, StateMemberY) = 0.05;
  process_noise_covariance_(StateMemberZ, StateMemberZ) = 0.06;
  process_noise_covariance_(StateMemberRoll, StateMemberRoll) = 0.03;
  process_noise_covariance_(StateMemberPitch, StateMemberPitch) = 0.03;
  process_noise_covariance_(StateMemberYaw, StateMemberYaw) = 0.06;
  process_noise_covariance_(StateMemberVx, StateMemberVx) = 0.025;
  process_noise_covariance_(StateMemberVy, StateMemberVy) = 0.025;
  process_noise_covariance_(StateMemberVz, StateMemberVz) = 0.04;
  process_noise_covariance_(StateMemberVroll, StateMemberVroll) = 0.01;
  process_noise_covariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
  process_noise_covariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
  process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.01;
  process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.01;
  process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.015;

  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

bool NavigationModelBase::get_initialized_status() const
{
  return initialized_;
}

const Eigen::MatrixXd& NavigationModelBase::get_process_noise_covariance() const
{
  return process_noise_covariance_;
}

const std::vector<bool>& NavigationModelBase::get_control_update_vector() const
{
  return control_update_vector_;
}

void NavigationModelBase::compute_dynamic_process_noise_covariance(
  const Eigen::VectorXd& state,
  Eigen::MatrixXd& covariance)
{
  Eigen::MatrixXd velocity_matrix(TWIST_SIZE, TWIST_SIZE);
  velocity_matrix.setIdentity();
  velocity_matrix.diagonal() *= state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

  covariance.block<TWIST_SIZE, TWIST_SIZE>(POSITION_OFFSET, POSITION_OFFSET) =
    velocity_matrix *
    process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(POSITION_OFFSET, POSITION_OFFSET) *
    velocity_matrix.transpose();
}

void NavigationModelBase::set_control_params(
  const std::vector<bool>& update_vector,
  const DurationNs control_timeout,
  const std::vector<double>& acceleration_limits,
  const std::vector<double>& acceleration_gains,
  const std::vector<double>& deceleration_limits,
  const std::vector<double>& deceleration_gains)
{
  if (control_timeout < 0 ||
    update_vector.size() != TWIST_SIZE ||
    acceleration_limits.size() != TWIST_SIZE ||
    acceleration_gains.size() != TWIST_SIZE ||
    deceleration_limits.size() != TWIST_SIZE ||
    deceleration_gains.size() != TWIST_SIZE)
  {
    throw std::invalid_argument("control configuration must contain six values and a non-negative timeout");
  }

  _use_control = true;
  control_update_vector_ = update_vector;
  control_timeout_ = control_timeout;
  acceleration_limits_ = acceleration_limits;
  acceleration_gains_ = acceleration_gains;
  deceleration_limits_ = deceleration_limits;
  deceleration_gains_ = deceleration_gains;
}

void NavigationModelBase::set_dynamic_process_noise_covariance(const bool enabled)
{
  use_dynamic_process_noise_covariance_ = enabled;
}

void NavigationModelBase::set_debug(const bool debug, std::ostream* output_stream)
{
  ModelBase::set_debug(debug, output_stream);
}

void NavigationModelBase::set_process_noise_covariance(
  const Eigen::MatrixXd& process_noise_covariance)
{
  if (process_noise_covariance.rows() != STATE_SIZE ||
    process_noise_covariance.cols() != STATE_SIZE ||
    !process_noise_covariance.allFinite())
  {
    throw std::invalid_argument("process noise covariance must be finite 15x15");
  }

  process_noise_covariance_ = process_noise_covariance;
  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

void NavigationModelBase::prepare_control(const TimestampNs reference_time)
{
  control_acceleration_.setZero();
  if (!_use_control)
  {
    return;
  }

  const bool timed_out = _control.stamp <= reference_time &&
    reference_time - _control.stamp >= control_timeout_;
  if (timed_out)
  {
    MB_DEBUG(
      "Control timed out. Reference time was " << reference_time <<
      ", latest control time was " << _control.stamp <<
      ", control timeout was " << control_timeout_ << "\n");
  }

  for (std::size_t control_index = 0; control_index < TWIST_SIZE; ++control_index)
  {
    if (control_update_vector_[control_index])
    {
      control_acceleration_(static_cast<Eigen::Index>(control_index)) =
        compute_control_acceleration(
        _state(static_cast<Eigen::Index>(control_index + POSITION_V_OFFSET)),
        timed_out ? 0.0 : _control.control(static_cast<Eigen::Index>(control_index)),
        acceleration_limits_[control_index],
        acceleration_gains_[control_index],
        deceleration_limits_[control_index],
        deceleration_gains_[control_index]);
    }
  }
}

double NavigationModelBase::compute_control_acceleration(
  const double state,
  const double control,
  const double acceleration_limit,
  const double acceleration_gain,
  const double deceleration_limit,
  const double deceleration_gain)
{
  const double error = control - state;
  const bool same_sign = std::fabs(error) <= std::fabs(control) + 0.01;
  const double set_point = same_sign ? control : 0.0;
  const bool decelerating = std::fabs(set_point) < std::fabs(state);
  const double limit = decelerating ? deceleration_limit : acceleration_limit;
  const double gain = decelerating ? deceleration_gain : acceleration_gain;
  return std::clamp(gain * error, -limit, limit);
}

}  // namespace rpp_localization
