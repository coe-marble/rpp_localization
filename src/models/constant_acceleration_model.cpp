
#include "rpp_localization/models/constant_acceleration_model.hpp"


namespace rpp_localization
{
ConstantAccelerationModel::ConstantAccelerationModel(int state_dim)
: NavigationModelBase(state_dim),
  _transfer_function(STATE_SIZE, STATE_SIZE),
  _transfer_function_jacobian(STATE_SIZE, STATE_SIZE)
{
  _use_control = false;
  _compute_jacobian = true;
  _compute_covariance = true;
}

ConstantAccelerationModel::~ConstantAccelerationModel() {}

void ConstantAccelerationModel::initialize_runtime()
{
  _transfer_function.setIdentity();
  _transfer_function_jacobian.setZero();
  initialized_ = true;
}

void ConstantAccelerationModel::predict(
  StateVector& state,
  CovarianceMatrix& state_covariance,
  const TimestampNs reference_time,
  const DurationNs delta)
{
  const double delta_seconds = nanoseconds_to_seconds(delta);

  double roll = state(StateMemberRoll);
  double pitch = state(StateMemberPitch);
  double yaw = state(StateMemberYaw);
  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);
  double cpi = 1.0 / cp;
  double tp = sp * cpi;

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);

  prepare_control(reference_time);

  // Prepare the transfer function
  _transfer_function(StateMemberX, StateMemberVx) = cy * cp * delta_seconds;
  _transfer_function(StateMemberX, StateMemberVy) =
    (cy * sp * sr - sy * cr) * delta_seconds;
  _transfer_function(StateMemberX, StateMemberVz) =
    (cy * sp * cr + sy * sr) * delta_seconds;
  _transfer_function(StateMemberX, StateMemberAx) =
    0.5 * _transfer_function(StateMemberX, StateMemberVx) * delta_seconds;
  _transfer_function(StateMemberX, StateMemberAy) =
    0.5 * _transfer_function(StateMemberX, StateMemberVy) * delta_seconds;
  _transfer_function(StateMemberX, StateMemberAz) =
    0.5 * _transfer_function(StateMemberX, StateMemberVz) * delta_seconds;
  _transfer_function(StateMemberY, StateMemberVx) = sy * cp * delta_seconds;
  _transfer_function(StateMemberY, StateMemberVy) =
    (sy * sp * sr + cy * cr) * delta_seconds;
  _transfer_function(StateMemberY, StateMemberVz) =
    (sy * sp * cr - cy * sr) * delta_seconds;
  _transfer_function(StateMemberY, StateMemberAx) =
    0.5 * _transfer_function(StateMemberY, StateMemberVx) * delta_seconds;
  _transfer_function(StateMemberY, StateMemberAy) =
    0.5 * _transfer_function(StateMemberY, StateMemberVy) * delta_seconds;
  _transfer_function(StateMemberY, StateMemberAz) =
    0.5 * _transfer_function(StateMemberY, StateMemberVz) * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberVx) = -sp * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberVy) = cp * sr * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberVz) = cp * cr * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberAx) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVx) * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberAy) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVy) * delta_seconds;
  _transfer_function(StateMemberZ, StateMemberAz) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVz) * delta_seconds;
  _transfer_function(StateMemberRoll, StateMemberVroll) = delta_seconds;
  _transfer_function(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_seconds;
  _transfer_function(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_seconds;
  _transfer_function(StateMemberPitch, StateMemberVpitch) = cr * delta_seconds;
  _transfer_function(StateMemberPitch, StateMemberVyaw) = -sr * delta_seconds;
  _transfer_function(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_seconds;
  _transfer_function(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_seconds;
  _transfer_function(StateMemberVx, StateMemberAx) = delta_seconds;
  _transfer_function(StateMemberVy, StateMemberAy) = delta_seconds;
  _transfer_function(StateMemberVz, StateMemberAz) = delta_seconds;


  MB_DEBUG(
    "---------------------- Ekf::predict ----------------------\n" <<
      "delta is " << delta_seconds << "\n" <<
      "state is " << state << "\n");

  // (1) Apply control terms, which are actually accelerations
  state(StateMemberVroll) +=
    control_acceleration_(ControlMemberVroll) * delta_seconds;
  state(StateMemberVpitch) +=
    control_acceleration_(ControlMemberVpitch) * delta_seconds;
  state(StateMemberVyaw) +=
    control_acceleration_(ControlMemberVyaw) * delta_seconds;

  state(StateMemberAx) = (control_update_vector_[ControlMemberVx] ?
    control_acceleration_(ControlMemberVx) :
    state(StateMemberAx));
  state(StateMemberAy) = (control_update_vector_[ControlMemberVy] ?
    control_acceleration_(ControlMemberVy) :
    state(StateMemberAy));
  state(StateMemberAz) = (control_update_vector_[ControlMemberVz] ?
    control_acceleration_(ControlMemberVz) :
    state(StateMemberAz));


  // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
  state = _transfer_function * state;

  // Handle wrapping
  filter_utilities::wrap_state_angles(state);

  MB_DEBUG(
    "Transfer function is:\n" <<
      _transfer_function << "\nCurrent state is:\n" <<
      state << "\nCurrent estimate error covariance is:\n" <<
      state_covariance << "\n");

  // Prepare the transfer function Jacobian. This function is analytically
  // derived from the transfer function.

  if (_compute_jacobian || _compute_covariance)
  {
    double x_vel = state(StateMemberVx);
    double y_vel = state(StateMemberVy);
    double z_vel = state(StateMemberVz);
    double pitch_vel = state(StateMemberVpitch);
    double yaw_vel = state(StateMemberVyaw);
    double x_acc = state(StateMemberAx);
    double y_acc = state(StateMemberAy);
    double z_acc = state(StateMemberAz);


    double x_coeff = 0.0;
    double y_coeff = 0.0;
    double z_coeff = 0.0;
    double one_half_at_squared = 0.5 * delta_seconds * delta_seconds;

    y_coeff = cy * sp * cr + sy * sr;
    z_coeff = -cy * sp * sr + sy * cr;
    double d_fx_d_roll = (y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double d_f_roll_d_roll = 1.0 + (cr * tp * pitch_vel - sr * tp * yaw_vel) * delta_seconds;

    x_coeff = -cy * sp;
    y_coeff = cy * cp * sr;
    z_coeff = cy * cp * cr;
    double d_fx_d_pitch =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;
    double d_f_roll_d_pitch =
      (cpi * cpi * sr * pitch_vel + cpi * cpi * cr * yaw_vel) * delta_seconds;

    x_coeff = -sy * cp;
    y_coeff = -sy * sp * sr - cy * cr;
    z_coeff = -sy * sp * cr + cy * sr;
    double d_fx_d_yaw =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    y_coeff = sy * sp * cr - cy * sr;
    z_coeff = -sy * sp * sr - cy * cr;
    double d_fy_d_roll = (y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double d_f_pitch_d_roll = (-sr * pitch_vel - cr * yaw_vel) * delta_seconds;

    x_coeff = -sy * sp;
    y_coeff = sy * cp * sr;
    z_coeff = sy * cp * cr;
    double d_fy_d_pitch =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    x_coeff = cy * cp;
    y_coeff = cy * sp * sr - sy * cr;
    z_coeff = cy * sp * cr + sy * sr;
    double d_fy_d_yaw =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    y_coeff = cp * cr;
    z_coeff = -cp * sr;
    double d_fz_d_roll = (y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double d_f_yaw_d_roll = (cr * cpi * pitch_vel - sr * cpi * yaw_vel) * delta_seconds;

    x_coeff = -cp;
    y_coeff = -sp * sr;
    z_coeff = -sp * cr;
    double d_fz_d_pitch =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_seconds +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;
    double d_f_yaw_d_pitch =
      (sr * tp * cpi * pitch_vel + cr * tp * cpi * yaw_vel) * delta_seconds;

    // Much of the transfer function Jacobian is identical to the transfer
    // function
    _transfer_function_jacobian = _transfer_function;
    _transfer_function_jacobian(StateMemberX, StateMemberRoll) = d_fx_d_roll;
    _transfer_function_jacobian(StateMemberX, StateMemberPitch) = d_fx_d_pitch;
    _transfer_function_jacobian(StateMemberX, StateMemberYaw) = d_fx_d_yaw;
    _transfer_function_jacobian(StateMemberY, StateMemberRoll) = d_fy_d_roll;
    _transfer_function_jacobian(StateMemberY, StateMemberPitch) = d_fy_d_pitch;
    _transfer_function_jacobian(StateMemberY, StateMemberYaw) = d_fy_d_yaw;
    _transfer_function_jacobian(StateMemberZ, StateMemberRoll) = d_fz_d_roll;
    _transfer_function_jacobian(StateMemberZ, StateMemberPitch) = d_fz_d_pitch;
    _transfer_function_jacobian(StateMemberRoll, StateMemberRoll) = d_f_roll_d_roll;
    _transfer_function_jacobian(StateMemberRoll, StateMemberPitch) = d_f_roll_d_pitch;
    _transfer_function_jacobian(StateMemberPitch, StateMemberRoll) = d_f_pitch_d_roll;
    _transfer_function_jacobian(StateMemberYaw, StateMemberRoll) = d_f_yaw_d_roll;
    _transfer_function_jacobian(StateMemberYaw, StateMemberPitch) = d_f_yaw_d_pitch;

  }

  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_) {
    compute_dynamic_process_noise_covariance(state, dynamic_process_noise_covariance_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  // (3) Project the error forward: P = J * P * J' + Q
  if (_compute_covariance)
  {
    state_covariance =
      (_transfer_function_jacobian * state_covariance *
      _transfer_function_jacobian.transpose());
    state_covariance.noalias() +=
      delta_seconds * (*process_noise_covariance);
  }
  MB_DEBUG(
    "Predicted state is:\n" << state
      << "\nProcess noise covariance is:\n" << process_noise_covariance
      << "\nPredicted estimate error covariance is:\n" << state_covariance
      << "\n\n--------------------- /Ekf::predict ----------------------\n");
}

}  // namespace rpp_localization
