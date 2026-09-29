
#include "rpp_localization/models/constant_acc_model.hpp"


namespace rpp_localization
{
ConstantAccelerationModel::ConstantAccelerationModel(int state_dim)
: NavModelBase(state_dim),
  _transfer_function(STATE_SIZE, STATE_SIZE),
  _transfer_function_jacobian(STATE_SIZE, STATE_SIZE)
{
  _use_control = false;
  _compute_jacobian = true;
  _compute_covariance = true;
}

ConstantAccelerationModel::~ConstantAccelerationModel() {}


void ConstantAccelerationModel::init(std::shared_ptr<rclcpp::Node> node)
{
  NavModelBase::init(node);
  load_params();
  _transfer_function.setIdentity();
  _transfer_function_jacobian.setZero();
  _initialized = true;
}

void ConstantAccelerationModel::step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance,
    const rclcpp::Time & reference_time, const double delta_sec)
{

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

  prepareControl(reference_time, delta_sec);

  // Prepare the transfer function
  _transfer_function(StateMemberX, StateMemberVx) = cy * cp * delta_sec;
  _transfer_function(StateMemberX, StateMemberVy) =
    (cy * sp * sr - sy * cr) * delta_sec;
  _transfer_function(StateMemberX, StateMemberVz) =
    (cy * sp * cr + sy * sr) * delta_sec;
  _transfer_function(StateMemberX, StateMemberAx) =
    0.5 * _transfer_function(StateMemberX, StateMemberVx) * delta_sec;
  _transfer_function(StateMemberX, StateMemberAy) =
    0.5 * _transfer_function(StateMemberX, StateMemberVy) * delta_sec;
  _transfer_function(StateMemberX, StateMemberAz) =
    0.5 * _transfer_function(StateMemberX, StateMemberVz) * delta_sec;
  _transfer_function(StateMemberY, StateMemberVx) = sy * cp * delta_sec;
  _transfer_function(StateMemberY, StateMemberVy) =
    (sy * sp * sr + cy * cr) * delta_sec;
  _transfer_function(StateMemberY, StateMemberVz) =
    (sy * sp * cr - cy * sr) * delta_sec;
  _transfer_function(StateMemberY, StateMemberAx) =
    0.5 * _transfer_function(StateMemberY, StateMemberVx) * delta_sec;
  _transfer_function(StateMemberY, StateMemberAy) =
    0.5 * _transfer_function(StateMemberY, StateMemberVy) * delta_sec;
  _transfer_function(StateMemberY, StateMemberAz) =
    0.5 * _transfer_function(StateMemberY, StateMemberVz) * delta_sec;
  _transfer_function(StateMemberZ, StateMemberVx) = -sp * delta_sec;
  _transfer_function(StateMemberZ, StateMemberVy) = cp * sr * delta_sec;
  _transfer_function(StateMemberZ, StateMemberVz) = cp * cr * delta_sec;
  _transfer_function(StateMemberZ, StateMemberAx) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVx) * delta_sec;
  _transfer_function(StateMemberZ, StateMemberAy) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVy) * delta_sec;
  _transfer_function(StateMemberZ, StateMemberAz) =
    0.5 * _transfer_function(StateMemberZ, StateMemberVz) * delta_sec;
  _transfer_function(StateMemberRoll, StateMemberVroll) = delta_sec;
  _transfer_function(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_sec;
  _transfer_function(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_sec;
  _transfer_function(StateMemberPitch, StateMemberVpitch) = cr * delta_sec;
  _transfer_function(StateMemberPitch, StateMemberVyaw) = -sr * delta_sec;
  _transfer_function(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_sec;
  _transfer_function(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_sec;
  _transfer_function(StateMemberVx, StateMemberAx) = delta_sec;
  _transfer_function(StateMemberVy, StateMemberAy) = delta_sec;
  _transfer_function(StateMemberVz, StateMemberAz) = delta_sec;


  MB_DEBUG(
    "---------------------- Ekf::predict ----------------------\n" <<
      "delta is " << delta_sec << "\n" <<
      "state is " << state << "\n");

  // (1) Apply control terms, which are actually accelerations
  state(StateMemberVroll) +=
    control_acceleration_(ControlMemberVroll) * delta_sec;
  state(StateMemberVpitch) +=
    control_acceleration_(ControlMemberVpitch) * delta_sec;
  state(StateMemberVyaw) +=
    control_acceleration_(ControlMemberVyaw) * delta_sec;

  state(StateMemberAx) = (_control_update_vector[ControlMemberVx] ?
    control_acceleration_(ControlMemberVx) :
    state(StateMemberAx));
  state(StateMemberAy) = (_control_update_vector[ControlMemberVy] ?
    control_acceleration_(ControlMemberVy) :
    state(StateMemberAy));
  state(StateMemberAz) = (_control_update_vector[ControlMemberVz] ?
    control_acceleration_(ControlMemberVz) :
    state(StateMemberAz));


  // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
  state = _transfer_function * state;

  // Handle wrapping
  filter_utilities::wrapStateAngles(state);

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
    double one_half_at_squared = 0.5 * delta_sec * delta_sec;

    y_coeff = cy * sp * cr + sy * sr;
    z_coeff = -cy * sp * sr + sy * cr;
    double dFx_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double dFR_dR = 1.0 + (cr * tp * pitch_vel - sr * tp * yaw_vel) * delta_sec;

    x_coeff = -cy * sp;
    y_coeff = cy * cp * sr;
    z_coeff = cy * cp * cr;
    double dFx_dP =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;
    double dFR_dP =
      (cpi * cpi * sr * pitch_vel + cpi * cpi * cr * yaw_vel) * delta_sec;

    x_coeff = -sy * cp;
    y_coeff = -sy * sp * sr - cy * cr;
    z_coeff = -sy * sp * cr + cy * sr;
    double dFx_dY =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    y_coeff = sy * sp * cr - cy * sr;
    z_coeff = -sy * sp * sr - cy * cr;
    double dFy_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double dFP_dR = (-sr * pitch_vel - cr * yaw_vel) * delta_sec;

    x_coeff = -sy * sp;
    y_coeff = sy * cp * sr;
    z_coeff = sy * cp * cr;
    double dFy_dP =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    x_coeff = cy * cp;
    y_coeff = cy * sp * sr - sy * cr;
    z_coeff = cy * sp * cr + sy * sr;
    double dFy_dY =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;

    y_coeff = cp * cr;
    z_coeff = -cp * sr;
    double dFz_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
    double dFY_dR = (cr * cpi * pitch_vel - sr * cpi * yaw_vel) * delta_sec;

    x_coeff = -cp;
    y_coeff = -sp * sr;
    z_coeff = -sp * cr;
    double dFz_dP =
      (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
      (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) *
      one_half_at_squared;
    double dFY_dP =
      (sr * tp * cpi * pitch_vel + cr * tp * cpi * yaw_vel) * delta_sec;

    // Much of the transfer function Jacobian is identical to the transfer
    // function
    _transfer_function_jacobian = _transfer_function;
    _transfer_function_jacobian(StateMemberX, StateMemberRoll) = dFx_dR;
    _transfer_function_jacobian(StateMemberX, StateMemberPitch) = dFx_dP;
    _transfer_function_jacobian(StateMemberX, StateMemberYaw) = dFx_dY;
    _transfer_function_jacobian(StateMemberY, StateMemberRoll) = dFy_dR;
    _transfer_function_jacobian(StateMemberY, StateMemberPitch) = dFy_dP;
    _transfer_function_jacobian(StateMemberY, StateMemberYaw) = dFy_dY;
    _transfer_function_jacobian(StateMemberZ, StateMemberRoll) = dFz_dR;
    _transfer_function_jacobian(StateMemberZ, StateMemberPitch) = dFz_dP;
    _transfer_function_jacobian(StateMemberRoll, StateMemberRoll) = dFR_dR;
    _transfer_function_jacobian(StateMemberRoll, StateMemberPitch) = dFR_dP;
    _transfer_function_jacobian(StateMemberPitch, StateMemberRoll) = dFP_dR;
    _transfer_function_jacobian(StateMemberYaw, StateMemberRoll) = dFY_dR;
    _transfer_function_jacobian(StateMemberYaw, StateMemberPitch) = dFY_dP;

  }

  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (_use_dynamic_process_noise_covariance) {
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
      delta_sec * (*process_noise_covariance);
  }
  MB_DEBUG(
    "Predicted state is:\n" << state
      << "\nProcess noise covariance is:\n" << process_noise_covariance
      << "\nPredicted estimate error covariance is:\n" << state_covariance
      << "\n\n--------------------- /Ekf::predict ----------------------\n");
}



void ConstantAccelerationModel::load_params()
{

}
}  // namespace rpp_localization
