#pragma once

// Benchmark-only model. The equations are copied from ConstantAccelerationModel.
// Both kernels share this class, all backing buffers, control policy, and math.
// StateSize changes only the Eigen Map types. The navigation layout remains 15.
#include <stdexcept>

#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/models/navigation_model_base.hpp"

namespace rpp_localization::test
{

class ModelMatrixComparison final : public NavigationModelBase
{
public:
  // A prepared binding borrows this model and must not outlive it.
  class PredictionBinding
  {
  public:
    void predict(TimestampNs reference_time, DurationNs delta) const
    {
      kernel_(*model_, reference_time, delta);
    }

  private:
    friend class ModelMatrixComparison;
    using Kernel = void (*)(ModelMatrixComparison&, TimestampNs, DurationNs);
    PredictionBinding(ModelMatrixComparison& model, Kernel kernel)
    : model_(&model), kernel_(kernel) {}

    ModelMatrixComparison* model_;
    Kernel kernel_;
  };

  explicit ModelMatrixComparison(int state_size)
  : NavigationModelBase(state_size),
    transfer_(state_size, state_size),
    jacobian_(state_size, state_size),
    predicted_state_(state_size),
    projected_covariance_(state_size, state_size),
    velocity_matrix_(TWIST_SIZE, TWIST_SIZE),
    velocity_covariance_(TWIST_SIZE, TWIST_SIZE)
  {
    if (state_size != STATE_SIZE) {
      throw std::invalid_argument("Model comparison uses the existing 15-state navigation equations");
    }
    transfer_.setIdentity();
    jacobian_.setZero();
    initialized_ = true;
  }

  template<int StateSize>
  PredictionBinding prepare_prediction()
  {
    static_assert(StateSize == Eigen::Dynamic || StateSize > 0, "Invalid model state size");
    if constexpr (StateSize != Eigen::Dynamic) {
      if (_state_dim != StateSize) {
        throw std::invalid_argument("Model comparison binding must match the configured state dimension");
      }
    }
    return PredictionBinding(*this, &predict_bound<StateSize>);
  }

  void predict(
    StateVector& state, CovarianceMatrix& covariance,
    TimestampNs reference_time, DurationNs delta) override
  {
    predict_impl<Eigen::Dynamic>(state, covariance, reference_time, delta);
  }

private:
  template<int StateSize>
  static void predict_bound(
    ModelMatrixComparison& model, TimestampNs reference_time, DurationNs delta)
  {
    model.predict_impl<StateSize>(model._state, model._state_covariance, reference_time, delta);
  }

  template<int StateSize>
  void predict_impl(
    StateVector& state_storage, CovarianceMatrix& covariance_storage,
    TimestampNs reference_time, DurationNs delta);

  void compute_prepared_dynamic_noise(const StateVector& state)
  {
    velocity_matrix_.setIdentity();
    velocity_matrix_.diagonal() *= state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();
    velocity_covariance_.noalias() = velocity_matrix_ *
      process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(POSITION_OFFSET, POSITION_OFFSET);
    dynamic_process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
      POSITION_OFFSET, POSITION_OFFSET).noalias() = velocity_covariance_ * velocity_matrix_.transpose();
  }

  Eigen::MatrixXd transfer_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd predicted_state_;
  Eigen::MatrixXd projected_covariance_;
  Eigen::MatrixXd velocity_matrix_;
  Eigen::MatrixXd velocity_covariance_;
};

template<int StateSize>
inline void ModelMatrixComparison::predict_impl(
  StateVector& state_storage, CovarianceMatrix& covariance_storage,
  TimestampNs reference_time, DurationNs delta)
{
  using VectorType = Eigen::Matrix<double, StateSize, 1>;
  using MatrixType = Eigen::Matrix<double, StateSize, StateSize>;
  const Eigen::Index state_size = _state_dim;
  Eigen::Map<VectorType> state(state_storage.data(), state_size);
  Eigen::Map<MatrixType> state_covariance(covariance_storage.data(), state_size, state_size);
  Eigen::Map<MatrixType> transfer(transfer_.data(), state_size, state_size);
  Eigen::Map<MatrixType> jacobian(jacobian_.data(), state_size, state_size);
  Eigen::Map<VectorType> predicted_state(predicted_state_.data(), state_size);
  Eigen::Map<MatrixType> projected_covariance(projected_covariance_.data(), state_size, state_size);
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
  transfer(StateMemberX, StateMemberVx) = cy * cp * delta_seconds;
  transfer(StateMemberX, StateMemberVy) =
    (cy * sp * sr - sy * cr) * delta_seconds;
  transfer(StateMemberX, StateMemberVz) =
    (cy * sp * cr + sy * sr) * delta_seconds;
  transfer(StateMemberX, StateMemberAx) =
    0.5 * transfer(StateMemberX, StateMemberVx) * delta_seconds;
  transfer(StateMemberX, StateMemberAy) =
    0.5 * transfer(StateMemberX, StateMemberVy) * delta_seconds;
  transfer(StateMemberX, StateMemberAz) =
    0.5 * transfer(StateMemberX, StateMemberVz) * delta_seconds;
  transfer(StateMemberY, StateMemberVx) = sy * cp * delta_seconds;
  transfer(StateMemberY, StateMemberVy) =
    (sy * sp * sr + cy * cr) * delta_seconds;
  transfer(StateMemberY, StateMemberVz) =
    (sy * sp * cr - cy * sr) * delta_seconds;
  transfer(StateMemberY, StateMemberAx) =
    0.5 * transfer(StateMemberY, StateMemberVx) * delta_seconds;
  transfer(StateMemberY, StateMemberAy) =
    0.5 * transfer(StateMemberY, StateMemberVy) * delta_seconds;
  transfer(StateMemberY, StateMemberAz) =
    0.5 * transfer(StateMemberY, StateMemberVz) * delta_seconds;
  transfer(StateMemberZ, StateMemberVx) = -sp * delta_seconds;
  transfer(StateMemberZ, StateMemberVy) = cp * sr * delta_seconds;
  transfer(StateMemberZ, StateMemberVz) = cp * cr * delta_seconds;
  transfer(StateMemberZ, StateMemberAx) =
    0.5 * transfer(StateMemberZ, StateMemberVx) * delta_seconds;
  transfer(StateMemberZ, StateMemberAy) =
    0.5 * transfer(StateMemberZ, StateMemberVy) * delta_seconds;
  transfer(StateMemberZ, StateMemberAz) =
    0.5 * transfer(StateMemberZ, StateMemberVz) * delta_seconds;
  transfer(StateMemberRoll, StateMemberVroll) = delta_seconds;
  transfer(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_seconds;
  transfer(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_seconds;
  transfer(StateMemberPitch, StateMemberVpitch) = cr * delta_seconds;
  transfer(StateMemberPitch, StateMemberVyaw) = -sr * delta_seconds;
  transfer(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_seconds;
  transfer(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_seconds;
  transfer(StateMemberVx, StateMemberAx) = delta_seconds;
  transfer(StateMemberVy, StateMemberAy) = delta_seconds;
  transfer(StateMemberVz, StateMemberAz) = delta_seconds;


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
  predicted_state.noalias() = transfer * state;
  state = predicted_state;

  // Handle wrapping
  for (const auto index : {StateMemberRoll, StateMemberPitch, StateMemberYaw}) {
    state(index) = angles::normalize_angle(state(index));
  }

  MB_DEBUG(
    "Transfer function is:\n" <<
      transfer << "\nCurrent state is:\n" <<
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
    jacobian = transfer;
    jacobian(StateMemberX, StateMemberRoll) = d_fx_d_roll;
    jacobian(StateMemberX, StateMemberPitch) = d_fx_d_pitch;
    jacobian(StateMemberX, StateMemberYaw) = d_fx_d_yaw;
    jacobian(StateMemberY, StateMemberRoll) = d_fy_d_roll;
    jacobian(StateMemberY, StateMemberPitch) = d_fy_d_pitch;
    jacobian(StateMemberY, StateMemberYaw) = d_fy_d_yaw;
    jacobian(StateMemberZ, StateMemberRoll) = d_fz_d_roll;
    jacobian(StateMemberZ, StateMemberPitch) = d_fz_d_pitch;
    jacobian(StateMemberRoll, StateMemberRoll) = d_f_roll_d_roll;
    jacobian(StateMemberRoll, StateMemberPitch) = d_f_roll_d_pitch;
    jacobian(StateMemberPitch, StateMemberRoll) = d_f_pitch_d_roll;
    jacobian(StateMemberYaw, StateMemberRoll) = d_f_yaw_d_roll;
    jacobian(StateMemberYaw, StateMemberPitch) = d_f_yaw_d_pitch;

  }

  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_) {
    compute_prepared_dynamic_noise(state_storage);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  // (3) Project the error forward: P = J * P * J' + Q
  if (_compute_covariance)
  {
    projected_covariance.noalias() = jacobian * state_covariance;
    state_covariance.noalias() = projected_covariance * jacobian.transpose();
    const Eigen::Map<const MatrixType> noise(
      process_noise_covariance->data(), state_size, state_size);
    state_covariance.noalias() += delta_seconds * noise;
  }
  MB_DEBUG(
    "Predicted state is:\n" << state
      << "\nProcess noise covariance is:\n" << process_noise_covariance
      << "\nPredicted estimate error covariance is:\n" << state_covariance
      << "\n\n--------------------- /Ekf::predict ----------------------\n");
}

}  // namespace rpp_localization::test
