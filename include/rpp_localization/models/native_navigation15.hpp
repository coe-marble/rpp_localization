#pragma once

#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/estimation/model.hpp"

#include <array>

namespace rpp_localization
{

// Concrete navigation physics behind the generic process-model interface.
// Owns configuration and reusable evaluation scratch, never an estimate or P.
// Prepared control inputs are [six effective accelerations, three 0/1 linear
// acceleration override flags]. Command age/timeout/gains belong to input
// preparation. With prepared_control=false the input layout is empty.
class NativeNavigationModel15 final : public estimation::ProcessModel
{
public:
  using VectorIn = estimation::VectorIn;
  using VectorOut = estimation::VectorOut;
  using MatrixOut = estimation::MatrixOut;
  using TimeNs = estimation::TimeNs;

  explicit NativeNavigationModel15(bool prepared_control = false)
  : space_(STATE_SIZE, {StateMemberRoll, StateMemberPitch, StateMemberYaw}),
    prepared_control_(prepared_control)
  {
    _transfer_function.setIdentity();
    _transfer_function_jacobian.setZero();
  }

  const estimation::VectorSpace& state_space() const override { return space_; }
  Eigen::Index input_size() const override { return prepared_control_ ? 9 : 0; }

  void enable_evaluation_cache(bool enabled = true) const
  {
    if (cache_enabled_ != enabled) {cached_ = false; jacobian_ready_ = false;}
    cache_enabled_ = enabled;
  }

  bool predict(VectorIn state, VectorIn input, TimeNs, TimeNs dt, TimeNs,
    VectorOut next) const override
  {
    if (!evaluate(state, input, dt, false)) {
      return false;
    }
    next = state_work_;
    return true;
  }

  bool linearize(VectorIn state, VectorIn input, TimeNs, TimeNs dt, TimeNs,
    MatrixOut a) const override
  {
    if (!evaluate(state, input, dt, true)) {
      return false;
    }
    a = _transfer_function_jacobian;
    return true;
  }

  bool predict_and_linearize(VectorIn state, VectorIn input, TimeNs, TimeNs dt,
    TimeNs, VectorOut next, MatrixOut a) const override
  {
    if (!evaluate(state, input, dt, true)) {
      return false;
    }
    next = state_work_;
    a = _transfer_function_jacobian;
    return true;
  }

  // Full shared-Model linearization. Binary override flags are held fixed;
  // their Jacobian columns are zero. Input slew policy is prepared by the caller.
  bool linearize_full(VectorIn state, VectorIn input, TimeNs dt,
    MatrixOut a, MatrixOut b) const
  {
    if (!evaluate(state, input, dt, true)) {return false;}
    a = _transfer_function_jacobian;
    b.setZero();
    if (prepared_control_) {
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        if (input(6 + axis) != 0.0) {
          b.col(axis) = _transfer_function.col(StateMemberAx + axis);
        }
        b.col(3 + axis) = _transfer_function.col(StateMemberVroll + axis) *
          (static_cast<double>(dt) * 1e-9);
      }
    }
    return true;
  }

private:
  bool evaluate(VectorIn origin, VectorIn input, TimeNs delta, bool with_jacobian) const;
  void prepare_cached_jacobian() const;

  estimation::VectorSpace space_;
  bool prepared_control_;
  // Prediction and linearization are separate shared-interface methods. Reuse
  // their identical evaluation without making UKF sigma predictions compute A.
  mutable Eigen::Matrix<double, STATE_SIZE, 1> state_work_, cached_origin_;
  mutable Eigen::Matrix<double, 9, 1> cached_input_;
  mutable std::array<double, 8> cached_trig_{};
  mutable TimeNs cached_dt_{0};
  mutable bool cache_enabled_{false}, cached_{false}, jacobian_ready_{false};
  mutable Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _transfer_function;
  mutable Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> _transfer_function_jacobian;
};

inline bool NativeNavigationModel15::evaluate(
  VectorIn origin, VectorIn input, TimeNs delta, bool with_jacobian) const
{
  if (origin.size() != STATE_SIZE || input.size() != input_size() ||
    !origin.allFinite() || !input.allFinite())
  {
    return false;
  }
  if (prepared_control_) {
    for (Eigen::Index flag = 6; flag < 9; ++flag) {
      if (input(flag) != 0.0 && input(flag) != 1.0) {
        return false;
      }
    }
  }
  if (cache_enabled_ && with_jacobian && cached_ && delta == cached_dt_ && origin == cached_origin_) {
    bool same_input = true;
    for (Eigen::Index index = 0; index < input.size(); ++index) {
      same_input = same_input && input(index) == cached_input_(index);
    }
    if (same_input) {prepare_cached_jacobian(); return _transfer_function_jacobian.allFinite();}
  }
  auto& state = state_work_;
  state = origin;
  const double delta_seconds = static_cast<double>(delta) * 1e-9;

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

  // (1) Apply control terms, which are actually accelerations
  state(StateMemberVroll) +=
    (prepared_control_ ? input(ControlMemberVroll) : 0.0) * delta_seconds;
  state(StateMemberVpitch) +=
    (prepared_control_ ? input(ControlMemberVpitch) : 0.0) * delta_seconds;
  state(StateMemberVyaw) +=
    (prepared_control_ ? input(ControlMemberVyaw) : 0.0) * delta_seconds;

  state(StateMemberAx) = ((prepared_control_ && input(6) != 0.0) ?
    input(ControlMemberVx) :
    state(StateMemberAx));
  state(StateMemberAy) = ((prepared_control_ && input(7) != 0.0) ?
    input(ControlMemberVy) :
    state(StateMemberAy));
  state(StateMemberAz) = ((prepared_control_ && input(8) != 0.0) ?
    input(ControlMemberVz) :
    state(StateMemberAz));

  // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
  state = _transfer_function * state;

  // Handle wrapping
  for (const auto axis : {StateMemberRoll, StateMemberPitch, StateMemberYaw}) {
    state(axis) = ::angles::normalize_angle(state(axis));
  }

  cached_ = state.allFinite(); jacobian_ready_ = false;
  cached_dt_ = delta;
  if (cache_enabled_) {
    cached_origin_ = origin;
    for (Eigen::Index index = 0; index < input.size(); ++index) {cached_input_(index) = input(index);}
  }
  if (with_jacobian || cache_enabled_) {cached_trig_ = {sp, cp, cpi, tp, sr, cr, sy, cy};}
  if (with_jacobian && cached_) {prepare_cached_jacobian();}
  return cached_ && (!with_jacobian || _transfer_function_jacobian.allFinite());
}

inline void NativeNavigationModel15::prepare_cached_jacobian() const
{
  if (jacobian_ready_) {return;}
  const auto& state = state_work_;
  const double delta_seconds = static_cast<double>(cached_dt_) * 1e-9;
  const double sp = cached_trig_[0], cp = cached_trig_[1], cpi = cached_trig_[2], tp = cached_trig_[3];
  const double sr = cached_trig_[4], cr = cached_trig_[5], sy = cached_trig_[6], cy = cached_trig_[7];
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

  jacobian_ready_ = true;
}

}  // namespace rpp_localization
