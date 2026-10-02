@0xf2496b5c22970fc9;

using Anot = import "rpp_common/anot.capnp";

# RPP localization plugin contract, version 1.
#
# Schema evolution is append-only: field and method ordinals are wire ABI.
# Never renumber, reuse, or change the type of an existing ordinal. Add new
# fields and methods at the end, and use a new payload when a semantic change
# cannot be expressed as an optional extension.

# Exactly 15 Float64 values, in this fixed order:
# [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az].
# Position is metres in the model navigation frame; roll/pitch/yaw are
# radians; linear velocity and acceleration are metres/s and metres/s^2;
# angular velocity is radians/s. The velocity, acceleration, and angular-rate
# triples use the model body frame, matching the legacy constant-acceleration
# model. The selected model declares its world/body convention; callers must
# not mix conventions within one filter instance.
struct State15 {
  values @0 :List(Float64);
}

# Exactly 225 Float64 values for the State15 covariance, stored row-major:
# value[15 * row + column]. Rows and columns use the State15 ordering.
struct Covariance15 {
  values @0 :List(Float64);
}

# A time-stamped six-axis model control in the order
# [vx, vy, vz, vroll, vpitch, vyaw]. Values are model-defined commands:
# ConstantAccelerationModel interprets them as legacy target velocities.
#
# `present = false` is the only no-control encoding; in that case every other
# field is ignored. When present, `values` and `enabled` each contain exactly
# six entries, every enabled value is finite, and `stampNs` uses the same clock
# domain as the prediction reference time. Disabled entries are ignored.
struct Control6 {
  present @0 :Bool;
  stampNs @1 :Int64;
  values @2 :List(Float64);
  enabled @3 :List(Bool);
}

# One state-15 measurement. `updateMask` has exactly 15 entries; only enabled
# state values and their covariance rows/columns participate in correction.
# `mahalanobisThreshold` is positive infinity when rejection is disabled.
# The timestamp is expressed in the filter caller-selected clock domain.
struct Measurement15 {
  state @0 :State15;
  covariance @1 :Covariance15;
  updateMask @2 :List(Bool);
  referenceTimeNs @3 :Int64;
  mahalanobisThreshold @4 :Float64;
  sourceName @5 :Text;
}

# A filtered state at `referenceTimeNs`, using the caller-selected clock
# domain. State and covariance keep the fixed layouts above.
struct Estimate15 {
  state @0 :State15;
  covariance @1 :Covariance15;
  referenceTimeNs @2 :Int64;
}

# `code` is stable: 0=ok, 1=invalidPayload, 2=invalidTime,
# 3=invalidState, 4=invalidCovariance, 5=notInitialized,
# 6=staleMeasurement, 7=modelFailure, 8=unsupportedControl,
# 9=numericalFailure. Unknown nonzero values are errors.
struct LocalizationStatus {
  code @0 :UInt16;
  message @1 :Text;
}

# All model-predict inputs share a clock domain. `deltaNs` is a non-negative
# elapsed duration in that domain; a negative value is invalid rather than a
# request to rewind the process model.
struct LocalizationModelPredictInput15 {
  state @0 :State15;
  covariance @1 :Covariance15;
  control @2 :Control6;
  referenceTimeNs @3 :Int64;
  deltaNs @4 :Int64;
}

struct LocalizationModelPredictOutput15 {
  state @0 :State15;
  covariance @1 :Covariance15;
  status @2 :LocalizationStatus;
}

struct LocalizationPredictInput15 {
  control @0 :Control6;
  referenceTimeNs @1 :Int64;
  deltaNs @2 :Int64;
}

struct LocalizationFilterResult15 {
  estimate @0 :Estimate15;
  status @1 :LocalizationStatus;
}

# Implementations validate all fixed lengths and finite selected values. A
# covariance must be finite and row-major; state/process covariance diagonals
# must be non-negative. For correction compatibility with the legacy EKF/UKF,
# enabled measurement covariance diagonals are normalized as abs(value), then
# floored at 1e-9. A wrong shape or non-finite value is rejected instead.
#
# A VehicleModel3D bridge maps State15[0..11] to its Odometry3D pose and
# body-twist. It derives State15[12..14] from the propagated body linear
# velocity, owns its numerical Jacobian and process covariance, and maps
# Control6 to vehicle commands explicitly. A bridge must reject a
# VehicleModel3D that exposes only a CasADi graph or lacks a declared control
# mapping; it must never silently fall back to an identity prediction.
interface NavModel15 $Anot.plugin("NavModel15") {
  predict @0 (input :LocalizationModelPredictInput15)
      -> (output :LocalizationModelPredictOutput15);
}

# The filter lifecycle is initialize, predict/correct, estimate, and reset.
# `correct` may report staleMeasurement when its replay policy cannot accept
# the timestamp. Lifecycle operations return status payloads instead of using
# transport errors for ordinary validation failures.
interface LocalizationFilter15 $Anot.plugin("LocalizationFilter15") {
  initialize @0 (initialEstimate :Estimate15) -> (status :LocalizationStatus);
  predict @1 (input :LocalizationPredictInput15)
      -> (result :LocalizationFilterResult15);
  correct @2 (measurement :Measurement15)
      -> (result :LocalizationFilterResult15);
  getEstimate @3 () -> (result :LocalizationFilterResult15);
  reset @4 (initialEstimate :Estimate15) -> (status :LocalizationStatus);
}
