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
  describe @1 () -> (description :NavModelDescription15);
}

# The filter lifecycle is initialize, predict/correct, estimate, and reset.
# `correct` may report staleMeasurement when its replay policy cannot accept
# the timestamp. Lifecycle operations return status payloads instead of using
# transport errors for ordinary validation failures.
# What a caller must know about a model to feed it consistently.
struct NavModelDescription15 {
  # True when the model turns Control6 into an acceleration. A measured
  # acceleration then competes with the control, so the caller disables the
  # control on every axis whose acceleration it measures. False for a model
  # that reads Control6 as an actuator command.
  controlDrivesAcceleration @0 :Bool;
}

interface LocalizationFilter15 $Anot.plugin("LocalizationFilter15") {
  initialize @0 (initialEstimate :Estimate15) -> (status :LocalizationStatus);
  predict @1 (input :LocalizationPredictInput15)
      -> (result :LocalizationFilterResult15);
  correct @2 (measurement :Measurement15)
      -> (result :LocalizationFilterResult15);
  getEstimate @3 () -> (result :LocalizationFilterResult15);
  reset @4 (initialEstimate :Estimate15) -> (status :LocalizationStatus);
  # Description of the model the filter predicts with.
  describeModel @5 () -> (description :NavModelDescription15);
}

# ---------------------------------------------------------------------------
# Invariant filter contract, version 1.
#
# An invariant filter estimates an inertial navigation state on a Lie group
# and is driven by IMU samples: the IMU is the input of every prediction, not
# a measurement. It does not share the State15 lifecycle above.
# ---------------------------------------------------------------------------

# Extended pose with IMU biases. `rotation` holds the 9 row-major values of
# the rotation from body to world; `velocity` and `position` are in the world
# frame; the biases are in the body frame. Vectors have exactly 3 values.
#
# `covariance` holds the 225 row-major values of the error covariance on the
# tangent space, ordered rotation, velocity, position, gyroscope bias,
# accelerometer bias. `rightInvariant` fixes the error convention: when true,
# truth = exp(error) * estimate; when false, truth = estimate * exp(error).
# A filter instance uses one convention and rejects states in the other.
struct InertialState {
  rotation @0 :List(Float64);
  velocity @1 :List(Float64);
  position @2 :List(Float64);
  gyroBias @3 :List(Float64);
  accelBias @4 :List(Float64);
  covariance @5 :List(Float64);
  rightInvariant @6 :Bool;
}

# One IMU reading in the body frame: angular velocity in radians/s and
# specific force in metres/s^2, each with exactly 3 values. A level IMU at
# rest reads +gravity on the axis that points up.
struct ImuSample {
  angularVelocity @0 :List(Float64);
  specificForce @1 :List(Float64);
}

struct InertialModelPredictInput {
  state @0 :InertialState;
  imu @1 :ImuSample;
  referenceTimeNs @2 :Int64;
  deltaNs @3 :Int64;
}

struct InertialModelPredictOutput {
  state @0 :InertialState;
  status @1 :LocalizationStatus;
}

# Propagates an inertial state and its covariance with one IMU sample held
# over `deltaNs`. A model is stateless: everything it needs is in the input.
interface InertialModel $Anot.plugin("InertialModel") {
  predict @0 (input :InertialModelPredictInput)
      -> (output :InertialModelPredictOutput);
}

# One measurement of a quantity the filter knows how to relate to its state.
# `kind` is stable: 0=position, 1=bodyVelocity, 2=attitude.
#   position:     3 values, world position of the sensor, world-frame covariance
#   bodyVelocity: 3 values, velocity of the sensor in the body frame
#   attitude:     9 row-major values, rotation from body to world; the
#                 covariance is of the small rotation error in the body frame
# `covariance` has 9 row-major values. `leverArm` is the position of the
# sensor in the body frame, 3 values, and is ignored for an attitude.
# `mahalanobisThreshold` is positive infinity when rejection is disabled.
struct InvariantMeasurement {
  kind @0 :UInt16;
  values @1 :List(Float64);
  covariance @2 :List(Float64);
  leverArm @3 :List(Float64);
  referenceTimeNs @4 :Int64;
  mahalanobisThreshold @5 :Float64;
  sourceName @6 :Text;
}

# `angularVelocity` is the last IMU rate with the gyroscope bias removed.
struct InertialEstimate {
  state @0 :InertialState;
  angularVelocity @1 :List(Float64);
  referenceTimeNs @2 :Int64;
}

struct InvariantPredictInput {
  imu @0 :ImuSample;
  referenceTimeNs @1 :Int64;
  deltaNs @2 :Int64;
}

# `accepted` is false when the Mahalanobis gate rejected a measurement.
struct InvariantFilterResult {
  estimate @0 :InertialEstimate;
  status @1 :LocalizationStatus;
  accepted @2 :Bool;
}

# The lifecycle is initialize, predict/correct, getEstimate, and reset.
# `predict` advances the estimate to `referenceTimeNs` and must be called
# with the time advancing by exactly `deltaNs`. `correct` applies a
# measurement to the current estimate; version 1 keeps no history, so a
# measurement stamped after the estimate is rejected with invalidTime and an
# older one is applied as if taken now.
interface InvariantFilter $Anot.plugin("InvariantFilter") {
  initialize @0 (initialEstimate :InertialEstimate) -> (status :LocalizationStatus);
  predict @1 (input :InvariantPredictInput) -> (result :InvariantFilterResult);
  correct @2 (measurement :InvariantMeasurement) -> (result :InvariantFilterResult);
  getEstimate @3 () -> (result :InvariantFilterResult);
  reset @4 (initialEstimate :InertialEstimate) -> (status :LocalizationStatus);
}
