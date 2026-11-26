# Localization interface migration

Implementation status on 2026-11-26. Protocol/schema version remains 1.
Replay uses the existing RPP runtime API. Separately requested runtime alias
changes in `rpp_cpp` are reflected in this package's callers.

## Implemented source changes

- Runtime-bound EKF and UKF numerical cores with estimator-owned state,
  covariance and reference time; prepared dynamic Eigen matrices, correction
  workspaces, decompositions and sigma storage.
  Larger products use bounded fixed-size tiles and vector gain solves; large UKF
  Cholesky avoids Eigen's allocating packing paths.
- Shared `Model` and `Estimator` plugins, runtime field binding and
  `SignalRouter` composition. Algorithms supply their own core factory,
  requirements and parameters; the script and ROS adapter contain no filter
  whitelist. Models evaluate supplied state rather than owning the posterior.
- Concrete navigation physics behind the generic model boundary, with cached
  prediction/Jacobian evaluation for EKF and state-only sigma prediction for UKF.
- Prepared bounded replay rings, algorithm-specific checkpoints and rollback.
  Late gated/failed observations do not mutate the posterior. Arbitrary discrete
  noise intervals are not split by default; navigation profiles explicitly use
  constant-rate splitting, freezing recorded inputs and noise per interval.
  Closed-loop control and dynamic noise are not recomputed during replay.
- One logical estimator with an idle estimator/model replica owned by
  `SharedEstimatorBase`, non-virtual checkpoint/fork helpers, prepared operation
  mirroring and ROS-executor replay. The live worker adopts a corrected replica
  only at a matching event sequence, without duplicating the script or router.
  Lifecycle/reset cancel stale work while both owners are quiescent. Inline
  worker replay remains selectable for comparison.
- Benchmark-only replay count and history-tail mean, population standard
  deviation and maximum for both legacy and generic implementations. Generic
  CPU comparisons include the worker batch and the disjoint ROS replay scope.
- Dedicated numeric thread with bounded prepared SPSC ingress/output queues.
  Live native guarded calls execute on the worker; replay calls use the spare
  on the ROS executor. Initialization, lifecycle, layout
  preparation, ROS conversion, TF, publication and message allocation execute on
  the owner executor while numeric execution is paused when necessary.
- Generic ROS state/input/measurement/prediction/reset APIs. Stable resource
  output definitions are advertised in layout metadata. Slot replacement,
  cleanup, reset, backward clock jumps and input expiration are handled.
- Optional navigation interface for planar/spatial state roles, odometry, pose,
  twist, IMU, TF/extrinsics, gravity removal, relative/differential pose,
  selected/invalid axes, planar constraints, set-pose and control/dynamic-noise
  preparation. Observation subsets and covariance routes are configured once.
- Explicit control-compatible 2D/3D navigation state projection, separately
  advertised from the complete estimator output.
- New shared `coe-marble/rpp_ros_interfaces` repository, grouped by common,
  control and estimation domains; local control and other consumers migrated
  mechanically to the shared ROS package namespace.
- Default ROS entrypoints now use the generic worker; previous ROS adapters are
  preserved under `legacy_*` executable names and a legacy launch file. Original
  EKF, constant-acceleration model and navigation model base remain unchanged.

## Extending the stack

A new Gaussian algorithm can derive `SharedEstimatorBase`, implement its native
filter core/factory and capability requirements, and declare its own parameters.
Other posterior families implement the shared `Estimator` interface directly.
Register the plugin and select it in workspace configuration. No ROS or script
algorithm branches are necessary. Current Gaussian cores support Euclidean and
scalar-angle states; quaternion/manifold numerical filtering remains a separate
algorithm capability, not implied by generic ROS transport.

Generic filter code currently lives under `rpp_localization/estimation`; moving
it into a standalone `rpp_estimation` library can be a later packaging change.
It is not required for generic sizing or this localization composition.

## Replay switching completion

The source implementation now covers:

- Idle spare, checkpoint/history transfer, mirrored operations, bounded ROS
  catch-up and worker-owned adoption at an identical event sequence.
- Ready notifications and adoption between ingress operations. Refreshing the
  worker's cached estimate precedes state-dependent input/noise policies;
  snapshot-pool pressure defers further execution to the next worker tick.
- Wall-time replay progress and publication of the adopted estimate after
  simulated time stops. The normal publication timer remains tied to ROS time.
- Shared activation/deactivation, cancellation on reset or actual enable-state
  transitions, and inactive reload followed by initialization of both instances.
  Selected-instance identity is rechecked before resuming after slot access.
- No spare when history is disabled. Known-stale samples cannot move history to
  a spare; stale samples in an active replay are skipped without discarding
  corrections already completed. Other replay failures retain the live estimate.
- Replay history statistics and job/switch/cancellation/stale-sample counts in
  profiling builds only. CPU comparisons include the worker and ROS replay scopes.

The spare uses the persisted component/model profile. A native host that uses
builder parameter overrides must also supply identical persisted configuration
for cloning. Exporting every resolved parameter and construction override would
require a separate RPP runtime API change; this ROS host does not use overrides.
After a cancelled fork, the running history starts at that fork checkpoint.
Both limitations are documented in the bag benchmark guide.

## Review and validation

User instruction: show the diff and do not build or run tests. Consequently all
new executable paths and regression cases remain unbuilt and unrun.

Prepared coverage includes runtime dimensions, nonlinear observations, angles,
model failures, covariance validation, storage reuse, original numerical parity,
EKF/UKF late replay and rollback, noise-split rejection, SPSC coherence, snapshot
lifetime/exhaustion, registered Model/Estimator RPC adapters, field/covariance
routing, lifecycle, and ROS navigation/projection/reset across the worker boundary.
Additional source cases cover both EKF and UKF with history disabled, repeated
role switches, stale samples within replay, cancellation while active/ready,
metrics aggregated across physical instances, a stopped simulated clock, and
reload after adoption. These cases have not been built or run.

When execution is authorized, first register current `rpp_interfaces` and
localization plugins, then use the colcon commands in README. Validate every
observation shape and replay path with allocation instrumentation, including
registered model calls and output handling. Compare end-to-end latency and
worst retained-tail replay separately. Verify ROS reset/reload/clock behavior and
the control projection using the supplied profiles. Do not treat static review
or earlier microbenchmarks as full-stack performance evidence.

The user's earlier benchmark table is preserved in
`benchmarks/results/generic-matrices-2026-11-25.txt`. Its zero allocation counts
and timing deltas apply to that matrix comparison, not the rewritten pipeline.
