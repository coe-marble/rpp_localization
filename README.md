rpp_localization
================

Runtime-sized state estimation through `rpp_interfaces::Estimator` and
`rpp_interfaces::Model`. The default ROS entrypoints execute the estimator on a
separate numerical thread. An optional navigation interface provides
`odometry/filtered`, sensor conversion, TF, and a state projection for control.

Composition
-----------

```text
ROS executor                         Numerical worker
sensor conversion, TF                GenericLocalizationScript / SignalRouter
lifecycle, reset, publication  <-->     Estimator (owns state, P, time, replay)
         prepared SPSC queues            Model (evaluates state and input)
```

The script and generic ROS node get state, input, and observation layouts from
the selected estimator. They contain no EKF/UKF selection or fixed state width.
`Ekf` and `Ukf` are initial implementations. Gaussian filters
can reuse `SharedEstimatorBase` by supplying a core factory and capability
requirements. Other families can implement `Estimator` directly.

Generic Eigen matrix sizes bind during configuration. Posterior and candidate
storage, per-observation correction workspaces, decompositions, sigma points,
wire buffers, queues, and replay checkpoints are prepared before execution.
The navigation model keeps its concrete 15-coordinate physics internally; the
filters accept other model dimensions. Current Gaussian cores support Euclidean
and scalar-angle coordinates. Quaternion or other manifold Gaussian filtering
requires a different core; the ROS transport already distinguishes stored and
tangent covariance dimensions.

Small matrix products retain Eigen's normal kernels. Larger products use bounded
fixed-size tiles; large gain solves use prepared vector solves and large UKF
Cholesky uses existing storage directly, avoiding hidden packing allocations.
Their throughput still needs measurement on the target system.

The worker performs native guarded estimator/model calls. RPC adapters are
available for external communication; RPC components are rejected inside the
numerical chain. ROS decoding, allocations for messages, TF, publication,
lifecycle, and initialization run on the executor. Reconfiguration pauses the
worker and rebuilds its storage and routes. Use a `SingleThreadedExecutor` for
this node, as the supplied executables do.

Queues have fixed capacities. Full ingress returns `codeBudgetExceeded`;
publication can drop snapshots while estimation continues. Shared interface
snapshots use three prepared slots; retaining every slot returns
`codeBudgetExceeded` until a view is released. Set `history_capacity=0` in the
estimator's RPP parameters to disable replay and its checkpoint-copy cost.

Shared ROS definitions
----------------------

Messages, services, and actions belong to
[coe-marble/rpp_ros_interfaces](https://github.com/coe-marble/rpp_ros_interfaces).
Sources are grouped under `common`, `control`, and `estimation` folders;
generated types use the usual `rpp_ros_interfaces::msg`, `srv`, and `action`
namespaces. Control consumers now use that package as well.

The generic adapter exposes:

| Topic/service | Type and meaning |
| --- | --- |
| `~/layout` | `EstimatorLayout`, transient: revision, state/input/observation fields, and stable resource interface orders |
| `~/measurement` | `EstimatorMeasurement`: port index, values, full tangent covariance, acquisition timestamp and layout revision |
| `~/prediction` | `EstimatorPrediction`: explicit end time, duration, input slew interval, inputs and discrete Q |
| `~/input` | `EstimatorInput`: latest prepared input and its acquisition/slew times |
| `~/estimate` | `EstimatorEstimate`: state and row-major tangent covariance in declared estimator order |
| `~/state` | `NumericSample`, grouped by resource; exact value order is advertised in `layout.state_interfaces` |
| `~/status` | `EstimatorStatus` |
| `~/reset` | `ResetEstimate`, full posterior reset while the worker is paused |
| `~/enable` | `std_srvs/SetBool`, execution and estimator lifecycle |

RPP slot-management services remain available through `rpp_ros2`. Resource
interface names keep their field order and meaning throughout the node's
lifetime; incompatible redefinitions fail during preparation. Use the current
layout revision on ingress. Future timestamps and invalid shapes are rejected.
Process covariance supplied to `~/prediction` is already discretized.
`process_noise_density` is multiplied by elapsed seconds for automatic prediction.

Navigation interface
--------------------

`expose_navigation_topics=true` attaches `NavigationInterface`. It binds roles
by resource, physical quantity, coordinate kind, axis, and frame. It accepts
planar or spatial state layouts with additional unrelated state coordinates.
Pose fields declare the world and body frames; velocity/acceleration fields use
the body frame. The supplied model examples use `map`, `base_link`, and resource
`base`. Set model and measurement metadata together in the RPP workspace.

The interface converts `odometry/input`, `pose/input`, `twist/input`, and
`imu/data` into configured observation ports. Topic names and selected axes are
startup parameters; an empty input topic disables its subscription. It handles
partial invalid samples, covariance cross terms, frame/lever-arm transforms,
optional gravity removal, relative/differential poses, and planar constraints.
Off-center IMU acceleration waits for angular-acceleration history. Differential
pose covariance uses independent successive pose uncertainty divided by dt².
`set_pose` topic/service resets the navigation posterior and preprocessing history.

`navigation.odometry_axes` and `navigation.imu_axes` select fifteen-coordinate
sensor masks independently; masks follow sensor-frame rotation into the model's
frames. Each sensor has a configurable `navigation.<sensor>_queue_size`.
The adapter retains up to `queue_capacity` sensor messages without copying their
payloads. On each publication tick, it converts ready messages in timestamp order;
future timestamps wait for the ROS clock. Reset and reconfiguration clear this queue.
Publication follows the ROS clock, including bag playback speed. With automatic
prediction disabled, a positive `sensor_timeout` enables prediction during gaps;
zero disables that fallback.

Outputs are `odometry/filtered`, optional `accel/filtered`, and optional
world-to-body TF. Corrected duplicate timestamps publish only when
`navigation.permit_corrected_publication=true`.

`navigation.publish_control_state=true` additionally publishes
`~/navigation/layout` (`NumericInterface`) and `~/navigation/state`
(`NumericSample`). This explicit projection matches the shared
`get_navigation_state(resource, dimensions)` layout used by control:

- 2D: position x/y, yaw, body velocity x/y, body yaw rate (6 values).
- 3D: position x/y/z, quaternion x/y/z/w, body linear/angular velocity (13 values).

Remap that state topic into the control state ingress. The full estimator state
on `~/state` retains its own advertised meaning.

`navigation_estimation` configurations include `ekf`, `ukf`, `ekf_2d`, `ukf_2d`,
`ekf_control`, and `ukf_control`. They are workspace examples, not a node-side
filter whitelist. Planar examples preconfigure the required constrained
observation subsets; control examples enable the model's prepared nine-value
control layout. Set `navigation.use_control=true` and choose control axes,
acceleration/deceleration gains and limits to use `cmd_vel`. Optional dynamic
noise applies speed scaling as S Q Sᵀ, preserving cross terms.

Bounded late-measurement replay
-------------------------------

The estimator prepares a bounded event ring and a separate transactional replay
tail. Checkpoints preserve algorithm-specific history, including UKF retained
sigma points. Stale observations replay the affected suffix; failures or gated
new late observations restore the original posterior. Events older than the
retained anchor return `codeStaleInput`.

The ROS node defaults to `replay_execution: ros`. `SharedEstimatorBase` owns an
idle second estimator/model instance, while the script and SignalRouter remain
single. A late sample transfers a checkpoint and retained history to that spare;
the live worker continues, mirroring subsequent operations into a prepared queue.
The ROS executor replays and catches up, then the worker swaps estimator roles
at a matching event sequence. Lifecycle changes quiesce both owners and cancel
pending work. `replay_execution: worker` keeps replay inline in one estimator.
An adopted estimate is published even if simulated time stops; ordinary periodic
publication retains its ROS-time frequency. `history_capacity=0` disables both
history and spare construction. Known-stale samples do not start replay, and
stale samples encountered during replay do not discard earlier repaired updates.
See the [bag benchmark guide](test/robot_localization_parity/README.md#replay-execution-and-role-switching)
for capacity limits, cancellation behavior and timing both execution threads.

The default `replay_noise_policy=reject_split` rejects corrections requiring a
split of an arbitrary discrete Q. Navigation profiles explicitly select
`constant_rate`: a split scales Q by the interval fraction. Inputs and noise
density remain frozen at the values recorded for each original prediction
interval. Replay does not recompute closed-loop control or state-dependent noise
under the corrected trajectory. Use a model-specific replay policy if that is
required by an application.

UKF defaults to retained-sigma correction: it uses propagated sigma points and
their predicted observation covariance. `legacy_ukf_correction=false`
selects the innovation-covariance policy. Filter settings, measurement layouts,
gating thresholds, and model control layout belong to RPP component parameters.

Build and run with colcon
-------------------------

From the ROS workspace root, with RPP and its C++ runtime installed, register
the source libraries on first use:

```sh
source /opt/ros/lyrical/setup.bash
source install/setup.bash
rpp library register src/rpp_dev/src/rpp_interfaces --link
rpp library register src/rpp_libraries/rpp_localization --link
colcon build --packages-select rpp_interfaces rpp_localization --cmake-args \
  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rpp_localization localization.launch.py
```

For already registered libraries, use `rpp library refresh rpp_localization --jobs 4`
after changing plugin code. Refresh shared interface dependencies first when
their schemas or headers change. Rebuilding the ROS node and refreshing its
registered plugins are separate operations.

Select a configured estimator or navigation variant with:

```sh
ros2 launch rpp_localization localization.launch.py configuration:=ukf
ros2 launch rpp_localization localization.launch.py configuration:=ekf_2d two_d_mode:=true
ros2 launch rpp_localization localization.launch.py configuration:=ekf_control use_control:=true
```

Launch arguments also accept `params_file` and `rpp_workspace`. Start from
`params/navigation.yaml` for navigation or `params/estimator.yaml` for numeric
messages only. `estimator_script_node` and `localization_node` run the generic
adapter. `ekf_node`, `ukf_node`, `ekf.launch.py`, and `ukf.launch.py`
select the corresponding navigation configuration. Startup
`activate_on_start` defaults to true.

The library registers `NavigationModel`, `Ekf`, `Ukf`, `ProfiledEkf`, and
`ProfiledUkf`. Its plugin contracts come from `rpp_interfaces`; it declares no
local plugin types. Numerical consumers link `rpp_localization::numerics`
without the ROS library.

Tests and measurements
-----------------------

`BUILD_TESTING` enables generic numerical, cross-translation-unit, and Python
parity-metric tests. The Python cases check interpolation, angular wrapping,
missing overlap, summaries, and GNSS alignment without starting ROS.
`RPP_LOCALIZATION_ENABLE_PLUGIN_TESTS` adds registered estimator/model and
threaded ROS node tests. The source library must be registered before building
those targets.

`RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS` adds a generic-node navigation
bag test. `RPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS` compares the generic stack
with robot_localization on the six EKF/UKF bag cases.
`RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS` enables the optional timing and
worker allocation measurements and the instrumented replay-counter tests.
CI enables this flag alongside the plugin and navigation bag tests.

See [bag benchmark instructions](test/robot_localization_parity/README.md)
for commands, replay settings, CPU/memory columns, and saved trajectory reports.
