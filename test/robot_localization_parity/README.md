# Localization bag benchmark and robot_localization parity

Runs rpp_localization and robot_localization side by side on robot_localization's bag tests
(`test1` Husky, `test2` Jackal, `test3` AR.Drone) with the upstream configs, for EKF and UKF.

Historical results from the previous fixed-size implementation: [results/report.md](results/report.md).
The figures below describe those historical runs; new runs save their results separately.

- EKF: same estimate as robot_localization (4–6 mm RMS).
- UKF: 5 to 24 cm from robot_localization in that report. The gap comes from the run, not the math:
  robot_localization does not replay late measurements by default, and at full bag speed message
  timing changes from run to run, so robot_localization's own UKF differs by up to 16 cm between
  two runs. See [Fair comparison](#fair-comparison).
- CPU: use the same `PARITY_RL_REPLAY` setting for every run being compared.

## Compare the generic stack with the previous implementation

The same six cases, source configs, bags, static IMU transform, final-position
limits, position RMS limits and yaw RMS limits are reused. Nothing in `config/`
or the recorded historical results is changed. The default
`PARITY_IMPLEMENTATION=both` launches the previous `legacy_localization_node`,
the generic `localization_node`, and `robot_localization` in separate namespaces.
`generic` and `legacy` modes run one RPP implementation beside the reference.

`PARITY_RL_REPLAY` controls all implementations, including the reference:

- `1`: enables `smooth_lagged_data` with a one-second history for legacy RPP and
  `robot_localization`, and sets generic estimator history capacity to 512 records.
- `0` (default): explicitly disables smoothing and sets history length to zero
  for legacy RPP and `robot_localization`; generic history capacity is zero,
  which also disables construction of the spare estimator.

The benchmark applies these overrides to its temporary configuration copy,
including when profiling replaces generic estimators with instrumented plugins.
Source and installed profiles are left untouched. Results record `replay_enabled`
for all implementations; older results only recorded the reference setting.

### Replay execution and role switching

The generic node defaults to `replay_execution: ros`. One script and one
SignalRouter address one logical estimator. `SharedEstimatorBase` constructs a
second estimator component from the same profile, with an independent model
subtree. Both are configured/initialized/activated together; the spare performs
no filtering work until a late measurement triggers replay.

Each replay service processes at most `replay_events_per_tick` queued events,
borrowing bounded batches and merging them with retained history in timestamp
order. Pending corrections share
one rewind to the earliest affected checkpoint instead of replaying the same
tail separately for every correction. Equal-time queued events retain arrival
order; retained events precede new events at the same timestamp. Preparation
allocates the queue views, sort indices and checkpoints; servicing does not
allocate. The candidate becomes ready only after the complete batch commits.
Stale or unsupported insertions are skipped, and a gated late correction leaves
no artificial prediction split. Other failures restore the candidate's prior
posterior and cancel the job, preserving the live estimator.

On that trigger, the current checkpoint (including algorithm caches) is copied
into the spare and the retained history bank changes owners. The live instance
continues with a fresh prepared history bank. Predictions and measurements are
then queued to both instances; the live one defers late corrections to replay.
A ROS wall timer services a bounded number of queued events, including after
bag playback stops advancing simulated time. Once both have consumed the same
event sequence, the numeric worker changes the running-instance pointer at an
execution boundary. The corrected instance keeps its model and replay history;
the previous live instance becomes idle. No lifecycle transition occurs during
this switch. Fork/adoption helpers are non-virtual methods on SharedEstimatorBase.
Replay readiness wakes the worker; it also checks for adoption between ingress
operations. Its cached estimate is refreshed before state-dependent prediction
inputs are evaluated. If the estimate snapshot pool is exhausted, execution
retries on its next tick instead of predicting from an older cached posterior.

The ROS wall timer also flushes an adopted estimate when simulated time has
stopped. Normal periodic publication keeps its configured ROS-time frequency;
a corrected snapshot can be published between those regular timer callbacks.
Active estimator reloads first deactivate the old instance, then rebuild routes
and initialize both new physical instances before activation. Instance identity
is rechecked after slot commands, including failed commands, so released native
callbacks cannot resume against the old estimator.

Reset, disable, lifecycle changes and cleanup quiesce execution and cancel
pending replay. Failed replay or a full replay queue leaves the live posterior
in place and reports an error; queued replay work is discarded. Following
cancellation, the live history starts at the last fork checkpoint, so older
late samples can be rejected as stale. Configure `replay_queue_capacity` (default
4096) and `replay_events_per_tick` (default 64) for the workload. Each historical
replay call itself is bounded by retained history, but is not preempted mid-call.
Known-stale samples are rejected before a fork. If an additional queued sample
is too old or requires an unsupported noise split, replay skips that sample and
keeps corrections already completed by the candidate. Profiling records these
skipped samples as `replay_stale_corrections`. Other replay failures still cancel
the candidate. Setting the estimator's `history_capacity` to 0 disables history
and spare construction in either execution mode.

Set `PARITY_REPLAY_EXECUTION=worker` to benchmark the previous inline replay path
with a single estimator instance. The default is `PARITY_REPLAY_EXECUTION=ros`.
When comparing these modes, use total process CPU or worker estimation CPU plus
the separate ROS `replay` CPU scope. Worker CPU alone now excludes offloaded work.
Metrics aggregate both physical instances so role switches retain their totals.
The spare is built from the persisted estimator/model profile. Native callers
using builder parameter overrides must arrange an identical profile for the
spare; the current RPP context API does not export a complete resolved tree of
parameters for cloning. The ROS node and bag fixtures use persisted profiles.

### Replay statistics

With `PARITY_PROFILE=1`, both legacy and generic snapshots contain
`estimation_timing.<implementation>.replay_statistics` with `count`,
`states_mean`, `states_stddev` and `states_max`. The report shows a separate
**Replay history statistics** table for old and new. Standard deviation is the
population standard deviation, accumulated online without allocations; failed
attempts and zero-length tails are included.

The same row shows **Replay jobs / legacy batches** and **Internal rewinds**.
Generic jobs are background sessions; rewinds are actual historical replays
within them. A service batch shares one rewind, while later arrivals can require
another batch in the same job. Legacy performs one rollback attempt per replay
batch, so its two counts match. Generic inline worker mode has no background
jobs. Unavailable counts appear as `—`, rather than zero.

“Replay states” counts saved history records after the rollback point. Generic
history saves a checkpoint per prediction/correction event; legacy history saves
at measurement timestamp batches. These counts therefore describe each history
representation rather than identical numerical operations. Existing saved runs
without these moments show unavailable statistics; standard deviation cannot be
reconstructed from a sum and maximum alone.

The generic adapter receives an explicit translation of each case's sensor
masks, topics, queue depths, gravity setting, frames, process noise, initial
covariance, frequency and sensor timeout. Temporary RPP workspace copies carry
model-frame and UKF-parameter overrides; source and installed workspaces are
left untouched. Original per-sensor masks rotate with their sensor frames,
including the rotated IMU in `test1`. The native navigation algorithms and
preprocessing are still different implementations: this comparison measures
those differences instead of assuming numerical parity or relaxing limits.

Navigation profiles use RPP's canonical plugin folders:
`.rppws/parts/rpp_localization__ekf/<uuid>` and
`.rppws/parts/rpp_localization__ukf/<uuid>`. After updating profiles,
rebuild localization and source the workspace so their installed copies are
current. The benchmark checks selected profiles before launching processes and
fails promptly if a filter exits, instead of replaying a full bag without it.
Unsupported `test1` runs on robot_localization <3.11 are skipped before any
filter starts; `test2` and `test3` continue to use their original checks.

Playback starts paused at the bag's first timestamp. The test waits for the
initial `/clock`, every filter's clock and sensor subscriptions, and its output
publisher before resuming. DDS discovery finishes while simulated time stays
fixed, so slow startup cannot discard the opening measurements. Missing
endpoints fail startup with their names after 30 seconds. Process CPU baselines
are taken immediately before resume, excluding this startup wait.

After building/installing `rpp_ros_interfaces`, registering current shared RPP
interfaces and localization plugins, and sourcing the workspace (see the main
README), from the colcon workspace root:

```bash
colcon build --packages-select rpp_localization --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS=ON
source install/setup.bash

PARITY_IMPLEMENTATION=both PARITY_RL_REPLAY=1 PARITY_BAG_RATE=0.5 \
  PARITY_RESULTS_DIR=/tmp/localization-bags \
  colcon test --packages-select rpp_localization \
    --event-handlers console_direct+ --return-code-on-test-failure \
    --ctest-args -R robot_localization_parity --output-on-failure

python3 src/rpp_libraries/rpp_localization/test/robot_localization_parity/parity_report.py \
  /tmp/localization-bags --output /tmp/localization-bag-report
```

If localization reports that `EstimatorLayout` has no `state_interfaces` member,
the installed generated ROS message headers are older than the source schema.
Rebuild the message package explicitly before retrying localization; registering
RPP plugins does not regenerate ROS messages:

```bash
colcon build --packages-select rpp_ros_interfaces --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
colcon build --packages-select rpp_localization
```

The comparison needs `robot_localization`, `launch_testing`, `rosbag2` with MCAP
support, NumPy and PyYAML. Plots also need Matplotlib; use `--no-plots` to produce
only the table without it. `test1` requires robot_localization ≥ 3.11, as before.

Use `PARITY_CASES=test2:ekf` for one case, or a comma-separated list for a subset.
The original direct launch command also works from this directory:

```bash
PARITY_IMPLEMENTATION=both PARITY_RL_REPLAY=1 PARITY_BAG_RATE=0.5 \
  PARITY_RESULTS_DIR=/tmp/localization-bags \
  launch_test test_robot_localization_parity.launch.py
```

A full-speed six-case run previously took about 20 minutes; half-speed playback
requires longer. Use fresh result directories for separate runs. Benchmark
registration, builds, playback and tests have not been run during this change.

## Measurements

The generated JSON and report include:

- Original final-position, position/yaw/velocity differences and output rates.
- Direct legacy-to-generic trajectory differences alongside the reference.
- Process CPU seconds by exact launch PID, including all threads, measured from
  bag playback start through the output drain; process startup is excluded.
- Process lifetime peak RSS, including startup and prepared matrices/history.
- With `PARITY_PROFILE=1`, separate columns for the new generic worker's CPU
  seconds, allocation calls, and peak live heap usage in KiB.
- Effective node parameters and whether each process survived playback.

Worker metrics are collected only when the test opts in with `PARITY_PROFILE=1`.
The test then enables `benchmark.enabled` and preloads its GNU libc allocation
tracker into the new generic node only. Normal launches and accuracy runs leave
worker tracking and the stats service disabled. The tracker requires an
empty `LD_PRELOAD` environment and is built when both the bag benchmark and
benchmark metrics build options are enabled.
It uses a fixed table for up to 65,536 live allocations, without allocating its
own bookkeeping. Exceeding that capacity fails the metric check and makes the
heap column unavailable instead of publishing an incomplete value.

All three worker metrics cover worker entry through the paused stats snapshot
after output drain, including live prediction, correction and role adoption.
Historical replay and catch-up run on the ROS executor in `ros` mode and are
excluded from these worker metrics; `worker` mode includes inline replay. This differs
from the process CPU window, which starts at bag playback. CPU comes from that
worker's pthread CPU clock. Allocation calls count `malloc`, `calloc`, `realloc`
and aligned allocation requests, including failures; this includes C++ `new`
and Eigen allocations through GNU libc, but excludes custom allocator pools and
direct `mmap`. Heap usage means peak live requested bytes allocated by the worker,
with frees accounted for even on other threads. It excludes allocator overhead,
stack, shared memory, and buffers prepared on other threads; it is not RSS.
Allocation bookkeeping adds CPU overhead and static tracker storage to process
RSS. Runs without profiling and old saved results do not contain these
measurements and show `—` in those columns.

All accuracy assertions use `robot_localization` timestamps within each pair's
shared overlap, as the original reference comparison did. The additional direct
legacy/generic comparison interpolates both tracks onto that same clock. This
avoids comparing an early published prior in only one pair, before the reference
has produced its first estimate. Limits are unchanged. The original comparison
on legacy timestamps is retained as `legacy_timeline_difference`, including any
earlier startup samples; raw tracks retain every recorded sample.

`parity_report.py` can recompute this direct comparison from already saved
three-way tracks. Rerunning the report needs no build or bag replay and does not
change the saved test outcome. The report shows the startup-inclusive diagnostic
alongside the comparison on reference timestamps.

A positive CPU percentage means the generic stack used more CPU than the legacy
stack. These are complete ROS-process costs, including conversion, scheduling,
publication and replay. They are not numerical-update latency or allocation
counts. Old reports measured CPU from process startup and are preserved as
historical results; compare new measurements collected in the same window.

Side-by-side runs share CPU and DDS resources. For timing confirmation, repeat
with `PARITY_IMPLEMENTATION=legacy` and `PARITY_IMPLEMENTATION=generic` in separate
runs, at identical bag rates and replay settings, then pass both directories to
`parity_report.py`. Use `PARITY_RL_REPLAY=1` to compare with replay enabled or
`0` to compare with replay disabled for all implementations. The default is `0`.
Older runs only applied this switch to the reference, so their workloads differ.
Slow playback reduces delivery-order variation; it does not make
message arrival deterministic or establish allocation-free execution.

## Numerical worker and previous estimator timing

Build with `RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=ON`, then set
`PARITY_PROFILE=1` only for the benchmark test that needs metrics. The build flag
defaults to OFF; the runtime flag also defaults to OFF. A profiling request
against an uninstrumented node fails during startup.

Build and source the node, then register the updated localization plugins so
they compile against the current installed headers. Registration compiles
plugins separately from colcon. `EkfBase<CollectMetrics>` and
`UkfBase<CollectMetrics>` forward their compile-time policy into
`SharedEstimatorBase<CollectMetrics>`, which owns the counters, replay and
validation policy in the estimator's single inheritance chain.
`PARITY_PROFILE=1` selects `ProfiledEkf` or `ProfiledUkf`
only in the benchmark's temporary workspace, updating the matching canonical
plugin folders. Source/installed profiles keep `Ekf`/`Ukf`
with counters discarded by `if constexpr`. No runtime policy switch or
estimator reconstruction is needed. Metrics snapshots are read from the common
shared estimator base, without an additional provider interface.

Rebuild the node and refresh the registered localization plugins together after
changing native estimator or replay headers. The native binding rejects older
shared-estimator ABIs at startup; rebuilding with colcon alone does not rebuild
the registered plugin libraries. This does not change the Cap'n Proto contract.

```bash
colcon build --packages-select rpp_localization --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS=ON \
  -DRPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=ON
source install/setup.bash
rpp library register src/rpp_libraries/rpp_localization --link
rpp library refresh rpp_localization --jobs 4

PARITY_PROFILE=1 PARITY_IMPLEMENTATION=both PARITY_RL_REPLAY=1 \
  PARITY_BAG_RATE=1.0 PARITY_CASES=test3:ekf \
  PARITY_RESULTS_DIR=/tmp/localization-worker-timing \
  colcon test --packages-select rpp_localization \
    --event-handlers console_direct+ --return-code-on-test-failure \
    --ctest-args -R robot_localization_parity --output-on-failure

python3 src/rpp_libraries/rpp_localization/test/robot_localization_parity/parity_report.py \
  /tmp/localization-worker-timing --output /tmp/localization-worker-report --no-plots
```

The report adds execution-thread CPU and wall-time distributions for:

- **Estimation batches:** the new worker's active batch includes live
  prediction/correction, history, role adoption and estimate copies. The legacy
  `integrate_measurements` batch also includes its inline historical replay.
  Sensor callbacks, TF conversion, ROS publication and blocked waits are outside.
  The legacy cold initialization batch is excluded.
- **ROS replay:** historical replay and catch-up on the generic spare estimator.
  This scope is disjoint from the worker batch. The aggregate estimator CPU
  comparison adds worker batch CPU and ROS replay CPU; it excludes predict/correct
  subscopes already nested inside the worker batch. Worker mode has no ROS replay scope.
- **Predict/correct interface calls:** counts, CPU totals/means and wall p50/p99
  upper bounds and exact maxima, saved in JSON. Generic correction calls include
  internal replay in `worker` mode; `ros` mode records that work in the separate
  replay scope. Legacy replay makes additional predict/correct calls, so compare
  aggregate estimator CPU and inspect counts before interpreting averages.
- **Worker diagnostics:** Linux CPU ticks for the named `rpp_estimator` thread,
  excluding executor/DDS/TF threads, plus ingress queue wait p99 and maximum.
  Queue wait starts at enqueue commit and ends at worker consumption; it excludes
  earlier sensor delivery and conversion.
- **Internal generic workload:** actual core predict/correct calls, projected
  corrections, late samples, replay success/failure, replayed operations,
  retained tail lengths and rollback timestamp spans. Failed replay work remains
  counted after rollback. Covariance counters distinguish diagonal checks, exact
  cache hits and fresh boundary LDLT validations; required innovation and UKF
  factorizations remain part of the algorithms and are not included there.

`CLOCK_THREAD_CPUTIME_ID` counts CPU actually consumed by the execution thread;
wall time also reflects preemption. Fixed histograms record without allocation
or locking; quantiles are upper bounds within 6.25% above 16 ns. Totals and
maxima use the original clock samples. JSON serialization happens after output
drain in the private `~/benchmark_stats` Trigger service, with the new worker
paused for the snapshot. Process/worker CPU sampling ends before these requests.
Scoped counters cover initialized execution through that snapshot, rather than
the exact `/proc` sampling endpoints. Missing services or CPU samples fail an
opt-in profiling run after its diagnostics have been saved.

Batch wall times exceeding `1/frequency` are **execution budget overruns**, not
scheduler deadline misses. Neither p99 nor the observed maximum establishes a
worst-case execution bound. Full-speed playback supplies realistic arrival
load; half-speed playback reduces it. Batch sizes and replay counts can differ
because the old executor and the new worker schedule differently. Raw worker
CPU therefore has no directly comparable old-thread number: the old executor
also handles ROS callbacks and publication.

With the build flag OFF, profiling scopes contain no clock reads or histogram
updates. With it ON, `PARITY_PROFILE=1` enables scoped timing, internal counters,
the allocation tracker and the stats service for this benchmark. Ordinary
accuracy tests do not enable them. Rebuild with
`-DRPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=OFF` for the production node.
For timing confirmation, repeat the same case separately with
`PARITY_IMPLEMENTATION=legacy` and `PARITY_IMPLEMENTATION=generic`, at rate 1.0,
using fresh results directories. The external robot_localization package is
not instrumented; its existing whole-process measurements remain separate.

Native models can explicitly declare a state-coordinate observation capability.
EKF then gathers covariance submatrices and applies the sparse factored Joseph
update for narrow observations, keeping dense Joseph products for wide ones;
UKF gathers observed sigma coordinates. Runtime state/observation sizes,
correlated measurement noise and angular residuals are preserved. Other models
retain the general nonlinear path. This capability is local to localization and
does not change the shared schema. Boundary PSD validation skips LDLT for finite
nonnegative diagonal matrices and reuses exact successful correlated inputs;
changed correlated inputs still receive full validation. No input is accepted
by approximate equality to a previous covariance.

## Historical fair comparison (previous implementation)

```bash
PARITY_IMPLEMENTATION=legacy PARITY_RL_REPLAY=1 PARITY_BAG_RATE=0.5 PARITY_CASES=test1:ukf,test2:ukf,test3:ukf \
  PARITY_RESULTS_DIR=/tmp/parity launch_test test_robot_localization_parity.launch.py
```

At half speed both filters process every message in order, and the UKF result repeats between
runs. Position RMS difference on develop (7dd7e65), two runs each:

| Bag | rpp_localization to robot_localization [mm] (run 1 / run 2) | robot_localization run to run [mm] | rpp_localization run to run [mm] |
|---|---|---|---|
| `test1` | 3.4 / 2.1 | 3.2 | 2.7 |
| `test2` | 4.9 / 5.1 | 0.6 | 2.1 |
| `test3` | 4.4 / 4.5 | 0.5 | 1.1 |

With `PARITY_RL_REPLAY=1` at full speed, CPU of rpp_localization divided by robot_localization's,
both from the same run, with the history tail copy (#12); `test3` UKF is the median of 3 runs:

| Bag | EKF | UKF |
|---|---|---|
| `test1` | 1.03 | 1.02 |
| `test2` | 1.04 | 1.12 |
| `test3` | 1.19 | 1.79 |

## Data

`config/` and `../data/` are from [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
(BSD-3-Clause, Charles River Analytics); the bags were converted to MCAP.
