# rpp_localization parity and metrics

This benchmark runs rpp_localization beside robot_localization
with the same sensor configuration and recorded input. It covers EKF and UKF
on test1 (Husky), test2 (Jackal), and test3 (AR.Drone).

The recorder compares tracks on robot_localization timestamps. It keeps the
original final-position, position-RMS, and yaw-RMS limits. On robot_localization
versions before 3.11, test1 is skipped because of its rotated-IMU gravity handling.

`test_parity_metrics.py` checks the measurements themselves with synthetic tracks:
timestamp alignment, angular wrapping, missing overlap, final-position summaries,
and GNSS alignment. It runs whenever `BUILD_TESTING=ON`, without launching ROS or
replaying a bag.

## Build and run

From the ROS workspace root, with the source library already registered:

```bash
colcon build --packages-select rpp_localization --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS=ON \
  -DRPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=ON
source install/setup.bash
rpp library refresh rpp_localization --jobs 4

PARITY_PROFILE=1 PARITY_RL_REPLAY=1 \
  PARITY_CASES=test2:ekf,test2:ukf,test3:ekf,test3:ukf \
  PARITY_RESULTS_DIR=/tmp/localization-parity \
  colcon test --packages-select rpp_localization \
    --event-handlers console_direct+ --return-code-on-test-failure \
    --ctest-args -R robot_localization_parity --output-on-failure

python3 src/rpp_libraries/rpp_localization/test/robot_localization_parity/parity_report.py \
  /tmp/localization-parity --output /tmp/localization-parity-report --no-plots
```

Each case prints only its JSON path; the report command prints its Markdown path.
Omit `--no-plots` to create track and position-error figures as well. Measurements
are saved before accuracy and instrumentation assertions, so failed cases retain
their results.

## Run settings

| Variable | Meaning |
| --- | --- |
| `PARITY_CASES` | Comma-separated bag/filter pairs; unset runs all six cases. |
| `PARITY_BAG_RATE` | Playback speed; defaults to 1.0. |
| `PARITY_CLOCK_HZ` | Bag clock publication rate; defaults to 1000. |
| `PARITY_RL_REPLAY` | 1 enables replay in RPP and robot_localization; 0 disables both. Defaults to 0. |
| `PARITY_REPLAY_EXECUTION` | `ros` (default) or `worker` for RPP replay. |
| `PARITY_PROFILE` | 1 collects timing, internal counters, and worker allocation statistics. |
| `PARITY_RESULTS_DIR` | Directory for JSON files and tracks; defaults to `rpp-localization-parity` under the system temporary directory. |

Replay-on sets robot_localization smoothing/history to true/one second and the
RPP estimator history capacity to 512 records. Replay-off sets smoothing/history
to false/zero and RPP history capacity to zero, which also disables its spare
estimator. Overrides apply to a temporary workspace copy, including profiled
plugins. Source and installed configurations are not edited.

## Playback and recording

Playback starts paused. The recorder waits for the initial clock and every
filter's sensor subscriptions/output publisher before resuming, so startup
does not discard the first measurements. CPU sampling starts immediately before
resume and ends after the output drain period.

The JSON contains effective ROS parameters, accuracy summaries, process usage,
and recorded tracks. With profiling enabled it also contains execution timings,
worker metrics, and internal replay counters.

The accuracy checks require finite timestamps, positions, yaw, velocities, and
position variances from both nodes. Profiling checks also require complete worker
heap tracking, matching CPU/wall sample counts, one replay-statistics sample per
internal rewind, completed replay jobs, and no replay work when replay is disabled.

## Replay execution and role switching

With ROS replay, one logical estimator owns two physical estimator/model
instances. A late measurement starts a repair session on the idle instance.
The live worker continues receiving events, and the ROS executor replays the
candidate and catches it up. Adoption happens at a worker boundary when both
instances have consumed the same sequence.

A service processes at most `replay_events_per_tick` events (default 64).
Queued corrections share a rewind within a batch. History, queue views,
sorting storage, and checkpoints are prepared before execution.

`replay_execution=worker` instead repairs the estimate inline. Reset, disable,
and lifecycle operations cancel pending work while execution is paused.
`history_capacity=0` disables replay in either mode.

## Reading the metrics

Process CPU includes the entire ROS process; process peak RSS includes startup.
Worker CPU measures only the
`rpp_estimator` thread. ROS-side replay is outside that worker; add the disjoint
estimation-batch and replay scopes when comparing total estimator work.
Predict/correct scopes are nested inside the estimation scope, so do not add
them again.

Worker allocation columns count allocations made by that thread. Peak live heap
counts requested bytes still owned by its allocations, excluding stack and
buffers prepared elsewhere. Profiling uses an allocator interposer and adds
measurement overhead. Use an empty `LD_PRELOAD` environment for that run.

Replay jobs count background repair sessions. Internal rewinds count historical
replays within those sessions. A later service batch can rewind again in the
same job. Worker mode has no background-job count.

Replay-state statistics describe the saved prediction/correction records after
the rollback point. Mean and population standard deviation include failed
attempts and zero-length tails. Core predict/correct counters also include
catch-up and historical replay work.

Use the same replay setting, input, build profile, and bag rate when comparing
runs. Repeat measurements to account for scheduling differences.
Use a separate results directory for each run to keep different settings apart.
