# Localization bag comparison

## Results

robot_localization 3.10.0; candidate: generic; bag rate: 1.0; replay (all implementations): True.

| Bag | Filter | End error RL / candidate (tol.) [m] | Pos. diff RMS [m] | Yaw diff RMS [deg] | Rate RL / candidate [Hz] | CPU RL / candidate [s] | New worker CPU [s] | New worker allocation calls | New worker peak live heap [KiB] |
|---|---|---|---|---|---|---|---|---|---|
| test2 | EKF | 0.011 / 0.011 (0.14) | 0.0001 | 0.00 | 49 / 49 | 5.52 / 6.25 | 0.545 | 0 | 0.00 |
| test2 | UKF | 0.011 / 0.009 (0.24) | 0.0071 | 0.00 | 48 / 48 | 5.90 / 6.87 | 0.725 | 0 | 0.00 |
| test3 | EKF | 0.057 / 0.057 (0.13) | 0.0026 | 0.02 | 29 / 29 | 14.03 / 15.32 | 1.245 | 0 | 0.00 |
| test3 | UKF | 0.041 / 0.041 (0.17) | 0.0051 | 0.02 | 29 / 29 | 14.24 / 16.60 | 2.123 | 0 | 0.00 |

### Previous implementation versus generic stack

| Bag | Filter | End error old / new [m] | Old/new pos. RMS [m] | Old/new yaw RMS [deg] | Rate old / new [Hz] | CPU old / new [s] | CPU change | Peak RSS old / new [MiB] | New worker CPU [s] | New worker allocation calls | New worker peak live heap [KiB] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| test2 | EKF | 0.011 / 0.011 | 0.0001 | 0.00 | 49.1 / 49.1 | 6.15 / 6.25 | +1.6% | 71.7 / 97.1 | 0.545 | 0 | 0.00 |
| test2 | UKF | 0.011 / 0.009 | 0.0071 | 0.00 | 48.3 / 48.3 | 6.95 / 6.87 | -1.2% | 78.6 / 104.7 | 0.725 | 0 | 0.00 |
| test3 | EKF | 0.059 / 0.057 | 0.0008 | 0.01 | 29.1 / 29.2 | 16.50 / 15.32 | -7.2% | 73.2 / 97.6 | 1.245 | 0 | 0.00 |
| test3 | UKF | 0.041 / 0.041 | 0.0028 | 0.01 | 29.1 / 29.2 | 20.12 / 16.60 | -17.5% | 76.9 / 105.4 | 2.123 | 0 | 0.00 |

Positive CPU change means the generic stack used more process CPU time. CPU includes all worker threads during playback and the drain window; RSS is the process lifetime peak, including startup.

Direct old/new errors use robot_localization timestamps within the shared overlap, matching the original accuracy comparisons. Saved test outcomes are unchanged.

test2 EKF: on the legacy timestamp grid (including any earlier startup samples), yaw RMS was 0.0010° and maximum difference was 0.0323°.

test2 UKF: on the legacy timestamp grid (including any earlier startup samples), yaw RMS was 0.0037° and maximum difference was 0.0744°.

test3 EKF: on the legacy timestamp grid (including any earlier startup samples), yaw RMS was 0.0038° and maximum difference was 0.1318°.

test3 UKF: on the legacy timestamp grid (including any earlier startup samples), yaw RMS was 0.0111° and maximum difference was 0.5179°.

### Estimator execution scopes

CPU is execution-thread time inside the scope. Wall time includes preemption. Worker batches include live prediction/correction, history, handoff and estimate copies. In ROS replay mode, the separate replay scope includes historical replay and catch-up on the ROS executor. The estimator CPU comparison adds worker batch CPU and ROS replay CPU. Legacy replay stays inside its batch. ROS sensor conversion, publication and blocked waits are outside. The legacy cold initialization batch is excluded. Scopes are nested: do not sum batch and operation CPU.

| Bag | Filter | Implementation | Scope | Calls | CPU [s] | CPU mean [µs] | Wall p99 upper [µs] | Wall max [µs] |
|---|---|---|---|---|---|---|---|---|
| test2 | EKF | new | estimation | 6828 | 0.4089 | 59.88 | 294.91 | 601.91 |
| test2 | EKF | new | predict | 5530 | 0.1513 | 27.36 | 114.69 | 214.83 |
| test2 | EKF | new | correct | 12239 | 0.1625 | 13.28 | 53.25 | 217.23 |
| test2 | EKF | new | replay | 590 | 0.0756 | 128.19 | 442.37 | 1002.29 |
| test2 | EKF | old | estimation | 3059 | 0.6170 | 201.71 | 622.59 | 1040.33 |
| test2 | EKF | old | predict | 6710 | 0.2350 | 35.02 | 155.65 | 290.51 |
| test2 | EKF | old | correct | 14012 | 0.2336 | 16.67 | 65.53 | 458.45 |
| test2 | UKF | new | estimation | 6712 | 0.5859 | 87.30 | 393.21 | 2270.82 |
| test2 | UKF | new | predict | 5589 | 0.2602 | 46.56 | 139.26 | 1041.90 |
| test2 | UKF | new | correct | 12239 | 0.2275 | 18.59 | 73.73 | 2033.14 |
| test2 | UKF | new | replay | 530 | 0.1204 | 227.21 | 622.59 | 923.86 |
| test2 | UKF | old | estimation | 3049 | 1.3388 | 439.10 | 1703.93 | 8255.38 |
| test2 | UKF | old | predict | 6635 | 0.6885 | 103.77 | 311.30 | 5873.56 |
| test2 | UKF | old | correct | 13807 | 0.3689 | 26.71 | 94.21 | 6546.87 |
| test3 | EKF | new | estimation | 9707 | 1.0749 | 110.74 | 589.82 | 5034.50 |
| test3 | EKF | new | predict | 35572 | 0.4049 | 11.38 | 51.20 | 582.94 |
| test3 | EKF | new | correct | 89109 | 0.4770 | 5.35 | 20.48 | 2779.55 |
| test3 | EKF | new | replay | 2386 | 0.8920 | 373.83 | 1179.65 | 3434.87 |
| test3 | EKF | old | estimation | 3690 | 2.7543 | 746.42 | 2359.30 | 10730.53 |
| test3 | EKF | old | predict | 53572 | 0.8088 | 15.10 | 57.34 | 3860.14 |
| test3 | EKF | old | correct | 107099 | 1.1523 | 10.76 | 30.72 | 6308.91 |
| test3 | UKF | new | estimation | 8818 | 1.9733 | 223.78 | 1179.65 | 3985.47 |
| test3 | UKF | new | predict | 35543 | 1.0507 | 29.56 | 69.63 | 2286.57 |
| test3 | UKF | new | correct | 89103 | 0.7395 | 8.30 | 27.65 | 2566.14 |
| test3 | UKF | new | replay | 2233 | 2.0878 | 934.96 | 2228.22 | 4457.47 |
| test3 | UKF | old | estimation | 3690 | 7.3434 | 1990.07 | 6029.31 | 18092.19 |
| test3 | UKF | old | predict | 53547 | 3.9597 | 73.95 | 172.03 | 3623.84 |
| test3 | UKF | old | correct | 107047 | 1.8847 | 17.61 | 51.20 | 3264.95 |

test2 EKF: aggregate estimation batch CPU change -21.5% (worker batch plus ROS replay; positive means the new stack used more).
test2 EKF new: 0 batches exceeded the 20.000 ms execution budget.
test2 EKF new: numerical worker used 0.540 s CPU in the playback/drain window (Linux thread ticks).
test2 EKF new: queue wait p99 upper 311.30 µs, maximum 952.22 µs (12239 events).
test2 EKF old: 0 batches exceeded the 20.000 ms execution budget.
test2 UKF: aggregate estimation batch CPU change -47.2% (worker batch plus ROS replay; positive means the new stack used more).
test2 UKF new: 0 batches exceeded the 20.000 ms execution budget.
test2 UKF new: numerical worker used 0.710 s CPU in the playback/drain window (Linux thread ticks).
test2 UKF new: queue wait p99 upper 589.82 µs, maximum 3071.20 µs (12239 events).
test2 UKF old: 0 batches exceeded the 20.000 ms execution budget.
test3 EKF: aggregate estimation batch CPU change -28.6% (worker batch plus ROS replay; positive means the new stack used more).
test3 EKF new: 0 batches exceeded the 33.333 ms execution budget.
test3 EKF new: numerical worker used 1.240 s CPU in the playback/drain window (Linux thread ticks).
test3 EKF new: queue wait p99 upper 655.36 µs, maximum 4958.18 µs (89109 events).
test3 EKF old: 0 batches exceeded the 33.333 ms execution budget.
test3 UKF: aggregate estimation batch CPU change -44.7% (worker batch plus ROS replay; positive means the new stack used more).
test3 UKF new: 0 batches exceeded the 33.333 ms execution budget.
test3 UKF new: numerical worker used 2.120 s CPU in the playback/drain window (Linux thread ticks).
test3 UKF new: queue wait p99 upper 1114.11 µs, maximum 4720.96 µs (89103 events).
test3 UKF old: 0 batches exceeded the 33.333 ms execution budget.

Quantiles are fixed-histogram upper bounds (at most 6.25% above 16 ns); maxima and totals are exact clock measurements. Recording uses prepared counters, with serialization only in the stats service. The clocks and nested scopes add measurement overhead to both implementations.

Compare worker batch plus ROS replay CPU for the estimator workload and inspect operation counts: generic worker mode replays inside correction, generic ROS mode uses its separate replay scope, and legacy replay calls predict/correct again. Per-call averages and batch distributions therefore include different work mixes and scheduling. Raw new-worker CPU is separate from scoped CPU and cannot be compared directly with an entire old ROS executor. Budget overruns are execution durations, not scheduler deadline misses. Use bag rate 1.0 for real-time load, and repeat isolated runs to assess CPU contention.


### Generic internal correction and replay workload

| Bag | Filter | Core predict / correct | Projected corrections | Late samples | Replay success / failure | Replayed predict / correct | Tail events mean / max | Rollback span mean / max [ms] | Covariance diagonal / cache / LDLT |
|---|---|---|---|---|---|---|---|---|---|
| test2 | EKF | 7,588 / 15,957 | 15,957 | 590 | 590 / 0 | 1,180 / 1,770 | 4.00 / 4 | 20.000 / 20.000 | 21,189 / 0 / 0 |
| test2 | UKF | 7,417 / 15,502 | 15,502 | 531 | 529 / 0 | 1,058 / 1,587 | 4.00 / 4 | 20.000 / 20.000 | 20,807 / 0 / 0 |
| test3 | EKF | 78,724 / 157,323 | 157,323 | 18,044 | 2,362 / 0 | 18,017 / 17,960 | 11.42 / 45 | 16.746 / 131.435 | 218,110 / 0 / 0 |
| test3 | UKF | 80,870 / 161,544 | 161,544 | 18,094 | 1,629 / 0 | 13,684 / 13,555 | 12.52 / 45 | 18.765 / 130.475 | 231,073 / 0 / 0 |

| Bag | Filter | Replay jobs started | Role switches | Jobs cancelled | Stale replay samples | Replay pending |
|---|---|---|---|---|---|---|
| test2 | EKF | 590 | 590 | 0 | 0 | False |
| test2 | UKF | 529 | 529 | 0 | 0 | False |
| test3 | EKF | 2,356 | 2,356 | 0 | 0 | False |
| test3 | UKF | 1,537 | 1,537 | 0 | 0 | False |

Core counts aggregate both physical estimator instances in ROS replay mode, including live work, catch-up and attempted calls inside historical replay, including gated or failed work. Replayed predictions include split intervals; replayed corrections exclude the newly arrived sample. Tail length counts retained events after the insertion point; rollback span reaches the restored checkpoint. Failed replay work remains counted after the posterior is restored. Covariance counters cover boundary validation, including initialization; they exclude the required innovation LDLT and UKF sigma-point factorization. Legacy interface call counts remain in the scope table. Older results without these counters cannot reconstruct internal replay work.


### Replay history statistics

| Bag | Filter | Implementation | Replay jobs / legacy batches | Internal rewinds | Mean replay states | Std. dev. | Max replay states |
|---|---|---|---|---|---|---|---|
| test2 | EKF | new | 590 | 590 | 4.00 | 0.00 | 4 |
| test2 | EKF | old | 591 | 591 | 1.00 | 0.00 | 1 |
| test2 | UKF | new | 529 | 529 | 4.00 | 0.00 | 4 |
| test2 | UKF | old | 536 | 536 | 1.00 | 0.00 | 1 |
| test3 | EKF | new | 2,356 | 2,362 | 11.42 | 6.69 | 45 |
| test3 | EKF | old | 2,359 | 2,359 | 3.82 | 2.23 | 15 |
| test3 | UKF | new | 1,537 | 1,629 | 12.52 | 7.00 | 45 |
| test3 | UKF | old | 2,360 | 2,360 | 3.82 | 2.20 | 15 |

Replay jobs count generic background catch-up sessions; legacy batches count rollback calls. Internal rewinds count generic historical replay attempts inside those jobs. A bounded batch of queued corrections shares one rewind; later arrivals or another service batch can require more. Legacy performs one rollback attempt per replay batch, so its two counts match. Generic worker-mode jobs are not recorded and appear as —. One history sample is the saved-history tail selected for an internal rewind, including failed attempts. The mean and population standard deviation use all recorded attempts; zero-length tails count too. Generic checkpoints follow prediction/correction events; legacy checkpoints follow measurement timestamp batches. These are history-record counts, not equal units of numerical work. Old recordings show available count/mean/max data and — for missing statistics; standard deviation cannot be recovered from totals and maxima.


![test2 ekf](run0_test2_ekf.png)

![test2 ukf](run0_test2_ukf.png)

![test3 ekf](run0_test3_ekf.png)

![test3 ukf](run0_test3_ukf.png)

Process CPU and RSS describe the complete ROS processes. The new worker columns cover only rpp_estimator, from worker entry through the paused stats snapshot, including live prediction, correction and role adoption. ROS replay and catch-up are outside that worker; inline worker-mode replay is included. Process CPU covers playback/drain instead. Allocation calls include malloc/calloc/realloc and aligned allocation requests, including failed requests. Peak live heap counts requested bytes allocated by that worker, excluding buffers prepared on other threads, shared memory and stack. Worker metrics require PARITY_PROFILE=1; allocator bookkeeping adds measurement overhead. Missing worker measurements in unprofiled or older runs are shown as —. Side-by-side runs share CPU and DDS resources; repeat isolated legacy/generic runs for timing confirmation.
