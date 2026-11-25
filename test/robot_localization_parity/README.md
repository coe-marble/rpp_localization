# robot_localization parity

Runs rpp_localization and robot_localization side by side on robot_localization's bag tests
(`test1` Husky, `test2` Jackal, `test3` AR.Drone) with the upstream configs, for EKF and UKF.

Results: [results/report.md](results/report.md), with the defaults below.

- EKF: same estimate as robot_localization (4–6 mm RMS).
- UKF: 5 to 24 cm from robot_localization in that report. The gap comes from the run, not the math:
  robot_localization does not replay late measurements by default, and at full bag speed message
  timing changes from run to run, so robot_localization's own UKF differs by up to 16 cm between
  two runs. See [Fair comparison](#fair-comparison).
- CPU: compare only with `PARITY_RL_REPLAY=1`, where both sides replay late measurements.

## Run

```bash
PARITY_RESULTS_DIR=/tmp/parity launch_test test_robot_localization_parity.launch.py
python3 parity_report.py /tmp/parity --output results
```

`PARITY_CASES` selects cases (e.g. `test2:ekf,test3:ukf`). `test1` needs robot_localization ≥ 3.11.
A full run takes about 20 minutes.

`PARITY_RL_REPLAY=1` starts robot_localization with `smooth_lagged_data`, so it replays late
measurements as rpp_localization always does. `PARITY_BAG_RATE` plays the bags slower; the wait
for the bag scales with it.

## Fair comparison

```bash
PARITY_RL_REPLAY=1 PARITY_BAG_RATE=0.5 PARITY_CASES=test1:ukf,test2:ukf,test3:ukf \
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
