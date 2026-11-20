# EKF vs UKF vs InEKF

Runs each rpp_localization filter alone on robot_localization's bag tests (`test1` Husky,
`test2` Jackal, `test3` AR.Drone) and compares them.

Results: [results/report.md](results/report.md)

- All three filters follow the same track. The InEKF uses the least CPU on every bag: 20 / 4 / 11 s
  against 27 / 5 / 25 s for the EKF and 30 / 6 / 50 s for the UKF (test1 / test2 / test3).
- `test2`: the InEKF ends 13 mm from the end point, the EKF 11 mm and the UKF 66 mm.
- `test1` and `test3`: the InEKF ends 1.55 m and 0.27 m from the end point, outside the upstream
  tolerance, and 0.85 m and 0.30 m RMS from the EKF. It runs with default noise, untuned for these bags.
- The end point is where robot_localization's EKF ended in the original test, not ground truth, so it
  favours filters that behave like that EKF.
- On `test1` the three tracks are 21.2 to 21.4 m RMS from the GNSS fixes. That is the drift of
  odometry and IMU alone, the same for all three, so it does not rank them.
- The UKF is the one on `develop`, without the fixes of #22.

## Setup

- EKF and UKF: `localization_node` with the upstream bag configs and process noise, as in
  [the parity test](../robot_localization_parity/README.md).
- InEKF: `inekf_node` with default noise. It is driven by the IMU, fuses the IMU attitude and
  the odometry body velocity, and on `test3` also the odometry height, which the upstream EKF fuses.
- Accuracy: distance of the end position to the upstream end point. On `test1` also the RMS
  distance to the GNSS fixes after a best-fit rotation and offset; no filter fuses the GNSS.
- CPU: user plus system time of the filter process over the whole bag.

## Run

```bash
BENCHMARK_RESULTS_DIR=/tmp/benchmark launch_test test_filter_benchmark.launch.py
python3 benchmark_report.py /tmp/benchmark --output results
```

`BENCHMARK_CASES` selects cases (e.g. `test2:ekf,test2:inekf`). A full run takes about 30 minutes.
The machine must not sleep during a run.
