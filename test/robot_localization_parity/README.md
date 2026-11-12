# robot_localization parity

Runs rpp_localization and robot_localization side by side on robot_localization's bag tests
(`test1` Husky, `test2` Jackal, `test3` AR.Drone) with the upstream configs, for EKF and UKF.

Results: [results/report.md](results/report.md)

- EKF: same estimate as robot_localization (4–6 mm RMS).
- UKF: 5–24 cm from robot_localization, because ours has fixes it lacks (upstream #711, angle
  wrapping, Euler averaging). With robot_localization's UKF math, ours matches it to 4 cm on
  `test1` until a correction gives an invalid covariance at 371 s and rpp_localization exits.
- CPU: rpp_localization uses 2–7× (EKF) and 6–13× (UKF) more; the UKF runs at 6 Hz on `test3`.

## Run

```bash
PARITY_RESULTS_DIR=/tmp/parity launch_test test_robot_localization_parity.launch.py
python3 parity_report.py /tmp/parity --output results
```

`PARITY_CASES` selects cases (e.g. `test2:ekf,test3:ukf`). `test1` needs robot_localization ≥ 3.11.
A full run takes about 20 minutes.

## Data

`config/` and `../data/` are from [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
(BSD-3-Clause, Charles River Analytics); the bags were converted to MCAP.
