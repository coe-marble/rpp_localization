# EKF vs UKF vs InEKF

Runs each rpp_localization filter alone on robot_localization's bag tests (`test1` Husky,
`test2` Jackal, `test3` AR.Drone) and compares them.

Results: [results/report.md](results/report.md)

- With the same sensors (`common`) the InEKF ends 0.39 / 0.017 / 0.15 m from the end point on
  `test1` / `test2` / `test3`, against 0.23 / 0.011 / 0.33 m for the EKF. The UKF on develop
  (without #22) diverges on `test3` once it uses the drone's accelerometer.
- The InEKF uses the least CPU on every bag: 20 / 3.6 / 10 s, against 26 / 4.2 / 24 s for the EKF
  and 28 / 5.8 / 49 s for the UKF.
- Against GNSS on `test1` all three drift alike (about 45 m at 300 s): the heading drift of this
  bag's IMU and odometry is shared by every filter, and none of them fuses GNSS.
- None of the bags has ground truth. The end point is where robot_localization's EKF ended in the
  original test, so it favours filters that behave like that EKF.

## Data

| Bag | Robot | IMU | Odometry | Other |
|---|---|---|---|---|
| `test1` | Husky, 395 s | `/imu/data`, with orientation, mounted rotated in `imu_link` | wheel odometry | GNSS `/fix`, not fused |
| `test2` | Jackal, 61 s | `/imu/data`, with orientation | wheel odometry | |
| `test3` | AR.Drone, 123 s | `/ardrone/imu`, with orientation | the drone's velocity and height estimate | |

## Setups

`BENCHMARK_SETUP` selects the sensors:

| Setup | EKF and UKF | InEKF |
|---|---|---|
| `upstream` | upstream bag configs and process noise, as in [the parity test](../robot_localization_parity/README.md) | IMU, odometry velocity, and odometry height on `test3` |
| `common` | the InEKF's sensors: IMU orientation, rates and acceleration, odometry velocity, and odometry height on `test3` | as in `upstream` |

What `common` changes for the EKF and UKF against `upstream`:

| Bag | Added | Removed |
|---|---|---|
| `test1` | IMU yaw rate | |
| `test2` | | odometry yaw rate |
| `test3` | IMU acceleration, odometry vertical velocity | odometry yaw rate |

The InEKF runs with its default noise. The EKF and UKF keep the upstream process noise in every setup.

## InEKF noise tuning

The default `gyro_noise` of `ImuInertialModel` comes from this benchmark. End error of InEKF
variants with common sensors:

| Variant | End error [m] (test1 / test2 / test3) | Yaw diff to EKF [deg] (test1 / test3) |
|---|---|---|
| EKF, for reference | 0.23 / 0.011 / 0.33 | 0 / 0 |
| previous default | 1.56 / 0.013 / 0.27 | 2.90 / 0.99 |
| `accel_noise` 0.5 | 1.57 / 0.020 / 0.23 | 2.90 / 0.98 |
| `accel_noise` 0.5, `gyro_noise` 0.005 | 0.42 / 0.018 / 0.19 | 0.78 / 0.33 |
| `accel_noise` 0.5, `accel_bias_noise` 1e-3 | 1.58 / 0.020 / 0.23 | 2.90 / 0.98 |
| `gyro_noise` 0.005 (new default) | 0.39 / 0.013 / 0.14 | 0.78 / 0.34 |

With `gyro_noise` 0.001 the InEKF trusted the gyroscope over the IMU's absolute heading, and its
heading drifted from the EKF's by up to 5 degrees in turns. At 0.005 it ends closest to the end point
on `test1` and `test3` with no change on `test2`. `accel_noise` and `accel_bias_noise` have little
effect on these bags.

Tuning runs use `BENCHMARK_INEKF_NOISE`, e.g. `BENCHMARK_INEKF_NOISE='{"gyro_noise": [0.005, 0.005, 0.005]}'`.

## Metrics

- End error per axis: to the upstream end point.
- Difference per axis and in yaw between each pair of filters, over the whole run.
- GNSS drift (`test1` only, GNSS not fused): each track is aligned to the GNSS fixes on the first
  60 s only, then the horizontal error is reported as it grows. A fit over the whole run would
  absorb the heading drift.
- CPU: user plus system time of the filter process over the whole bag.

## Run

```bash
for setup in upstream common; do
  BENCHMARK_SETUP=$setup BENCHMARK_RESULTS_DIR=/tmp/benchmark/$setup \
    launch_test test_filter_benchmark.launch.py
done
python3 benchmark_report.py --upstream /tmp/benchmark/upstream --common /tmp/benchmark/common \
  --output results
```

`BENCHMARK_CASES` selects cases (e.g. `test2:ekf,test2:inekf`). Each setup takes about 30 minutes.
The machine must not sleep during a run, and no other bag may play on the same ROS graph.
