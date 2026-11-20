from __future__ import annotations


class ComponentParameters:
    gravity = 9.80665
    gyro_noise = [0.005, 0.005, 0.005]
    accel_noise = [0.1, 0.1, 0.1]
    position_noise = [0.05, 0.05, 0.05]
    gyro_bias_noise = [1e-05, 1e-05, 1e-05]
    accel_bias_noise = [0.0001, 0.0001, 0.0001]
