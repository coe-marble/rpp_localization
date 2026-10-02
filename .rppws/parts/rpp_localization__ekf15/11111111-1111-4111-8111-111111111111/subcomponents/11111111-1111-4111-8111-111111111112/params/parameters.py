from __future__ import annotations


class ComponentParameters:
    process_noise_covariance = []
    dynamic_process_noise_covariance = False
    use_control = False
    control_timeout_seconds = 0.2
    control_config = [False, False, False, False, False, False]
    acceleration_limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    acceleration_gains = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    deceleration_limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    deceleration_gains = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
