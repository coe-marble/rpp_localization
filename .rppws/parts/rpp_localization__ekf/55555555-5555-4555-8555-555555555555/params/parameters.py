# Coordinates and frames must match the configured NavigationModel.
_NAMES = ("x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "vroll", "vpitch", "vyaw", "ax", "ay", "az")
_QUANTITIES = (1, 1, 1, 4, 4, 4, 2, 2, 2, 5, 5, 5, 3, 3, 3)

def _field(index):
    return dict(name="base." + _NAMES[index], kind=2 if 3 <= index < 6 else 1,
                quantity=_QUANTITIES[index], size=1,
                frameId="map" if index < 6 else "base_link",
                childFrameId="base_link", resource="base")

# Prepare valid-axis subsets once. A missing axis never becomes a synthetic
# observation, and covariance cross terms for the retained axes stay intact.
_LAYOUTS = []
for _group in (tuple(range(6)), tuple(range(6, 12)), tuple(range(12, 15))):
    for _mask in range(1, 1 << len(_group)):
        _LAYOUTS.append(dict(fields=[_field(axis) for bit, axis in enumerate(_group) if _mask & (1 << bit)]))

class ComponentParameters:
    measurements = _LAYOUTS
    mahalanobis_thresholds = []
    history_capacity = 512
    # ROS automatic prediction supplies a frozen continuous density per interval.
    replay_noise_policy = "constant_rate"
