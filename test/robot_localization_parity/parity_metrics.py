"""Compare two filter tracks recorded from nav_msgs/Odometry."""

from dataclasses import dataclass, field
import math

import numpy


EARTH_RADIUS = 6378137.0


@dataclass
class Track:
    time: list[float] = field(default_factory=list)
    position: list[tuple[float, float, float]] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    velocity: list[tuple[float, float, float]] = field(default_factory=list)
    position_variance: list[tuple[float, float, float]] = field(default_factory=list)

    def append(self, message) -> None:
        stamp = message.header.stamp
        pose = message.pose.pose
        q = pose.orientation
        twist = message.twist.twist.linear
        covariance = message.pose.covariance
        self.time.append(stamp.sec + stamp.nanosec * 1e-9)
        self.position.append((pose.position.x, pose.position.y, pose.position.z))
        self.yaw.append(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.velocity.append((twist.x, twist.y, twist.z))
        self.position_variance.append((covariance[0], covariance[7], covariance[14]))

    def arrays(self) -> dict[str, numpy.ndarray]:
        order = numpy.argsort(self.time, kind="stable")
        return {
            "time": numpy.asarray(self.time)[order],
            "position": numpy.asarray(self.position)[order],
            "yaw": numpy.unwrap(numpy.asarray(self.yaw)[order]),
            "velocity": numpy.asarray(self.velocity)[order],
            "position_variance": numpy.asarray(self.position_variance)[order],
        }

    def to_dict(self) -> dict[str, list]:
        return {name: values.tolist() for name, values in self.arrays().items()}


def _interpolate(time: numpy.ndarray, source_time: numpy.ndarray,
                 values: numpy.ndarray) -> numpy.ndarray:
    if values.ndim == 1:
        return numpy.interp(time, source_time, values)
    return numpy.column_stack(
        [numpy.interp(time, source_time, values[:, i]) for i in range(values.shape[1])])


def _wrap(angle: numpy.ndarray) -> numpy.ndarray:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compare(reference: Track, candidate: Track) -> dict[str, float]:
    """Differences of candidate from reference, on the reference timestamps."""
    a = reference.arrays()
    b = candidate.arrays()
    start = max(a["time"][0], b["time"][0])
    end = min(a["time"][-1], b["time"][-1])
    inside = (a["time"] >= start) & (a["time"] <= end)
    time = a["time"][inside]

    position = _interpolate(time, b["time"], b["position"]) - a["position"][inside]
    distance = numpy.linalg.norm(position, axis=1)
    yaw = _wrap(_interpolate(time, b["time"], b["yaw"]) - a["yaw"][inside])
    velocity = numpy.linalg.norm(
        _interpolate(time, b["time"], b["velocity"]) - a["velocity"][inside], axis=1)
    sigma_a = numpy.sqrt(a["position_variance"][inside].sum(axis=1))
    sigma_b = numpy.sqrt(_interpolate(time, b["time"], b["position_variance"]).sum(axis=1))
    valid = sigma_a > 0.0

    return {
        "samples": int(time.size),
        "position_rms_m": float(numpy.sqrt(numpy.mean(distance ** 2))),
        "position_max_m": float(distance.max()),
        "final_position_difference_m": float(numpy.linalg.norm(
            b["position"][-1] - a["position"][-1])),
        "yaw_rms_deg": float(numpy.degrees(numpy.sqrt(numpy.mean(yaw ** 2)))),
        "yaw_max_deg": float(numpy.degrees(numpy.abs(yaw).max())),
        "velocity_rms_mps": float(numpy.sqrt(numpy.mean(velocity ** 2))),
        "position_sigma_ratio_median": float(numpy.median(sigma_b[valid] / sigma_a[valid]))
        if valid.any() else float("nan"),
    }


def summary(track: Track, expected_final: tuple[float, float, float]) -> dict[str, float]:
    a = track.arrays()
    duration = float(a["time"][-1] - a["time"][0])
    return {
        "messages": int(a["time"].size),
        "rate_hz": float((a["time"].size - 1) / duration) if duration > 0.0 else 0.0,
        "final_position": a["position"][-1].tolist(),
        "final_error_m": float(numpy.linalg.norm(a["position"][-1] - numpy.asarray(expected_final))),
    }


def rms_distance_to_fixes(track: Track, fixes: list[tuple[float, float, float]]) -> float:
    """RMS horizontal distance to GNSS fixes after a best-fit rotation and offset."""
    a = track.arrays()
    fix_array = numpy.asarray(fixes)
    latitude0, longitude0 = fix_array[0, 1], fix_array[0, 2]
    east = numpy.radians(fix_array[:, 2] - longitude0) * EARTH_RADIUS * math.cos(
        math.radians(latitude0))
    north = numpy.radians(fix_array[:, 1] - latitude0) * EARTH_RADIUS
    inside = (fix_array[:, 0] >= a["time"][0]) & (fix_array[:, 0] <= a["time"][-1])
    target = numpy.column_stack((east, north))[inside]
    source = _interpolate(fix_array[inside, 0], a["time"], a["position"][:, :2])
    source_centre, target_centre = source.mean(axis=0), target.mean(axis=0)
    left, _, right = numpy.linalg.svd((source - source_centre).T @ (target - target_centre))
    sign = numpy.sign(numpy.linalg.det(right.T @ left.T))
    rotation = right.T @ numpy.diag([1.0, sign]) @ left.T
    fitted = (source - source_centre) @ rotation.T + target_centre
    return float(numpy.sqrt(numpy.mean(numpy.sum((fitted - target) ** 2, axis=1))))
