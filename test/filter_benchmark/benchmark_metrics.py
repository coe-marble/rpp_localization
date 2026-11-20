"""Per-axis and GNSS drift metrics of filter tracks recorded by parity_metrics.Track."""

import math

import numpy

EARTH_RADIUS = 6378137.0
AXES = ("x", "y", "z")


def _arrays(track: dict) -> dict[str, numpy.ndarray]:
    return {key: numpy.asarray(value) for key, value in track.items()}


def _interpolate(time: numpy.ndarray, source_time: numpy.ndarray,
                 values: numpy.ndarray) -> numpy.ndarray:
    if values.ndim == 1:
        return numpy.interp(time, source_time, values)
    return numpy.column_stack(
        [numpy.interp(time, source_time, values[:, i]) for i in range(values.shape[1])])


def _wrap(angle: numpy.ndarray) -> numpy.ndarray:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def end_error_axes(track: dict, expected: list[float]) -> dict[str, float]:
    """Signed end error per axis and its norm."""
    error = _arrays(track)["position"][-1] - numpy.asarray(expected)
    return {**{axis: float(error[i]) for i, axis in enumerate(AXES)},
            "norm": float(numpy.linalg.norm(error))}


def difference_axes(reference: dict, candidate: dict) -> dict[str, float]:
    """RMS difference per axis and of yaw, on the reference timestamps."""
    a, b = _arrays(reference), _arrays(candidate)
    inside = (a["time"] >= b["time"][0]) & (a["time"] <= b["time"][-1])
    time = a["time"][inside]
    position = _interpolate(time, b["time"], b["position"]) - a["position"][inside]
    yaw = _wrap(_interpolate(time, b["time"], b["yaw"]) - a["yaw"][inside])
    result = {axis: float(numpy.sqrt(numpy.mean(position[:, i] ** 2)))
              for i, axis in enumerate(AXES)}
    result["yaw_deg"] = float(numpy.degrees(numpy.sqrt(numpy.mean(yaw ** 2))))
    return result


def _fixes_east_north(fixes: list) -> tuple[numpy.ndarray, numpy.ndarray]:
    fix = numpy.asarray(fixes)
    latitude0, longitude0 = fix[0, 1], fix[0, 2]
    east = numpy.radians(fix[:, 2] - longitude0) * EARTH_RADIUS * math.cos(math.radians(latitude0))
    north = numpy.radians(fix[:, 1] - latitude0) * EARTH_RADIUS
    return fix[:, 0], numpy.column_stack((east, north))


def gnss_drift(track: dict, fixes: list, align_seconds: float = 60.0,
               checkpoints: tuple[float, ...] = (100.0, 200.0, 300.0)) -> dict[str, float]:
    """Horizontal error to GNSS after aligning on the first `align_seconds` only.

    The rotation and offset come from the start of the run, so the error that
    follows is the drift of the track, which a fit over the whole run would hide.
    """
    a = _arrays(track)
    fix_time, target = _fixes_east_north(fixes)
    inside = (fix_time >= a["time"][0]) & (fix_time <= a["time"][-1])
    fix_time, target = fix_time[inside], target[inside]
    source = _interpolate(fix_time, a["time"], a["position"][:, :2])
    elapsed = fix_time - fix_time[0]
    aligned = elapsed <= align_seconds
    source_centre = source[aligned].mean(axis=0)
    target_centre = target[aligned].mean(axis=0)
    left, _, right = numpy.linalg.svd(
        (source[aligned] - source_centre).T @ (target[aligned] - target_centre))
    sign = numpy.sign(numpy.linalg.det(right.T @ left.T))
    rotation = right.T @ numpy.diag([1.0, sign]) @ left.T
    error = numpy.linalg.norm((source - source_centre) @ rotation.T + target_centre - target, axis=1)
    result = {"align_rms_m": float(numpy.sqrt(numpy.mean(error[aligned] ** 2))),
              "rms_after_m": float(numpy.sqrt(numpy.mean(error[~aligned] ** 2))),
              "end_m": float(error[-1])}
    for checkpoint in checkpoints:
        index = int(numpy.argmin(numpy.abs(elapsed - checkpoint)))
        result[f"at_{int(checkpoint)}s_m"] = float(error[index])
    return result
