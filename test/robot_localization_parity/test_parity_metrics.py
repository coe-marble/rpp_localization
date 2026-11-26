"""Check parity measurements against tracks with known distances and angles."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parity_metrics import EARTH_RADIUS, Track, compare, rms_distance_to_fixes, summary  # noqa: E402


def straight_track(times: list[float]) -> Track:
    return Track(
        time=times,
        position=[(stamp, 0.0, 0.0) for stamp in times],
        yaw=[0.0] * len(times),
        velocity=[(1.0, 0.0, 0.0)] * len(times),
        position_variance=[(0.25, 0.25, 0.25)] * len(times),
    )


class TestParityMetrics(unittest.TestCase):
    def test_interpolation_aligns_clocks_and_wraps_yaw(self):
        reference = straight_track([3.0, 1.0, 2.0])
        reference.yaw = [math.radians(angle) for angle in (-179.5, 179.5, 180.0)]
        rpp = Track(
            time=[0.0, 4.0],
            position=[(3.0, 4.0, 0.0), (7.0, 4.0, 0.0)],
            yaw=[math.radians(179.0), math.radians(-179.0)],
            velocity=[(3.0, 0.0, 0.0)] * 2,
            position_variance=[(1.0, 1.0, 1.0)] * 2,
        )

        difference = compare(reference, rpp)

        self.assertIsNotNone(difference)
        self.assertEqual(difference["samples"], 3)
        self.assertEqual(difference["start_time"], 1.0)
        self.assertEqual(difference["end_time"], 3.0)
        self.assertAlmostEqual(difference["position_rms_m"], 5.0)
        self.assertAlmostEqual(difference["position_max_m"], 5.0)
        self.assertAlmostEqual(difference["yaw_rms_deg"], 0.0)
        self.assertAlmostEqual(difference["yaw_max_deg"], 0.0)
        self.assertAlmostEqual(difference["velocity_rms_mps"], 2.0)
        self.assertAlmostEqual(difference["position_sigma_ratio_median"], 2.0)

    def test_comparison_uses_only_reference_samples_inside_the_overlap(self):
        reference = straight_track([0.0, 1.0, 2.0, 3.0, 4.0])
        rpp = straight_track([1.5, 2.5])

        difference = compare(reference, rpp)

        self.assertIsNotNone(difference)
        self.assertEqual(difference["samples"], 1)
        self.assertEqual(difference["start_time"], 2.0)
        self.assertEqual(difference["end_time"], 2.0)
        self.assertAlmostEqual(difference["position_rms_m"], 0.0)

    def test_missing_comparisons_do_not_extrapolate_or_report_zero_error(self):
        cases = (
            (Track(), straight_track([1.0, 2.0])),
            (straight_track([1.0, 2.0]), Track()),
            (straight_track([0.0, 1.0]), straight_track([2.0, 3.0])),
            (straight_track([0.0, 3.0]), straight_track([1.0, 2.0])),
        )
        for reference, rpp in cases:
            with self.subTest(reference=reference.time, rpp=rpp.time):
                self.assertIsNone(compare(reference, rpp))

    def test_summary_uses_the_latest_timestamp_and_actual_sample_rate(self):
        track = straight_track([8.0, 2.0, 5.0])

        result = summary(track, (8.0, 3.0, 0.0))

        self.assertEqual(result["messages"], 3)
        self.assertEqual(result["final_position"], [8.0, 0.0, 0.0])
        self.assertAlmostEqual(result["final_error_m"], 3.0)
        self.assertAlmostEqual(result["rate_hz"], 2.0 / 6.0)

    def test_gnss_alignment_removes_a_rigid_offset_and_rotation(self):
        ten_metres_in_degrees = math.degrees(10.0 / EARTH_RADIUS)
        fixes = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, ten_metres_in_degrees),
            (2.0, ten_metres_in_degrees, ten_metres_in_degrees),
            (3.0, ten_metres_in_degrees, 0.0),
        ]
        track = straight_track([0.0, 1.0, 2.0, 3.0])
        track.position = [(3.0, -4.0, 0.0), (3.0, 6.0, 0.0),
                          (-7.0, 6.0, 0.0), (-7.0, -4.0, 0.0)]

        self.assertAlmostEqual(rms_distance_to_fixes(track, fixes), 0.0)
