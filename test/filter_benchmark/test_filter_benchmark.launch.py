"""EKF, UKF and InEKF of rpp_localization on robot_localization's test bags, one filter per run.

BENCHMARK_SETUP selects the sensors:
- upstream: EKF and UKF with the upstream bag configs, the InEKF with all it can fuse
- common: the same sensors for all three (IMU orientation, rates and acceleration, odometry velocity)
"""

import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix

_DIRECTORY = Path(__file__).resolve().parent
_PARITY = _DIRECTORY.parent / "robot_localization_parity"
sys.path.insert(0, str(_PARITY))
import parity_metrics  # noqa: E402

# Shares the bag setup of the parity test; its file name is not importable.
_spec = importlib.util.spec_from_file_location(
    "robot_localization_parity", _PARITY / "test_robot_localization_parity.launch.py")
_parity = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_parity)
_cpu_seconds = _parity._cpu_seconds
_rpp_workspace = _parity._rpp_workspace
_upstream_parameters = _parity._upstream_parameters

_BAG_RATE = float(os.environ.get("BENCHMARK_BAG_RATE", "1.0"))
_CLOCK_HZ = os.environ.get("BENCHMARK_CLOCK_HZ", "1000")
_SETUP = os.environ.get("BENCHMARK_SETUP", "upstream")
if _SETUP not in ("upstream", "common"):
    raise ValueError(f"unknown BENCHMARK_SETUP {_SETUP}")

_CASES = [(bag, filter_type) for bag in ("test1", "test2", "test3")
          for filter_type in ("ekf", "ukf", "inekf")]
if os.environ.get("BENCHMARK_CASES"):
    _CASES = [tuple(case.split(":")) for case in os.environ["BENCHMARK_CASES"].split(",")]

# The InEKF fuses the IMU and the odometry velocity, plus the odometry axes
# the upstream EKF fuses as positions.
_INEKF_POSITION_AXES = {"test1": [False, False, False], "test2": [False, False, False],
                        "test3": [False, False, True]}

# The sensors of the InEKF, as 15-state masks for the EKF and UKF.
_COMMON_IMU = [False, False, False, True, True, True, False, False, False,
               True, True, True, True, True, True]


# InEKF model noise overrides as JSON, e.g. '{"accel_noise": [0.5, 0.5, 0.5]}'.
_INEKF_NOISE = json.loads(os.environ.get("BENCHMARK_INEKF_NOISE", "{}"))


def _inekf_workspace() -> str:
    """A copy of the package's .rppws with the InEKF model noise overrides applied."""
    from ament_index_python.packages import get_package_share_directory
    workspace = Path(tempfile.mkdtemp(prefix="rpp_benchmark_"))
    shutil.copytree(Path(get_package_share_directory("rpp_localization")) / ".rppws",
                    workspace / ".rppws")
    model = next((workspace / ".rppws/parts/rpp_localization__inekf").rglob(
        "subcomponents/*/params/parameters.py"))
    text = model.read_text()
    for name, value in _INEKF_NOISE.items():
        text, count = re.subn(rf"^(\s*{name}) = .*$", rf"\1 = {value!r}", text, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"unknown InEKF model parameter {name}")
    model.write_text(text)
    return str(workspace)


def _common_odometry(bag: str) -> list[bool]:
    mask = [False] * 15
    mask[2] = _INEKF_POSITION_AXES[bag][2]
    mask[6:9] = [True, True, True]
    return mask


def _filter_node(bag: str, filter_type: str) -> Node:
    upstream, _ = _upstream_parameters(bag, "ukf" if filter_type == "ukf" else "ekf")
    if filter_type == "inekf":
        return Node(
            package="rpp_localization", executable="inekf_node", name="filter",
            namespace="inekf", output="log",
            parameters=[{
                "use_sim_time": True,
                "publish_tf": False,
                "world_frame": upstream["world_frame"],
                "base_link_frame": upstream["base_link_frame"],
                "imu0": upstream["imu0"],
                "odom0": upstream["odom0"],
                "odom0_position_axes": _INEKF_POSITION_AXES[bag],
            } | ({"rpp_workspace": _inekf_workspace()} if _INEKF_NOISE else {})])
    if _SETUP != "upstream":
        upstream["imu0_config"] = _COMMON_IMU
        upstream["odom0_config"] = _common_odometry(bag)
    return Node(
        package="rpp_localization", executable="localization_node", name="filter",
        namespace=filter_type, output="log",
        parameters=[upstream | {
            "publish_tf": False,
            "configuration": filter_type,
            "rpp_workspace": _rpp_workspace(upstream.get("process_noise_covariance")),
        }])


@launch_testing.parametrize("bag,filter_type", _CASES)
def generate_test_description(bag, filter_type):
    _, pose = _upstream_parameters(bag, "ekf")
    expected = tuple(pose[name] for name in ("final_x", "final_y", "final_z"))
    bag_player = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(_DIRECTORY.parent / "data" / bag),
             "--clock", _CLOCK_HZ, "--rate", str(_BAG_RATE), "--disable-keyboard-controls"],
        output="log",
    )
    actions = [_filter_node(bag, filter_type)]
    if bag == "test1":
        # As in the original ROS 1 test.
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher", output="log",
            arguments=["--x", "0.0", "--y", "-0.3", "--z", "0.52",
                       "--roll", "1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        ))
    actions += [TimerAction(period=3.0, actions=[bag_player]), launch_testing.actions.ReadyToTest()]
    return LaunchDescription(actions), {
        "bag_player": bag_player, "expected": expected, "tolerance": pose["tolerance"]}


class TestFilterBenchmark(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("filter_benchmark_recorder")
        cls.tracks = {name: parity_metrics.Track() for name in ("ekf", "ukf", "inekf")}
        cls.fixes = []
        qos = QoSProfile(depth=1000)
        for name, track in cls.tracks.items():
            cls.node.create_subscription(Odometry, f"/{name}/odometry/filtered", track.append, qos)
        cls.node.create_subscription(NavSatFix, "/fix", cls._on_fix, qos)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_fix(cls, message: NavSatFix) -> None:
        stamp = message.header.stamp
        cls.fixes.append((stamp.sec + stamp.nanosec * 1e-9, message.latitude, message.longitude))

    def _record_until_bag_ends(self, proc_info, bag_player) -> None:
        deadline = time.monotonic() + 600.0
        while time.monotonic() < deadline:
            for _ in range(100):
                rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                proc_info.assertWaitForShutdown(process=bag_player, timeout=0.01)
                break
            except AssertionError:
                pass
        else:
            self.fail("the bag did not finish playing")
        drain = time.monotonic() + 3.0
        while time.monotonic() < drain:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_benchmark(self, proc_info, bag_player, bag, filter_type, expected, tolerance) -> None:
        # A second bag player on the same graph would mix two clocks into the tracks.
        rclpy.spin_once(self.node, timeout_sec=5.0)
        self.assertLessEqual(self.node.count_publishers("/clock"), 1, "another bag is playing")
        self._record_until_bag_ends(proc_info, bag_player)
        executable = "inekf_node" if filter_type == "inekf" else "localization_node"
        track = self.tracks[filter_type]
        self.assertGreater(len(track.time), 100, f"{filter_type} published too little")
        self.assertTrue(numpy.isfinite(numpy.asarray(track.position)).all())

        result = {
            "bag": bag,
            "filter": filter_type,
            "setup": _SETUP,
            "inekf_noise": _INEKF_NOISE,
            "bag_rate": _BAG_RATE,
            "expected_final_position": list(expected),
            "upstream_tolerance_m": tolerance,
            "summary": parity_metrics.summary(track, expected),
            "cpu_seconds": _cpu_seconds(executable),
        }
        print(json.dumps(result, indent=2))
        if len(self.fixes) > 10:
            result["fixes"] = self.fixes

        results_directory = os.environ.get("BENCHMARK_RESULTS_DIR")
        if results_directory:
            output = Path(results_directory)
            output.mkdir(parents=True, exist_ok=True)
            result["track"] = track.to_dict()
            (output / f"{bag}_{filter_type}.json").write_text(json.dumps(result))
