"""rpp_localization and robot_localization side by side on robot_localization's test bags."""

import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import LifecycleNode, Node
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import parity_metrics  # noqa: E402


_DIRECTORY = Path(__file__).resolve().parent
_BAG_RATE = float(os.environ.get("PARITY_BAG_RATE", "1.0"))
_CLOCK_HZ = os.environ.get("PARITY_CLOCK_HZ", "1000")

_POSITION_RMS_LIMIT_M = {"ekf": 0.02, "ukf": 0.3}
_YAW_RMS_LIMIT_DEG = {"ekf": 0.1, "ukf": 1.0}

_CASES = [(bag, filter_type) for bag in ("test1", "test2", "test3") for filter_type in ("ekf", "ukf")]
if os.environ.get("PARITY_CASES"):
    _CASES = [tuple(case.split(":")) for case in os.environ["PARITY_CASES"].split(",")]


def _number(value):
    if isinstance(value, list):
        return [_number(item) for item in value]
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return value
    return value


def _upstream_parameters(bag: str, filter_type: str) -> tuple[dict, dict]:
    """Filter parameters and pose-tester parameters of the upstream bag test."""
    path = _DIRECTORY / "config" / f"test_{filter_type}_localization_node_bag{bag[-1]}.yaml"
    sections = yaml.safe_load(path.read_text())
    filter_parameters = next(v for k, v in sections.items() if k.endswith(f"_{filter_type}"))
    pose_parameters = next(v for k, v in sections.items() if k.endswith("_pose"))
    return ({key: _number(value) for key, value in filter_parameters["ros__parameters"].items()},
            pose_parameters["ros__parameters"])


def _rpp_workspace(process_noise: list[float] | None) -> str:
    """A copy of the package's .rppws with the bag's process noise in every part."""
    workspace = Path(tempfile.mkdtemp(prefix="rpp_parity_"))
    shutil.copytree(Path(get_package_share_directory("rpp_localization")) / ".rppws",
                    workspace / ".rppws")
    if process_noise:
        for parameters in (workspace / ".rppws").rglob("parameters.py"):
            text = parameters.read_text()
            text = re.sub(r"process_noise_covariance = \[\]",
                          f"process_noise_covariance = {process_noise!r}", text)
            parameters.write_text(text)
    return str(workspace)


def _robot_localization_version() -> tuple[int, ...]:
    package = Path(get_package_share_directory("robot_localization")) / "package.xml"
    version = re.search(r"<version>([0-9.]+)</version>", package.read_text()).group(1)
    return tuple(int(part) for part in version.split("."))


@launch_testing.parametrize("bag,filter_type", _CASES)
def generate_test_description(bag, filter_type):
    upstream, pose = _upstream_parameters(bag, filter_type)
    expected = tuple(pose[name] for name in ("final_x", "final_y", "final_z"))
    tolerance = pose["tolerance"]
    parameters = dict(upstream)
    parameters["publish_tf"] = False

    reference_arguments = dict(
        package="robot_localization",
        executable=f"{filter_type}_node",
        name="filter",
        namespace="robot_localization",
        parameters=[parameters],
        output="log",
    )
    # robot_localization is a lifecycle node from 3.11.
    reference = (LifecycleNode(autostart=True, **reference_arguments)
                 if _robot_localization_version() >= (3, 11) else Node(**reference_arguments))
    candidate = Node(
        package="rpp_localization",
        executable="localization_node",
        name="filter",
        namespace="rpp_localization",
        parameters=[parameters | {
            "configuration": filter_type,
            "rpp_workspace": _rpp_workspace(upstream.get("process_noise_covariance")),
        }],
        output="log",
    )
    bag_player = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(_DIRECTORY.parent / "data" / bag),
             "--clock", _CLOCK_HZ, "--rate", str(_BAG_RATE), "--disable-keyboard-controls"],
        output="log",
    )
    actions = [reference, candidate]
    if bag == "test1":
        # As in the original ROS 1 test.
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--x", "0.0", "--y", "-0.3", "--z", "0.52",
                       "--roll", "1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
            output="log",
        ))
    actions += [TimerAction(period=3.0, actions=[bag_player]), launch_testing.actions.ReadyToTest()]
    return LaunchDescription(actions), {
        "bag_player": bag_player, "expected": expected, "tolerance": tolerance}


def _cpu_seconds(command: str) -> float:
    """CPU time used so far by the running processes with this command name."""
    ticks = 0
    for stat in Path("/proc").glob("[0-9]*/stat"):
        try:
            text = stat.read_text()
        except OSError:
            continue
        name = text[text.index("(") + 1:text.rindex(")")]
        if name == command[:15]:
            fields = text[text.rindex(")") + 2:].split()
            if fields[0] == "Z":
                continue
            ticks += int(fields[11]) + int(fields[12])
    return ticks / os.sysconf("SC_CLK_TCK")


class TestRobotLocalizationParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("robot_localization_parity_recorder")
        cls.tracks = {"robot_localization": parity_metrics.Track(),
                      "rpp_localization": parity_metrics.Track()}
        cls.fixes = []
        qos = QoSProfile(depth=1000)
        for name, track in cls.tracks.items():
            cls.node.create_subscription(
                Odometry, f"/{name}/odometry/filtered", track.append, qos)
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

    def test_parity(self, proc_info, bag_player, bag, filter_type, expected, tolerance) -> None:
        if bag == "test1" and _robot_localization_version() < (3, 11):
            self.skipTest("robot_localization < 3.11 removes gravity wrongly for the rotated IMU of test1")
        self._record_until_bag_ends(proc_info, bag_player)
        cpu_seconds = {"robot_localization": _cpu_seconds(f"{filter_type}_node"),
                       "rpp_localization": _cpu_seconds("localization_node")}
        reference = self.tracks["robot_localization"]
        candidate = self.tracks["rpp_localization"]
        self.assertGreater(len(reference.time), 100, "robot_localization published too little")
        self.assertGreater(len(candidate.time), 100, "rpp_localization published too little")

        result = {
            "bag": bag,
            "filter": filter_type,
            "bag_rate": _BAG_RATE,
            "robot_localization_version": ".".join(map(str, _robot_localization_version())),
            "expected_final_position": list(expected),
            "upstream_tolerance_m": tolerance,
            "robot_localization": parity_metrics.summary(reference, expected),
            "rpp_localization": parity_metrics.summary(candidate, expected),
            "difference": parity_metrics.compare(reference, candidate),
            "cpu_seconds": cpu_seconds,
        }
        if len(self.fixes) > 10:
            result["gnss_rms_m"] = {
                name: parity_metrics.rms_distance_to_fixes(track, self.fixes)
                for name, track in self.tracks.items()}
        print(json.dumps(result, indent=2))

        results_directory = os.environ.get("PARITY_RESULTS_DIR")
        if results_directory:
            output = Path(results_directory)
            output.mkdir(parents=True, exist_ok=True)
            result["tracks"] = {name: track.to_dict() for name, track in self.tracks.items()}
            (output / f"{bag}_{filter_type}.json").write_text(json.dumps(result))

        self.assertLess(result["rpp_localization"]["final_error_m"], tolerance)
        self.assertLess(result["robot_localization"]["final_error_m"], tolerance)
        self.assertLess(result["difference"]["position_rms_m"], _POSITION_RMS_LIMIT_M[filter_type])
        self.assertLess(result["difference"]["yaw_rms_deg"], _YAW_RMS_LIMIT_DEG[filter_type])
