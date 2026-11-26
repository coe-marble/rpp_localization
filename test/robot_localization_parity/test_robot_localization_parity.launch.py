"""Compare rpp_localization with robot_localization and record runtime metrics."""

import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch_ros.actions import LifecycleNode, Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bag_parameters  # noqa: E402
import bag_playback  # noqa: E402
from bag_process_metrics import Processes  # noqa: E402
import parity_metrics  # noqa: E402


_DIRECTORY = Path(__file__).resolve().parent
_RESULTS_DIRECTORY = Path(os.environ.get("PARITY_RESULTS_DIR") or
                          Path(tempfile.gettempdir()) / "rpp-localization-parity")
_BAG_RATE = float(os.environ.get("PARITY_BAG_RATE", "1.0"))
_CLOCK_HZ = os.environ.get("PARITY_CLOCK_HZ", "1000")
if not math.isfinite(_BAG_RATE) or _BAG_RATE <= 0:
    raise ValueError("PARITY_BAG_RATE must be positive and finite")
_REPLAY_ENABLED = os.environ.get("PARITY_RL_REPLAY", "0") == "1"
_PROFILE = os.environ.get("PARITY_PROFILE", "0") == "1"
_REPLAY_EXECUTION = os.environ.get("PARITY_REPLAY_EXECUTION", "ros")
if _REPLAY_EXECUTION not in ("ros", "worker"):
    raise ValueError("PARITY_REPLAY_EXECUTION must be ros or worker")
_REPLAY_PARAMETERS = {"smooth_lagged_data": _REPLAY_ENABLED,
                      "history_length": 1.0 if _REPLAY_ENABLED else 0.0}

# Accuracy limits apply independently of replay mode and instrumentation.
_POSITION_RMS_LIMIT_M = {"ekf": 0.02, "ukf": 0.3}
_YAW_RMS_LIMIT_DEG = {"ekf": 0.1, "ukf": 1.0}
_CASES = [(bag, filter_type) for bag in ("test1", "test2", "test3") for filter_type in ("ekf", "ukf")]
if os.environ.get("PARITY_CASES"):
    requested = [tuple(case.split(":")) for case in os.environ["PARITY_CASES"].split(",")]
    if not requested or any(case not in _CASES for case in requested):
        raise ValueError("PARITY_CASES must select test1/test2/test3 with ekf/ukf")
    _CASES = requested


def _robot_localization_version() -> tuple[int, ...]:
    package = Path(get_package_share_directory("robot_localization")) / "package.xml"
    version = re.search(r"<version>([0-9.]+)</version>", package.read_text()).group(1)
    return tuple(int(part) for part in version.split("."))


def _worker_metrics_environment() -> dict[str, str]:
    if not _PROFILE:
        return {}
    if os.environ.get("LD_PRELOAD"):
        raise RuntimeError("Worker allocation tracking requires an empty LD_PRELOAD environment")
    library = (Path(get_package_prefix("rpp_localization")) / "lib" /
               "librpp_localization_worker_metrics.so")
    if not library.is_file():
        raise RuntimeError(
            f"Worker allocation tracker missing: {library}. "
            "Build/install rpp_localization with RPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS=ON on GNU/Linux.")
    return {"LD_PRELOAD": str(library)}


@launch_testing.parametrize("bag,filter_type", _CASES)
# Set the marker before parametrization copies it to each case's launch fixture.
@launch_testing.markers.keep_alive
def generate_test_description(bag, filter_type):
    upstream, pose = bag_parameters.upstream_parameters(bag, filter_type)
    expected = tuple(pose[name] for name in ("final_x", "final_y", "final_z"))
    if bag == "test1" and _robot_localization_version() < (3, 11):
        # Skip before launching clock-waiting nodes: shutdown during startup can
        # otherwise turn a supported skip into process crashes and noisy errors.
        return LaunchDescription([launch_testing.actions.ReadyToTest()]), {
            "bag_player": None, "expected": expected, "tolerance": pose["tolerance"],
            "resources": None, "effective_parameters": {},
            "skip_reason": "robot_localization < 3.11 removes gravity wrongly for the rotated IMU of test1",
        }
    worker_environment = _worker_metrics_environment()
    parameters = dict(upstream) | {"publish_tf": False} | _REPLAY_PARAMETERS
    temporary = Path(tempfile.mkdtemp(prefix="rpp_bag_benchmark_"))
    workspace = bag_parameters.workspace(
        Path(get_package_share_directory("rpp_localization")), temporary / "workspace", upstream,
        replay=_REPLAY_ENABLED, profile=_PROFILE)
    localization_parameters = bag_parameters.localization_parameters(upstream, workspace, filter_type)
    localization_parameters["replay_execution"] = _REPLAY_EXECUTION
    localization_parameters["benchmark.enabled"] = _PROFILE
    try:
        bag_parameters.validate_workspace(
            workspace, "navigation_estimation", localization_parameters["configuration"])
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    reference_arguments = dict(
        package="robot_localization", executable=f"{filter_type}_node",
        name="filter", namespace="robot_localization",
        parameters=[parameters], output="log")
    reference = (LifecycleNode(autostart=True, **reference_arguments)
                 if _robot_localization_version() >= (3, 11) else Node(**reference_arguments))
    nodes = {
        "robot_localization": reference,
        "rpp_localization": Node(
            package="rpp_localization",
            executable="localization_node",
            name="filter", namespace="rpp_localization", parameters=[localization_parameters],
            additional_env=worker_environment, output="log"),
    }
    bag_player = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(_DIRECTORY.parent / "data" / bag),
             "--clock", _CLOCK_HZ, "--rate", str(_BAG_RATE),
             "--start-paused", "--disable-keyboard-controls"],
        output="log")
    resources = Processes(profile=_PROFILE)

    def remember(name):
        def started(event, _context):
            resources.register(name, event.pid)
        return started

    def remember_exit(name):
        def exited(event, _context):
            resources.exited(name, event.returncode)
        return exited

    def cleanup(_event, _context):
        shutil.rmtree(temporary, ignore_errors=True)

    actions = [RegisterEventHandler(OnProcessStart(target_action=node, on_start=remember(name)))
               for name, node in nodes.items()]
    actions += [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=remember_exit(name)))
                for name, node in nodes.items()]
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=cleanup)))
    actions.extend(nodes.values())
    if bag == "test1":
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--x", "0.0", "--y", "-0.3", "--z", "0.52",
                       "--roll", "1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"], output="log"))
    actions += [bag_player, launch_testing.actions.ReadyToTest()]
    return LaunchDescription(actions), {
        "bag_player": bag_player, "expected": expected, "tolerance": pose["tolerance"],
        "resources": resources, "skip_reason": None, "effective_parameters": {
            "robot_localization": parameters,
            "rpp_localization": localization_parameters,
        },
    }


class TestRobotLocalizationParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("robot_localization_parity_recorder")
        names = ["robot_localization", "rpp_localization"]
        cls.tracks = {name: parity_metrics.Track() for name in names}
        cls.fixes = []
        cls.subscriptions = [cls.node.create_subscription(
            Odometry, f"/{name}/odometry/filtered", track.append, QoSProfile(depth=1000))
            for name, track in cls.tracks.items()]
        cls.subscriptions.append(cls.node.create_subscription(NavSatFix, "/fix", cls._on_fix, 1000))

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_fix(cls, message: NavSatFix) -> None:
        stamp = message.header.stamp
        cls.fixes.append((stamp.sec + stamp.nanosec * 1e-9, message.latitude, message.longitude))

    def _record_until_bag_ends(self, proc_info, bag_player, resources) -> None:
        deadline = time.monotonic() + 600.0 / _BAG_RATE
        while time.monotonic() < deadline:
            for _ in range(100):
                rclpy.spin_once(self.node, timeout_sec=0.01)
                resources.sample()
                self.assertFalse(resources.exits(), f"Filter exited before the bag completed: {resources.exits()}")
            try:
                proc_info.assertWaitForShutdown(process=bag_player, timeout=0.01)
                break
            except AssertionError:
                pass
        else:
            self.fail("the bag did not finish playing")
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_player)
        drain = time.monotonic() + 3.0
        while time.monotonic() < drain:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            resources.sample()
            self.assertFalse(resources.exits(), f"Filter exited while draining output: {resources.exits()}")

    def test_parity(self, proc_info, bag_player, bag, filter_type, expected, tolerance,
                    resources, effective_parameters, skip_reason) -> None:
        if skip_reason:
            self.skipTest(skip_reason)
        sensor_topics = sorted({topic for key, topic in effective_parameters["robot_localization"].items()
                                if re.fullmatch(r"(?:odom|imu|pose|twist)\d+", key) and topic})
        metadata = yaml.safe_load((_DIRECTORY.parent / "data" / bag / "metadata.yaml").read_text())
        start_time_ns = metadata["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]

        def check_running():
            self.assertFalse(resources.exits(), f"Filter exited during bag startup: {resources.exits()}")
            self.assertIsNone(bag_player.return_code, "Bag player exited before playback could start")

        bag_playback.resume_when_ready(
            self.node, self.tracks, sensor_topics, start_time_ns,
            check_running=check_running, before_resume=resources.begin)
        self._record_until_bag_ends(proc_info, bag_player, resources)
        usage = resources.finish()
        statistics = self._benchmark_statistics() if _PROFILE else {}
        result = {
            "bag": bag, "filter": filter_type, "bag_rate": _BAG_RATE,
            "replay_enabled": _REPLAY_ENABLED,
            "robot_localization_version": ".".join(map(str, _robot_localization_version())),
            "expected_final_position": list(expected), "upstream_tolerance_m": tolerance,
            "resources": usage, "effective_parameters": effective_parameters,
            "profiling_enabled": _PROFILE,
            "estimation_timing": {"rpp_localization": statistics} if _PROFILE else {},
            "cpu_seconds": {name: value["cpu_seconds"] for name, value in usage["processes"].items()},
        }
        if _PROFILE:
            result["worker_metrics"] = statistics.get("worker_metrics", {
                "error": statistics.get("error", "Worker metrics missing from benchmark_stats")})
        for name, track in self.tracks.items():
            result[name] = parity_metrics.summary(track, expected) if track.time else None
        result["difference"] = parity_metrics.compare(
            self.tracks["robot_localization"], self.tracks["rpp_localization"])
        if len(self.fixes) > 10:
            result["gnss_rms_m"] = {
                name: parity_metrics.rms_distance_to_fixes(track, self.fixes)
                for name, track in self.tracks.items() if track.time}
        _RESULTS_DIRECTORY.mkdir(parents=True, exist_ok=True)
        result["tracks"] = {name: track.to_dict() for name, track in self.tracks.items()}
        result_path = _RESULTS_DIRECTORY / f"{bag}_{filter_type}.json"
        result_path.write_text(json.dumps(result))
        print(result_path.resolve())

        # Save first so failed accuracy or instrumentation checks retain their evidence.
        if _PROFILE:
            self._assert_metrics(statistics, usage)
        for name, track in self.tracks.items():
            self.assertGreater(len(track.time), 100, f"{name} published too little")
            for field, values in track.arrays().items():
                self.assertTrue(numpy.isfinite(values).all(), f"{name}: nonfinite {field}")
            self.assertIn(name, usage["processes"], f"No launch PID was recorded for {name}")
            self.assertTrue(usage["processes"][name]["running_at_end"], f"{name} stopped during playback")
            self.assertFalse(usage["processes"][name]["started_after_playback"], f"{name} started after playback")
            self.assertLess(result[name]["final_error_m"], tolerance, name)
        difference = result["difference"]
        self.assertIsNotNone(difference, "Tracks have no overlapping samples")
        self.assertLess(difference["position_rms_m"], _POSITION_RMS_LIMIT_M[filter_type])
        self.assertLess(difference["yaw_rms_deg"], _YAW_RMS_LIMIT_DEG[filter_type])

    def _assert_metrics(self, statistics: dict, usage: dict) -> None:
        self.assertNotIn("error", statistics, f"RPP instrumentation unavailable: {statistics}")
        self.assertTrue(statistics["enabled"], "RPP benchmark instrumentation is disabled")
        self.assertGreater(statistics["operations"]["estimation"]["cpu"]["count"], 0,
                           "No estimation thread CPU measurements")
        for scope, values in statistics["operations"].items():
            self.assertEqual(values["cpu"]["count"], values["wall"]["count"],
                             f"{scope}: incomplete thread CPU measurements")

        worker = statistics.get("worker_metrics", {})
        self.assertTrue(worker, "Worker metrics missing from benchmark_stats")
        self.assertNotIn("error", worker, f"Worker metrics unavailable: {worker}")
        self.assertTrue(worker["memory_complete"], "Worker allocation tracking capacity exceeded")
        self.assertGreater(worker["thread_id"], 0)
        self.assertGreater(worker["cpu_seconds"], 0)
        self.assertGreaterEqual(worker["allocation_calls"], 0)
        self.assertGreaterEqual(worker["live_heap_bytes"], 0)
        self.assertGreaterEqual(worker["peak_live_heap_bytes"], worker["live_heap_bytes"])
        process = usage["processes"]["rpp_localization"]
        self.assertIsNotNone(process["worker_cpu_seconds"],
                             "No numerical worker thread was sampled during playback")
        self.assertEqual(worker["thread_id"], process["worker_thread_id"],
                         "CPU sampling and heap tracking must identify the same worker")

        counts = statistics.get("execution_counts")
        self.assertIsNotNone(counts, "Internal counters missing; register the updated localization plugins")
        self.assertGreater(counts["core_predictions"], 0)
        self.assertGreater(counts["core_corrections"], 0)
        self.assertEqual(counts["replay_attempts"], counts["replay_successes"] + counts["replay_failures"])
        self.assertFalse(counts["replay_pending"], "Replay did not catch up within the output drain window")
        if _REPLAY_EXECUTION == "ros":
            self.assertEqual(counts["replay_jobs_started"],
                             counts["replay_role_switches"] + counts["replay_jobs_cancelled"])

        replay = statistics["replay_statistics"]
        self.assertEqual(replay["count"], counts["replay_attempts"],
                         "Replay statistics must include every internal rewind")
        self.assertGreaterEqual(replay["states_mean"], 0)
        self.assertGreaterEqual(replay["states_stddev"], 0)
        self.assertLessEqual(replay["states_mean"], replay["states_max"])
        if not _REPLAY_ENABLED:
            for counter in ("replay_attempts", "replay_jobs_started", "replay_role_switches",
                            "replay_jobs_cancelled", "replayed_predictions", "replayed_corrections"):
                self.assertEqual(counts[counter], 0, f"{counter} occurred while PARITY_RL_REPLAY=0")

    def _benchmark_statistics(self) -> dict:
        """Collect cold snapshots after process CPU sampling has ended."""
        client = self.node.create_client(Trigger, "/rpp_localization/filter/benchmark_stats")
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                return {"error": "benchmark_stats unavailable; rebuild the instrumented node"}
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            if not future.done():
                return {"error": "benchmark_stats timed out"}
            response = future.result()
            return json.loads(response.message) if response.success else {"error": response.message}
        except (ValueError, RuntimeError) as error:
            return {"error": str(error)}
        finally:
            self.node.destroy_client(client)
