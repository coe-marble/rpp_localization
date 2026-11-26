"""The original six bag cases, with generic/legacy RPP and robot_localization."""

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
_BAG_RATE = float(os.environ.get("PARITY_BAG_RATE", "1.0"))
_CLOCK_HZ = os.environ.get("PARITY_CLOCK_HZ", "1000")
_MODE = os.environ.get("PARITY_IMPLEMENTATION", "both")
if _MODE not in ("both", "generic", "legacy"):
    raise ValueError("PARITY_IMPLEMENTATION must be both, generic, or legacy")
if not math.isfinite(_BAG_RATE) or _BAG_RATE <= 0:
    raise ValueError("PARITY_BAG_RATE must be positive and finite")
_RL_REPLAY = os.environ.get("PARITY_RL_REPLAY", "0") == "1"
_PROFILE = os.environ.get("PARITY_PROFILE", "0") == "1"
_REPLAY_EXECUTION = os.environ.get("PARITY_REPLAY_EXECUTION", "ros")
if _REPLAY_EXECUTION not in ("ros", "worker"):
    raise ValueError("PARITY_REPLAY_EXECUTION must be ros or worker")
_REPLAY_PARAMETERS = {"smooth_lagged_data": _RL_REPLAY, "history_length": 1.0 if _RL_REPLAY else 0.0}

# Keep the original accuracy requirements for every implementation.
_POSITION_RMS_LIMIT_M = {"ekf": 0.02, "ukf": 0.3}
_YAW_RMS_LIMIT_DEG = {"ekf": 0.1, "ukf": 1.0}
_CASES = [(bag, filter_type) for bag in ("test1", "test2", "test3") for filter_type in ("ekf", "ukf")]
if os.environ.get("PARITY_CASES"):
    requested = [tuple(case.split(":")) for case in os.environ["PARITY_CASES"].split(",")]
    if not requested or any(case not in _CASES for case in requested):
        raise ValueError("PARITY_CASES must select test1/test2/test3 with ekf/ukf")
    _CASES = requested


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
    path = _DIRECTORY / "config" / f"test_{filter_type}_localization_node_bag{bag[-1]}.yaml"
    sections = yaml.safe_load(path.read_text())
    filter_parameters = next(v for k, v in sections.items() if k.endswith(f"_{filter_type}"))
    pose_parameters = next(v for k, v in sections.items() if k.endswith("_pose"))
    return ({key: _number(value) for key, value in filter_parameters["ros__parameters"].items()},
            pose_parameters["ros__parameters"])


def _robot_localization_version() -> tuple[int, ...]:
    package = Path(get_package_share_directory("robot_localization")) / "package.xml"
    version = re.search(r"<version>([0-9.]+)</version>", package.read_text()).group(1)
    return tuple(int(part) for part in version.split("."))


def _compare(reference, candidate, *, timebase=None):
    if not reference.time or not candidate.time:
        return None
    start = max(min(reference.time), min(candidate.time))
    end = min(max(reference.time), max(candidate.time))
    clock = reference if timebase is None else timebase
    if not any(start <= stamp <= end for stamp in clock.time):
        return None
    return parity_metrics.compare(reference, candidate, timebase=timebase)


def _worker_metrics_environment() -> dict[str, str]:
    if not _PROFILE or _MODE == "legacy":
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
    upstream, pose = _upstream_parameters(bag, filter_type)
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
        replay=_RL_REPLAY, profile=_PROFILE and _MODE != "legacy")
    generic_parameters = bag_parameters.generic(upstream, workspace, filter_type)
    generic_parameters["replay_execution"] = _REPLAY_EXECUTION
    legacy_parameters = parameters | {"configuration": filter_type, "rpp_workspace": workspace}
    generic_parameters["benchmark.enabled"] = _PROFILE
    legacy_parameters["benchmark.enabled"] = _PROFILE
    try:
        if _MODE != "legacy":
            bag_parameters.validate_workspace(workspace, "navigation_estimation", generic_parameters["configuration"])
        if _MODE != "generic":
            bag_parameters.validate_workspace(workspace, "localization", filter_type)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    candidate_parameters = legacy_parameters if _MODE == "legacy" else generic_parameters
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
            executable="legacy_localization_node" if _MODE == "legacy" else "localization_node",
            name="filter", namespace="rpp_localization", parameters=[candidate_parameters],
            additional_env=worker_environment, output="log"),
    }
    if _MODE == "both":
        nodes["legacy_rpp_localization"] = Node(
            package="rpp_localization", executable="legacy_localization_node", name="filter",
            namespace="legacy_rpp_localization", parameters=[legacy_parameters], output="log")
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
            "rpp_localization": candidate_parameters,
            **({"legacy_rpp_localization": legacy_parameters} if _MODE == "both" else {}),
        },
    }


class TestRobotLocalizationParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("robot_localization_parity_recorder")
        names = ["robot_localization", "rpp_localization"]
        if _MODE == "both":
            names.append("legacy_rpp_localization")
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
        statistics_names = []
        if _PROFILE:
            statistics_names = [name for name in self.tracks if name != "robot_localization"]
        statistics = self._benchmark_statistics(statistics_names) if _PROFILE else {}
        reference = self.tracks["robot_localization"]
        candidate = self.tracks["rpp_localization"]
        result = {
            "bag": bag, "filter": filter_type, "bag_rate": _BAG_RATE,
            "implementation": "legacy" if _MODE == "legacy" else "generic",
            "benchmark_mode": _MODE, "robot_localization_replay": _RL_REPLAY,
            "replay_enabled": _RL_REPLAY,
            "robot_localization_version": ".".join(map(str, _robot_localization_version())),
            "expected_final_position": list(expected), "upstream_tolerance_m": tolerance,
            "resources": usage, "effective_parameters": effective_parameters,
            "profiling_enabled": _PROFILE,
            "estimation_timing": statistics if _PROFILE else {},
            "cpu_seconds": {name: value["cpu_seconds"] for name, value in usage["processes"].items()},
        }
        if _PROFILE and _MODE != "legacy":
            generic_statistics = statistics["rpp_localization"]
            result["worker_metrics"] = generic_statistics.get("worker_metrics", {
                "error": generic_statistics.get("error", "Worker metrics missing from benchmark_stats")})
        for name, track in self.tracks.items():
            result[name] = parity_metrics.summary(track, expected) if track.time else None
        result["difference"] = _compare(reference, candidate)
        if _MODE == "both":
            legacy = self.tracks["legacy_rpp_localization"]
            result["comparison_timebase"] = "robot_localization"
            result["legacy_difference"] = _compare(legacy, candidate, timebase=reference)
            # Preserve startup behavior before the external reference's first
            # sample as a separate diagnostic, without changing accuracy limits.
            result["legacy_timeline_difference"] = _compare(legacy, candidate)
            result["legacy_reference_difference"] = _compare(reference, legacy)
            old = result["cpu_seconds"].get("legacy_rpp_localization")
            new = result["cpu_seconds"].get("rpp_localization")
            running = all(usage["processes"].get(name, {}).get("running_at_end", False)
                          for name in ("rpp_localization", "legacy_rpp_localization"))
            result["generic_cpu_change_percent"] = (100.0 * (new / old - 1.0)
                                                     if running and old and new is not None else None)
            if _PROFILE:
                timing = result["estimation_timing"]
                batches = [timing.get(name, {}).get("operations", {}).get("estimation", {})
                           for name in ("legacy_rpp_localization", "rpp_localization")]
                valid = all(batch.get("cpu", {}).get("count", 0) > 0 and
                            batch["cpu"]["count"] == batch["wall"]["count"] for batch in batches)
                old_ns, new_ns = (batch.get("cpu", {}).get("total_ns", 0) for batch in batches)
                # ROS replay executes outside the numeric worker's batch. Count
                # both disjoint scopes when comparing the total estimator work.
                replay = timing.get("rpp_localization", {}).get("operations", {}).get("replay", {})
                valid = valid and replay.get("cpu", {}).get("count", 0) == replay.get("wall", {}).get("count", 0)
                new_ns += replay.get("cpu", {}).get("total_ns", 0)
                result["generic_estimation_cpu_change_percent"] = (
                    100.0 * (new_ns / old_ns - 1.0) if valid and old_ns else None)
        if len(self.fixes) > 10:
            result["gnss_rms_m"] = {
                name: parity_metrics.rms_distance_to_fixes(track, self.fixes)
                for name, track in self.tracks.items() if track.time}
        print(json.dumps(result, indent=2))
        results_directory = os.environ.get("PARITY_RESULTS_DIR")
        if results_directory:
            output = Path(results_directory)
            output.mkdir(parents=True, exist_ok=True)
            result["tracks"] = {name: track.to_dict() for name, track in self.tracks.items()}
            (output / f"{bag}_{filter_type}.json").write_text(json.dumps(result))

        # Save measurements before assertions, so failed migrations remain reviewable.
        if _PROFILE and _MODE != "legacy":
            worker = result["worker_metrics"]
            self.assertNotIn("error", worker, f"Generic worker metrics unavailable: {worker}")
            self.assertTrue(worker["memory_complete"], "Worker allocation tracking capacity exceeded")
            self.assertGreater(worker["thread_id"], 0)
            self.assertGreater(worker["cpu_seconds"], 0)
            self.assertGreaterEqual(worker["allocation_calls"], 0)
            self.assertGreaterEqual(worker["peak_live_heap_bytes"], worker["live_heap_bytes"])
            counts = result["estimation_timing"]["rpp_localization"].get("execution_counts")
            self.assertIsNotNone(counts, "Internal counters missing; register the updated localization plugins")
            self.assertGreater(counts["core_predictions"], 0)
            self.assertGreater(counts["core_corrections"], 0)
            self.assertEqual(counts["replay_attempts"], counts["replay_successes"] + counts["replay_failures"])
            self.assertFalse(counts.get("replay_pending", False), "Replay did not catch up within the output drain window")
            if _REPLAY_EXECUTION == "ros":
                self.assertEqual(counts["replay_jobs_started"],
                                 counts["replay_role_switches"] + counts["replay_jobs_cancelled"])
        for name, timing in result["estimation_timing"].items():
            self.assertNotIn("error", timing, f"{name}: {timing}")
            self.assertTrue(timing.get("enabled"), f"{name}: benchmark instrumentation is disabled")
            self.assertGreater(timing["operations"]["estimation"]["cpu"]["count"], 0,
                               f"{name}: no estimation thread CPU measurements")
            replay = timing.get("replay_statistics")
            self.assertIsNotNone(replay, f"{name}: replay statistics missing; rebuild the node/plugins")
            self.assertGreaterEqual(replay["count"], 0)
            if not _RL_REPLAY:
                self.assertEqual(replay["count"], 0, f"{name}: replay occurred while PARITY_RL_REPLAY=0")
            self.assertGreaterEqual(replay["states_stddev"], 0)
            self.assertLessEqual(replay["states_mean"], replay["states_max"])
            for scope, values in timing["operations"].items():
                self.assertEqual(values["cpu"]["count"], values["wall"]["count"],
                                 f"{name} {scope}: incomplete thread CPU measurements")
        if _PROFILE and _MODE != "legacy":
            self.assertIsNotNone(usage["processes"].get("rpp_localization", {}).get("worker_cpu_seconds"),
                                 "No numerical worker thread was sampled during playback")
        for name, track in self.tracks.items():
            self.assertGreater(len(track.time), 100, f"{name} published too little")
            self.assertIn(name, usage["processes"], f"No launch PID was recorded for {name}")
            self.assertTrue(usage["processes"][name]["running_at_end"], f"{name} stopped during playback")
            self.assertFalse(usage["processes"][name]["started_after_playback"], f"{name} started after playback")
            self.assertLess(result[name]["final_error_m"], tolerance, name)
        comparisons = ["difference"]
        if _MODE == "both":
            comparisons += ["legacy_difference", "legacy_reference_difference"]
        for name in comparisons:
            self.assertIsNotNone(result[name], name)
            self.assertLess(result[name]["position_rms_m"], _POSITION_RMS_LIMIT_M[filter_type], name)
            self.assertLess(result[name]["yaw_rms_deg"], _YAW_RMS_LIMIT_DEG[filter_type], name)

    def _benchmark_statistics(self, names: list[str]) -> dict:
        """Collect cold snapshots after process CPU sampling has ended."""
        clients = {name: self.node.create_client(Trigger, f"/{name}/filter/benchmark_stats")
                   for name in names}
        futures, result = {}, {}
        deadline = time.monotonic() + 10.0
        try:
            while len(result) < len(clients) and time.monotonic() < deadline:
                for name, client in clients.items():
                    if name not in futures and client.service_is_ready():
                        futures[name] = client.call_async(Trigger.Request())
                    if name in result or name not in futures or not futures[name].done():
                        continue
                    try:
                        response = futures[name].result()
                        result[name] = (json.loads(response.message) if response.success
                                        else {"error": response.message})
                    except (ValueError, RuntimeError) as error:
                        result[name] = {"error": str(error)}
                rclpy.spin_once(self.node, timeout_sec=0.01)
            for name in clients:
                result.setdefault(name, {"error": "benchmark_stats timed out; rebuild the instrumented node"})
            return result
        finally:
            for client in clients.values():
                self.node.destroy_client(client)
