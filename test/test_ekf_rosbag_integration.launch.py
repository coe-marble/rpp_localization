"""Replay the navigation bag through the generic localization node."""

from pathlib import Path
import math
import re
import shutil
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from nav_msgs.msg import Odometry
import rclpy
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "robot_localization_parity"))
import bag_parameters  # noqa: E402
import bag_playback  # noqa: E402
import parity_metrics  # noqa: E402


@launch_testing.markers.keep_alive
def generate_test_description():
    upstream, pose = bag_parameters.upstream_parameters("test1", "ekf")
    temporary = Path(tempfile.mkdtemp(prefix="rpp_generic_bag_"))
    workspace = bag_parameters.workspace(
        Path(get_package_share_directory("rpp_localization")),
        temporary / "workspace", upstream, replay=True)
    parameters = bag_parameters.localization_parameters(upstream, workspace, "ekf")
    bag_parameters.validate_workspace(workspace, "navigation_estimation", parameters["configuration"])
    bag_directory = Path(__file__).resolve().parent / "data/test1"
    metadata = yaml.safe_load((bag_directory / "metadata.yaml").read_text())
    start_time_ns = metadata["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]
    sensor_topics = sorted({value for key, value in upstream.items()
                            if re.fullmatch(r"(?:odom|imu|pose|twist)\d+", key) and value})
    localization_node = Node(
        package="rpp_localization", executable="localization_node", name="filter",
        namespace="rpp_localization", parameters=[parameters], output="screen")
    imu_transform = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "-0.3", "--z", "0.52",
            "--roll", "1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
            "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        parameters=[{"use_sim_time": True}], output="log")
    bag_player = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(bag_directory), "--clock", "100",
             "--rate", "10.0", "--start-paused", "--disable-keyboard-controls"],
        output="log")

    def cleanup(_event, _context):
        shutil.rmtree(temporary, ignore_errors=True)

    return LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        localization_node, imu_transform, bag_player,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "bag_player": bag_player, "localization_node": localization_node,
        "sensor_topics": sensor_topics, "start_time_ns": start_time_ns,
        "expected_position": tuple(pose[name] for name in ("final_x", "final_y", "final_z")),
        "position_tolerance": pose["tolerance"],
    }


class TestEkfRosbagIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("generic_localization_bag_recorder")
        cls.track = parity_metrics.Track()
        cls.subscription = cls.node.create_subscription(
            Odometry, "/rpp_localization/odometry/filtered", cls.track.append, 1000)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_final_filtered_position(
            self, proc_info, bag_player, localization_node, sensor_topics,
            start_time_ns, expected_position, position_tolerance):
        def check_running():
            self.assertIsNone(localization_node.return_code, "Generic localization node exited")

        bag_playback.resume_when_ready(
            self.node, {"rpp_localization": self.track}, sensor_topics, start_time_ns,
            check_running=check_running, before_resume=lambda: None)
        deadline = time.monotonic() + 75.0
        while time.monotonic() < deadline:
            check_running()
            for _ in range(100):
                rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                proc_info.assertWaitForShutdown(process=bag_player, timeout=0.01)
                break
            except AssertionError:
                pass
        else:
            self.fail("Bag playback did not finish")
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_player)
        drain_deadline = time.monotonic() + 3.0
        while time.monotonic() < drain_deadline:
            check_running()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreater(len(self.track.time), 100, "Too few generic odometry samples")
        final_position = self.track.arrays()["position"][-1]
        self.assertLess(math.dist(final_position, expected_position), position_tolerance)
