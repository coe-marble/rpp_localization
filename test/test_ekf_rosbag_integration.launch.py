from pathlib import Path
import math
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import rclpy
from nav_msgs.msg import Odometry


_EXPECTED_POSITION = (-40.0454, -76.9988, -2.6974)
_POSITION_TOLERANCE = 1.0452


def generate_test_description() -> tuple[LaunchDescription, dict[str, ExecuteProcess]]:
    test_directory = Path(__file__).resolve().parent
    bag_directory = test_directory / "data" / "test1"
    parameters_file = test_directory / "test_ekf_rosbag.yaml"

    localization_node = Node(
        package="rpp_localization",
        executable="localization_node",
        name="localization_node",
        parameters=[str(parameters_file)],
        output="screen",
    )
    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="test_base_link_to_imu_link",
        arguments=[
            "--x", "0.0", "--y", "-0.3", "--z", "0.52",
            "--roll", "1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
            "--frame-id", "base_link", "--child-frame-id", "imu_link",
        ],
        output="screen",
    )
    bag_player = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag_directory),
            "--clock", "100", "--rate", "10.0", "--disable-keyboard-controls",
        ],
        output="screen",
    )

    return LaunchDescription([
        localization_node,
        imu_transform,
        TimerAction(period=3.0, actions=[bag_player]),
        launch_testing.actions.ReadyToTest(),
    ]), {"bag_player": bag_player}


class TestEkfRosbagIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("rpp_localization_rosbag_evaluator")
        cls.final_odometry: Odometry | None = None
        cls.subscription = cls.node.create_subscription(
            Odometry,
            "/odometry/filtered",
            cls._on_odometry,
            10,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_odometry(cls, message: Odometry) -> None:
        cls.final_odometry = message

    def test_final_filtered_position(self, proc_info: object, bag_player: ExecuteProcess) -> None:
        proc_info.assertWaitForShutdown(process=bag_player, timeout=75)
        proc_info.assertProcessExit(process=bag_player)

        deadline = time.monotonic() + 5.0
        while self.final_odometry is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.final_odometry)
        assert self.final_odometry is not None
        position = self.final_odometry.pose.pose.position
        distance = math.dist(
            (position.x, position.y, position.z),
            _EXPECTED_POSITION,
        )
        self.assertLess(distance, _POSITION_TOLERANCE)
