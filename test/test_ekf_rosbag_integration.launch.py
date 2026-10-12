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
import numpy
from sensor_msgs.msg import NavSatFix


# End of the robot_localization EKF on this bag with the same inputs. It shows
# agreement with that filter, not accuracy.
_EXPECTED_POSITION = (-40.0454, -76.9988, -2.6974)
_POSITION_TOLERANCE = 1.0452

# The GPS fixes in the bag are not fused, so they are an independent check.
# The IMU heading in this bag is off by about 15 degrees with a sign that
# changes when the robot turns around, which leaves the track about 21 m RMS
# from the fixes. The bound catches a filter that does worse than that.
_GPS_RMS_LIMIT = 25.0
_EARTH_RADIUS = 6378137.0


def _rms_distance_to_gps(track: list[tuple[float, float, float]],
                         fixes: list[tuple[float, float, float]]) -> float:
    """RMS distance after fitting the track to the fixes with one rotation and offset.

    The odometry frame and east-north differ by an unknown heading and origin.
    """
    track_array = numpy.array(track)
    fix_array = numpy.array(fixes)
    latitude0, longitude0 = fix_array[0, 1], fix_array[0, 2]
    east = numpy.radians(fix_array[:, 2] - longitude0) * _EARTH_RADIUS * math.cos(
        math.radians(latitude0))
    north = numpy.radians(fix_array[:, 1] - latitude0) * _EARTH_RADIUS
    inside = (fix_array[:, 0] >= track_array[0, 0]) & (fix_array[:, 0] <= track_array[-1, 0])
    target = numpy.column_stack((east, north))[inside]
    source = numpy.column_stack((
        numpy.interp(fix_array[inside, 0], track_array[:, 0], track_array[:, 1]),
        numpy.interp(fix_array[inside, 0], track_array[:, 0], track_array[:, 2]),
    ))
    source_centre, target_centre = source.mean(axis=0), target.mean(axis=0)
    left, _, right = numpy.linalg.svd((source - source_centre).T @ (target - target_centre))
    sign = numpy.sign(numpy.linalg.det(right.T @ left.T))
    rotation = right.T @ numpy.diag([1.0, sign]) @ left.T
    fitted = (source - source_centre) @ rotation.T + target_centre
    return float(numpy.sqrt(numpy.mean(numpy.sum((fitted - target) ** 2, axis=1))))


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
        cls.track: list[tuple[float, float, float]] = []
        cls.fixes: list[tuple[float, float, float]] = []
        cls.subscription = cls.node.create_subscription(
            Odometry,
            "/odometry/filtered",
            cls._on_odometry,
            1000,
        )
        cls.fix_subscription = cls.node.create_subscription(
            NavSatFix,
            "/fix",
            cls._on_fix,
            1000,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_odometry(cls, message: Odometry) -> None:
        cls.final_odometry = message
        stamp = message.header.stamp
        position = message.pose.pose.position
        cls.track.append((stamp.sec + stamp.nanosec * 1e-9, position.x, position.y))

    @classmethod
    def _on_fix(cls, message: NavSatFix) -> None:
        stamp = message.header.stamp
        cls.fixes.append((stamp.sec + stamp.nanosec * 1e-9, message.latitude, message.longitude))

    def test_final_filtered_position(self, proc_info: object, bag_player: ExecuteProcess) -> None:
        # The messages are collected while the bag plays.
        deadline = time.monotonic() + 75.0
        finished = False
        while not finished and time.monotonic() < deadline:
            for _ in range(200):
                rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                proc_info.assertWaitForShutdown(process=bag_player, timeout=0.01)
                finished = True
            except AssertionError:
                pass
        self.assertTrue(finished, "the bag did not finish playing")

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.final_odometry)
        assert self.final_odometry is not None
        position = self.final_odometry.pose.pose.position
        distance = math.dist(
            (position.x, position.y, position.z),
            _EXPECTED_POSITION,
        )
        self.assertLess(distance, _POSITION_TOLERANCE)

        self.assertGreater(len(self.track), 1000)
        self.assertGreater(len(self.fixes), 500)
        self.assertLess(_rms_distance_to_gps(self.track, self.fixes), _GPS_RMS_LIMIT)
