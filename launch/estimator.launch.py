from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rpp_localization",
            executable="estimator_script_node",
            name="estimator_script",
            output="screen",
            parameters=[str(Path(get_package_share_directory("rpp_localization")) / "params" / "estimator.yaml")],
        ),
    ])
