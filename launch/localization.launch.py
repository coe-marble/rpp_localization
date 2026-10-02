from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="rpp_localization",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=[
                str(
                    Path(get_package_share_directory("rpp_localization")) /
                    "params" /
                    "localization.yaml"
                )
            ],
        ),
    ])
