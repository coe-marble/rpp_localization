from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = str(Path(get_package_share_directory("rpp_localization")) / "params" / "navigation.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=defaults),
        DeclareLaunchArgument("configuration", default_value="ekf"),
        DeclareLaunchArgument("rpp_workspace", default_value=""),
        DeclareLaunchArgument("two_d_mode", default_value="false"),
        DeclareLaunchArgument("use_control", default_value="false"),
        Node(
            package="rpp_localization",
            executable="localization_node",
            name="estimator_script",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "configuration": LaunchConfiguration("configuration"),
                "rpp_workspace": LaunchConfiguration("rpp_workspace"),
                "navigation.two_d_mode": ParameterValue(LaunchConfiguration("two_d_mode"), value_type=bool),
                "navigation.use_control": ParameterValue(LaunchConfiguration("use_control"), value_type=bool),
            }],
        ),
    ])
