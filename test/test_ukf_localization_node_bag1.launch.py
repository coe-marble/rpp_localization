# SPDX-FileCopyrightText: 2018 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_ukf_localization_node_bag1.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='ukf1.txt'),
	
	#launch_ros.actions.Node(
         #   package='tf2_ros', executable='static_transform_publisher',node_name='bl_imu', output='screen',                       
          #  arguments=['0', '-0.3', '0.52', '-1.570796327', '0', '1.570796327', 'base_link', 'imu_link']		
           # ),	

	launch_ros.actions.Node(
            package='rpp_localization', executable='ukf_node', name='test_ukf_localization_node_bag1_ukf',
	    output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_ukf_localization_node_bag1.yaml'],
           ],
           ),
        
        launch_ros.actions.Node(
            package='rpp_localization', executable='test_ukf_localization_node_bag1', name='test_ukf_localization_node_bag1_pose',
            output='screen',
	    parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_ukf_localization_node_bag1.yaml'],
           ],
           ),
])
