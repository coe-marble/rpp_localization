#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2018 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Launch file for test_se_node_interfaces."""

import launch
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
import sys
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_testing.legacy import LaunchTestService
from launch.actions import ExecuteProcess
from launch import LaunchService
from ament_index_python.packages import get_package_prefix

def generate_launch_description(node_type):
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_se_node_interfaces.yaml'    
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    se_node = launch_ros.actions.Node(
            package='rpp_localization',
            executable=node_type + '_node',
            name='test_se_node_interfaces',
            output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_se_node_interfaces.yaml'],
           ],)

    return LaunchDescription([
        se_node,
    ])


def main(argv=sys.argv[1:]):
    node_type = os.environ['NODE_TYPE']
    ld = generate_launch_description(node_type)

    test1_action = ExecuteProcess(
        cmd=[get_package_prefix('rpp_localization') + '/lib/rpp_localization/test_' + node_type + '_node_interfaces'],
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == '__main__':
    sys.exit(main())
