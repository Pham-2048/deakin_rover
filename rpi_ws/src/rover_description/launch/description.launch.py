#!/usr/bin/env python3
"""Launch robot_state_publisher with the rover URDF."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/arm/joint_states'],
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
    ])
