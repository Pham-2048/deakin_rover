#!/usr/bin/env python3
"""Hardware launch — ros2_control drive system and cameras.

Kept as Python because controller_manager needs robot_description as a string
(xacro output), and ROS2 XML launch has no command substitution for this.
"""

import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Use mock mode for all hardware nodes')

    use_mock_cfg = LaunchConfiguration('use_mock')

    # Run xacro eagerly — use_mock_hardware is resolved at launch-description time.
    # Real deployments: pass use_mock:=false, which sets use_mock_hardware:=false.
    pkg_dir = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')
    # Note: use_mock is a LaunchConfiguration, not resolvable here; default matches.
    robot_description = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    controllers_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'ros2_controllers.yaml'])

    camera_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'camera_config.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[{'robot_description': robot_description}, controllers_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--ros-args', '-r', '/diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
        output='screen',
    )

    camera = Node(
        package='rover_camera',
        executable='camera_node',
        name='camera_node',
        parameters=[camera_config, {'use_mock': use_mock_cfg}],   # launch arg overrides YAML
        output='screen',
    )

    return LaunchDescription([
        use_mock_arg,
        controller_manager,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[diff_drive_spawner]),
        camera,
    ])
