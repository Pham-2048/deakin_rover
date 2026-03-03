#!/usr/bin/env python3
"""
Hardware launch — CAN bridge, RS485 bridge, cameras.

All hardware nodes support use_mock=true for testing without physical hardware.

MOCK MODE (Orange Pi): All nodes simulate hardware responses.
HARDWARE UPGRADE: Set use_mock=false in the respective config or here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Use mock mode for all hardware nodes')

    # Config files
    camera_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'camera_config.yaml'])
    arm_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'arm_motors.yaml'])
    drive_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'drive_params.yaml'])

    # -- Camera Node --
    camera = Node(
        package='rover_camera',
        executable='camera_node',
        name='camera_node',
        parameters=[camera_config],
        output='screen',
    )

    # -- CAN Bridge (arm motors) --
    can_bridge = Node(
        package='rover_can',
        executable='can_bridge_node',
        name='can_bridge_node',
        parameters=[{
            'use_mock': LaunchConfiguration('use_mock'),
            'can_interface': 'can0',
            'feedback_rate_hz': 100.0,
            'status_rate_hz': 2.0,
        }],
        output='screen',
    )

    # -- RS485 Bridge (drive motors) --
    rs485_bridge = Node(
        package='rover_serial',
        executable='rs485_bridge_node',
        name='rs485_bridge_node',
        parameters=[{
            'use_mock': LaunchConfiguration('use_mock'),
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 9600,
            'max_rpm': 3000,
        }],
        output='screen',
    )

    return LaunchDescription([
        use_mock,
        camera,
        can_bridge,
        rs485_bridge,
    ])
