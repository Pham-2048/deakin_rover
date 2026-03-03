#!/usr/bin/env python3
"""
Monitoring launch — system monitor + node manager.

System monitor publishes telemetry to the GUI Status Dashboard.
Node manager provides services for the GUI to launch/stop nodes remotely.

MOCK MODE: Battery is simulated. Network and nodes are real.
HARDWARE UPGRADE: Set use_mock=false for real battery sensor.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_mock = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Mock battery sensor (network/nodes are always real)')

    # -- System Monitor --
    monitor = Node(
        package='rover_monitor',
        executable='system_monitor_node',
        name='system_monitor_node',
        parameters=[{
            'use_mock': LaunchConfiguration('use_mock'),
            'power_topic_rate_hz': 1.0,
            'network_topic_rate_hz': 2.0,
            'node_list_rate_hz': 1.0,
            'battery_cells': 6,
            'ping_host': '192.168.1.1',
        }],
        output='screen',
    )

    # -- Node Manager --
    node_manager = Node(
        package='rover_node_manager',
        executable='node_manager_node',
        name='node_manager_node',
        output='screen',
    )

    return LaunchDescription([
        use_mock,
        monitor,
        node_manager,
    ])
