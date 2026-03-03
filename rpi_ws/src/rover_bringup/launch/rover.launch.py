#!/usr/bin/env python3
"""
Master launch file — starts the entire Deakin Rover system.

Launches 4 layers in order:
  1. Infrastructure: rosbridge, Foxglove, web_video_server, TF
  2. Hardware: CAN bridge, RS485 bridge, cameras
  3. Control: e-stop, drive mixer, arm controller, joystick
  4. Monitoring: system monitor, node manager

USAGE:
  # Default (mock mode for Orange Pi testing):
  ros2 launch rover_bringup rover.launch.py

  # Explicit mock mode:
  ros2 launch rover_bringup rover.launch.py use_mock:=true

  # Real hardware (on Jetson/RPi with motors and cameras):
  ros2 launch rover_bringup rover.launch.py use_mock:=false

  # Infrastructure only (for GUI connection testing):
  ros2 launch rover_bringup infrastructure.launch.py

PORTS:
  9090 — rosbridge WebSocket (Next.js GUI connects here)
  8765 — Foxglove Bridge WebSocket (Foxglove Studio backup GUI)
  8080 — web_video_server HTTP (camera MJPEG streams)

FOXGLOVE:
  Open Foxglove Studio → "Open connection" → ws://<rover_ip>:8765
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Arguments --
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Use mock hardware mode (true for Orange Pi, false for real rover)')

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port', default_value='9090',
        description='rosbridge WebSocket port')

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='Foxglove Bridge WebSocket port')

    video_port_arg = DeclareLaunchArgument(
        'video_port', default_value='8080',
        description='web_video_server HTTP port')

    pkg = FindPackageShare('rover_bringup')

    # Layer 1: Infrastructure
    infrastructure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'infrastructure.launch.py'])
        ]),
        launch_arguments={
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            'foxglove_port': LaunchConfiguration('foxglove_port'),
            'video_port': LaunchConfiguration('video_port'),
        }.items(),
    )

    # Layer 2: Hardware
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
        }.items(),
    )

    # Layer 3: Control
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'control.launch.py'])
        ]),
    )

    # Layer 4: Monitoring
    monitoring = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'monitoring.launch.py'])
        ]),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
        }.items(),
    )

    return LaunchDescription([
        use_mock_arg,
        rosbridge_port_arg,
        foxglove_port_arg,
        video_port_arg,
        infrastructure,
        hardware,
        control,
        monitoring,
    ])
