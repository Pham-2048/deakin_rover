#!/usr/bin/env python3
"""
Infrastructure launch — rosbridge, web_video_server, TF.

Provides GUI connectivity:
  - rosbridge_server on port 9090 (for custom Next.js GUI via roslib)
  - web_video_server on port 8080 (MJPEG camera streams for GUI)
  - robot_state_publisher + joint_state_publisher (TF tree)

MOCK MODE: Fully functional on Orange Pi.
HARDWARE UPGRADE: No changes needed.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Arguments --
    rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port', default_value='9090',
        description='WebSocket port for rosbridge (GUI uses this)')

    video_port = DeclareLaunchArgument(
        'video_port', default_value='8080',
        description='HTTP port for web_video_server (camera MJPEG streams)')

    # -- rosbridge_server (primary GUI connection) --
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
            'address': '',
            'retry_startup_delay': 5.0,
        }],
        output='screen',
    )

    # -- web_video_server (camera MJPEG streams) --
    # If ros-jazzy-web-video-server is not installed, this node will fail to start.
    # See implementation.md for installation instructions.
    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': LaunchConfiguration('video_port'),
            'address': '0.0.0.0',
            'server_threads': 2,
            'ros_threads': 2,
        }],
        output='screen',
    )

    # -- Robot description (TF tree) --
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_description'),
                'launch',
                'description.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        rosbridge_port,
        video_port,
        rosbridge,
        web_video,
        description_launch,
    ])
