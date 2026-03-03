#!/usr/bin/env python3
"""
Control launch — e-stop, drive mixer, arm controller, joystick teleop.

E-stop launches first as it's the highest-priority safety node.
All control nodes check /estop/active before publishing commands.

MOCK MODE: Fully functional on Orange Pi.
HARDWARE UPGRADE: No changes needed — control nodes are hardware-independent.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'config', 'drive_params.yaml'])

    # -- E-Stop (must start first) --
    estop = Node(
        package='rover_estop',
        executable='estop_node',
        name='estop_node',
        parameters=[{
            'watchdog_timeout_sec': 5.0,   # Generous timeout for testing
            'require_manual_reset': True,
        }],
        output='screen',
    )

    # -- Drive Mixer (cmd_vel → motor speeds) --
    drive = Node(
        package='rover_drive',
        executable='drive_node',
        name='drive_node',
        parameters=[drive_config],
        output='screen',
    )

    # -- Arm Controller --
    arm = Node(
        package='rover_arm',
        executable='arm_controller_node',
        name='arm_controller_node',
        parameters=[{
            'velocity_scale': 0.5,
            'publish_rate_hz': 50.0,
        }],
        output='screen',
    )

    # -- Joystick Teleop --
    joy_teleop = Node(
        package='rover_joy',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        parameters=[{
            'linear_axis': 1,
            'angular_axis': 0,
            'linear_scale': 1.0,
            'angular_scale': 2.0,
            'mode_button': 0,
            'deadzone': 0.1,
            'arm_speed_scale': 0.5,
        }],
        output='screen',
    )

    return LaunchDescription([
        estop,
        drive,
        arm,
        joy_teleop,
    ])
