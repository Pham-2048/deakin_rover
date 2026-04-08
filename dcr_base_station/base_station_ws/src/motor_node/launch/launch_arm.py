import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = os.path.join(get_package_share_directory('motor_node'), 'urdf', 'arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ld.add_action(DeclareLaunchArgument(
        'ee_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for end effector ESP32'
    ))

    # Robot state publisher — converts /joint_states to /tf for Foxglove visualisation
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    ))

    # Joystick node
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        #parameters=[{'autorepeat_rate': 0.0}],
        output='screen'
    ))

    # SocketCAN bridge
    ld.add_action(Node(
        package='nobleo_socketcan_bridge',
        executable='socketcan_bridge',
        name='socketcan_bridge',
        output='screen',
        parameters=[{'interface': 'can1'}]
    ))

    # Motor controller
    ld.add_action(Node(
        package='motor_node',
        executable='start_controller',
        name='motor_controller',
        parameters=[{'ee_serial_port': LaunchConfiguration('ee_serial_port')}],
        output='screen'
    ))

    return ld
