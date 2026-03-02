import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_arm_description = get_package_share_directory('arm_description')

    urdf_xacro_file = os.path.join(pkg_arm_description, 'urdf', 'arm_controlled.urdf.xacro')
    rviz_config = os.path.join(pkg_arm_description, 'config', 'view_arm.rviz')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_xacro_file, ' use_ros2_control:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
