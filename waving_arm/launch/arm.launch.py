import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    pkg_share = get_package_share_directory('waving_arm')

    xacro_file = os.path.join(pkg_share, 'urdf/arm.xacro')

    robot_description_raw = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': False}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config/view_arm.rviz')]
        ),

        Node(
            package='waving_arm',
            executable='arm_waving_node',
            name='arm_waving_node'
        )
    ])