import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    AHRS_node = Node(
        package='bioles_gazebo',
        executable='imu_filtering',
        name = 'filtered_imu',
        output = 'screen'
    )
    MPC_node = Node(
        package='bioles_gazebo',
        executable='mpc_calc',
        name='mpc',
        output='screen'
    )

    return LaunchDescription([
        AHRS_node, MPC_node
    ])