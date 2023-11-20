import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    SENSOR_node = Node(
        package='bioles_robot',
        executable='serial_left',
        name = 'sensor1',
        output = 'screen'
    )
    SENSOR2_node = Node(
        package='bioles_robot',
        executable='serial_right',
        name = 'sensor2',
        output = 'screen'
    )
    AHRS_node = Node(
        package='bioles_robot',
        executable='imu_filtering',
        name = 'filtered_imu',
        output = 'screen'
    )
    BALANCING_node = Node(
        package='bioles_robot',
        executable='balancing',
        name='balancing_control',
        output='screen'
    )
    JUMPING_node = Node(
        package='bioles_robot',
        executable='jumping',
        name='jumping_control',
        output='screen'
    )

    return LaunchDescription([
        SENSOR_node, AHRS_node, SENSOR2_node, #BALANCING_node, JUMPING_node
    ])