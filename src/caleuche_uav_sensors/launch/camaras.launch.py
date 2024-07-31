import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='caleuche_uav_sensors',
            executable='forward_camera_node',
            name='forward_camera_node',
            output='screen',
        ),

        Node(
            package='caleuche_uav_sensors',
            executable='downward_camera_node',
            name='downward_camera_node',
            output='screen',
        ),
    ])