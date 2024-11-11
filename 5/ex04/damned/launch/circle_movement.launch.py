import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('damned'), 'launch'),
            '/diff_drive.launch.py']),
       )

    return LaunchDescription([
        demo_nodes,
        DeclareLaunchArgument(
            'radius', default_value='1',
            description='Radius.'
        ),
        Node(
            package='damned',
            executable='circle_movement',
            name='circle_movement',
            arguments=[
                LaunchConfiguration('radius')
            ]
        ),
    ])
