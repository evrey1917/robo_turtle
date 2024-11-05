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
            get_package_share_directory('evro_carrot'), 'launch'),
            '/mini.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    return LaunchDescription([
        demo_nodes,
        DeclareLaunchArgument(
            'radius', default_value='3',
            description='Radius.'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation', default_value='1',
            description='Direction of rotation 1 or -1.'
        ),
        Node(
            package='evro_carrot',
            executable='dynamic_frame_broadcaster',
            name='dynamic_broadcaster',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        ),
    ])
