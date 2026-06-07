from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share_dir = get_package_share_directory('bluerov2_navigation')
    params = os.path.join(share_dir, 'config', 'thruster_allocator.yaml')

    return LaunchDescription([
        Node(
            package='bluerov2_navigation',
            executable='thruster_allocator',
            namespace='bluerov2',
            name='thruster_allocator',
            output='screen',
            parameters=[params],
        ),
    ])
