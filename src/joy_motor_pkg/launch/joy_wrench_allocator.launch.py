import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('joy_motor_pkg'),
        'config',
        'joy_wrench_allocator.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='joy_motor_pkg',
                executable='joy_to_wrench',
                name='joy_to_wrench_node',
                parameters=[config_path],
                output='screen',
            ),
            Node(
                package='joy_motor_pkg',
                executable='thruster_allocator',
                name='thruster_allocator_node',
                parameters=[config_path],
                output='screen',
            ),
        ]
    )
