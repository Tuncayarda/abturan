import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('joy_motor_pkg'),
        'config',
        'minirov_joy.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='joy_motor_pkg',
                executable='minirov_joy',
                name='minirov_joy_node',
                parameters=[config_path],
                output='screen',
            ),
        ]
    )
