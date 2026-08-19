import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('stm32_bridge'),
        'config',
        'stm32_bridge.yaml',
    )

    return LaunchDescription([
        Node(
            package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            parameters=[config_path],
            output='screen',
        ),
    ])
