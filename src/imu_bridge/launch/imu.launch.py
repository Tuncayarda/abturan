import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('imu_bridge'),
        'config',
        'imu.yaml',
    )

    return LaunchDescription([
        Node(
            package='imu_bridge',
            executable='imu_node',
            name='imu_node',
            parameters=[config_path],
            output='screen',
        ),
    ])
