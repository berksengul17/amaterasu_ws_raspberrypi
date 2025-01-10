from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to the configuration file
    imu_config_file = os.path.join(
        get_package_share_directory('imu_dmp_2'),
        'config',
        'imu_config.yaml'
    )

    return LaunchDescription([
        # Node to load parameters
        Node(
            package='imu_dmp_2',
            executable='imu_publisher',
            name='mpu6050',
            parameters=[imu_config_file],
            output='screen'
        )
    ])