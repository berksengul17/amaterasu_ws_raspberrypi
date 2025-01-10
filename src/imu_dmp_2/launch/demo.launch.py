from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the included launch file
    mpu6050_launch_path = os.path.join(
        get_package_share_directory('imu_dmp_2'),
        'launch',
        'mpu6050.launch.py'
    )

    # Path to the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory('imu_dmp_2'),
        'rviz',
        'demo.rviz'
    )

    return LaunchDescription([
        # Include the MPU6050 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mpu6050_launch_path)
        ),

        # Static Transform Publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link', '100']
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path]
        )
    ])
