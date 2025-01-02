from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amaterasu',
            executable='magnetometer',
            name='magnetometer',
            output='screen'
        ),

        Node(
            package='amaterasu',
            executable='imu',
            name='imu',
            output='screen'
        ),

        Node(
            package='amaterasu',
            executable='imu_mag',
            name='imu_mag',
            output='screen'
        ),
    ])
