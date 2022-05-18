from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='petsitter_slam_auto',
            executable='slam_server',
            output='screen'
        )
    ])