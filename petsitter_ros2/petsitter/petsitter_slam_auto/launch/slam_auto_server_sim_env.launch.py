#slam_auto_server_sim_env.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_dir = os.path.join(get_package_share_directory('petsitter_slam'), 'rviz', 'slam_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', get_package_share_directory('petsitter_slam') + '/config',
                '-configuration_basename','petsitter_slam.lua'
            ],
        ),          
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['-resolution','0.05','-publish_period_sec','1.0']
        ),
        Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
        ),
        Node(
            package='petsitter_slam_auto',
            executable='slam_server',
            output='screen'
        )
    ])