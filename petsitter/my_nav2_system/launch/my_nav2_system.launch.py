import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nav2_yaml = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'my_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'my_slam.rviz')

    return LaunchDescription([
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            node_name = 'map_server',
            output = 'screen',
            parameters=[nav2_yaml,
                        {'yaml_filename':map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            node_name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
