"""
map_server.launch.py — 单独发布已有栅格地图到 /map topic

用法:
  ros2 launch robot_nav map_server.launch.py
  ros2 launch robot_nav map_server.launch.py map:=/path/to/other_map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_nav')
    default_map = os.path.join(nav_share, 'maps', 'map_20261012_112835.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time':  use_sim_time,
        }],
    )

    # 自动 configure + activate，省去手动 lifecycle set
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'autostart':     True,
            'node_names':    ['map_server'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='地图 yaml 路径'),
        map_server,
        lifecycle_manager,
    ])
