import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_nav')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_path  = LaunchConfiguration('output_path')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    pcd_map_saver = Node(
        package='robot_nav',
        executable='pcd_map_saver',
        name='pcd_map_saver',
        output='screen',
        parameters=[{
            'map_frame':   'map',
            'output_path': output_path,
            'min_points':  5000,
            'voxel_size':  0.05,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('output_path',
            default_value=os.path.join(pkg_share, 'maps', 'map.pcd'),
            description='Output .pcd file path'),
        slam_toolbox,
        pcd_map_saver,
    ])
