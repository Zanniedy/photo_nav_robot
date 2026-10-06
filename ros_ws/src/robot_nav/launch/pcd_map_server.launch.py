"""
pcd_map_server.launch.py
点云地图服务：加载已有 .pcd 文件，持续发布到 /pcd_map 话题
需要先另开终端启动仿真：
  ros2 launch photo_nav_description sim.launch.py
TODO: 后续在此加入点云匹配定位节点（pcd_localizer_node）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_nav')

    pcd_server_config = os.path.join(nav_share, 'config', 'pcd_map_server.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path     = LaunchConfiguration('map_path')

    pcd_map_server = Node(
        package='robot_nav',
        executable='pcd_map_server_node',
        name='pcd_map_server_node',
        output='screen',
        # pcd_map_server.yaml 提供基础参数，map_path 可被命令行覆盖
        parameters=[pcd_server_config, {
            'map_path':     map_path,
            'use_sim_time': use_sim_time,
        }],
    )

    # TODO: 点云匹配定位节点（帧-地图 ICP / NDT）
    # localizer = Node(
    #     package='robot_nav',
    #     executable='pcd_localizer_node',
    #     name='pcd_localizer_node',
    #     output='screen',
    #     parameters=[{
    #         'map_topic':    '/pcd_map',
    #         'scan_topic':   '/scan',
    #         'map_frame':    'map',
    #         'odom_frame':   'odom',
    #         'use_sim_time': use_sim_time,
    #     }],
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.join(nav_share, 'maps', 'map.pcd'),
            description='要加载的 .pcd 地图文件路径',
        ),
        pcd_map_server,
        # TODO: localizer,
    ])
