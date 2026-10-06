"""
pcd_map.launch.py
点云建图：lio_ekf（接管 TF）+ slam_toolbox（提供 map->lio_odom）+ pcd_map_node

前提：sim.launch.py 已在另一个终端运行，且关闭了 lio_slam 的 TF 输出。

正确启动顺序：
  # 终端1：启动仿真，lio_slam 不发 TF
  ros2 launch photo_nav_description sim.launch.py slam_publish_tf:=false

  # 终端2：启动点云建图
  ros2 launch robot_nav pcd_map.launch.py
  Ctrl+C 触发 pcd_map_node 保存 .pcd 文件
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_nav')

    lio_ekf_config = os.path.join(nav_share, 'config', 'lio_ekf.yaml')
    slam_config    = os.path.join(nav_share, 'config', 'slam_toolbox.yaml')
    pcd_map_config = os.path.join(nav_share, 'config', 'pcd_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_path  = LaunchConfiguration('output_path')

    # EKF 接管 lio_odom->base_footprint TF
    lio_ekf = Node(
        package='robot_nav',
        executable='lio_ekf_node',
        name='lio_ekf_node',
        output='screen',
        parameters=[lio_ekf_config, {
            'use_sim_time': use_sim_time,
            'publish_tf':   True,
        }],
    )

    # slam_toolbox 提供 map->lio_odom TF，供 pcd_map_node 做坐标变换
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    pcd_map = Node(
        package='robot_nav',
        executable='pcd_map_node',
        name='pcd_map_node',
        output='screen',
        parameters=[pcd_map_config, {
            'output_path':  output_path,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'output_path',
            default_value=os.path.join(nav_share, 'maps', 'map.pcd'),
            description='Ctrl+C 触发保存的 .pcd 文件路径',
        ),
        lio_ekf,
        slam_toolbox,
        pcd_map,
    ])
