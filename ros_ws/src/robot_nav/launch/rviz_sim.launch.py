"""
rviz_sim.launch.py — 仿真专用 rviz2
默认参数来自 robot_nav/config/launch_config.yaml，命令行可覆盖。
"""
import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _load_lc():
    nav_share = get_package_share_directory('robot_nav')
    spec = importlib.util.spec_from_file_location(
        "launch_config",
        os.path.join(nav_share, 'launch', 'launch_config.py')
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

lc_mod = _load_lc()


def generate_launch_description():
    cfg      = lc_mod.load()
    rviz_cfg = lc_mod.resolve_rviz(cfg)

    use_sim_time    = LaunchConfiguration('use_sim_time')
    default_use_sim = 'true' if cfg['common']['use_sim_time'] else 'false'

    odom_path = Node(
        package='robot_nav',
        executable='odom_path_node',
        name='odom_path_node',
        output='screen',
        parameters=[{'map_frame': 'map', 'max_poses': 5000, 'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=default_use_sim),
        odom_path,
        rviz,
    ])
