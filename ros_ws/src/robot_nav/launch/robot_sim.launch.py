"""
robot_sim.launch.py — 仿真版感知全栈
  Ignition Gazebo + RSP + bridge + lio_slam + sim_imu_bridge + lio_ekf [+ slam_toolbox]

默认参数来自 robot_nav/config/launch_config.yaml，命令行可覆盖。

建图:   ros2 launch robot_nav robot_sim.launch.py slam:=true
导航:   ros2 launch robot_nav robot_sim.launch.py slam:=false
"""
import importlib.util
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- 加载统一配置 ---
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
# --------------------


def generate_launch_description():
    cfg       = lc_mod.load()
    nav_share  = get_package_share_directory('robot_nav')
    desc_share = get_package_share_directory('photo_nav_description')

    lio_slam_cfg = os.path.join(nav_share, 'config', 'lio_slam.yaml')
    lio_ekf_cfg  = os.path.join(nav_share, 'config', 'lio_ekf.yaml')
    slam_cfg     = os.path.join(nav_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')
    slam         = LaunchConfiguration('slam')

    odom_source = cfg['robot']['odom_source']
    for arg in sys.argv:
        if arg.startswith('odom_source:='):
            odom_source = arg.split(':=', 1)[1]
            break
    use_ekf = (odom_source != 'raw')

    sim_cfg = cfg['sim']
    default_world = os.path.join(desc_share, 'worlds', sim_cfg['world'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'ignition.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world':        world,
            'launch_rviz':  'false',
        }.items(),
    )

    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        name='lio_slam_node',
        output='screen',
        parameters=[lio_slam_cfg, {
            'use_sim_time': use_sim_time,
            'publish_tf':   not use_ekf,
        }],
    )

    sim_imu_bridge = Node(
        package='robot_nav',
        executable='sim_imu_bridge_node',
        name='sim_imu_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lio_ekf = Node(
        package='robot_nav',
        executable='lio_ekf_node',
        name='lio_ekf_node',
        output='screen',
        parameters=[lio_ekf_cfg, {
            'use_sim_time': use_sim_time,
            'publish_tf':   use_ekf,
        }],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg, {'use_sim_time': use_sim_time}],
        condition=IfCondition(slam),
    )

    # yaml bool -> 'true'/'false' 小写字符串
    default_slam = 'true' if sim_cfg['slam'] else 'false'
    default_use_sim = 'true' if cfg['common']['use_sim_time'] else 'false'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value=default_use_sim),
        DeclareLaunchArgument('world',         default_value=default_world),
        DeclareLaunchArgument('odom_source',   default_value=cfg['robot']['odom_source'],
                              description='slam_toolbox 里程计来源: ekf | raw'),
        DeclareLaunchArgument('slam',          default_value=default_slam,
                              description='true=建图，false=不启动slam_toolbox'),

        ignition,
        lio_slam,
        sim_imu_bridge,
        TimerAction(period=1.0, actions=[lio_ekf]),
        TimerAction(period=3.0, actions=[slam_toolbox]),
    ])
