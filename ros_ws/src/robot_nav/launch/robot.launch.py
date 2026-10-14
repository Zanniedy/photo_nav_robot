"""
robot.launch.py — 实机感知全栈（无 rviz）
  static_tf / robot_bridge / sllidar / lio_slam / lio_ekf [/ slam_toolbox]

建图:  ros2 launch robot_nav robot.launch.py slam:=true
导航:  ros2 launch robot_nav robot.launch.py slam:=false   # 再另开 nav2.launch.py
       导航时不能起 slam_toolbox，否则它和 AMCL 都发 map→lio_odom

默认参数来自 robot_nav/config/launch_config.yaml，命令行可覆盖。
"""
import importlib.util
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
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
    cfg          = lc_mod.load()
    nav_share    = get_package_share_directory('robot_nav')
    bridge_share = get_package_share_directory('robot_bridge')

    lio_slam_cfg = os.path.join(nav_share,    'config', 'lio_slam.yaml')
    lio_ekf_cfg  = os.path.join(nav_share,    'config', 'lio_ekf.yaml')
    slam_cfg     = os.path.join(nav_share,    'config', 'slam_toolbox.yaml')
    bridge_cfg   = os.path.join(bridge_share, 'config', 'serial.yaml')

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baudrate')

    robot_cfg   = cfg['robot']
    odom_source = robot_cfg['odom_source']
    for arg in sys.argv:
        if arg.startswith('odom_source:='):
            odom_source = arg.split(':=', 1)[1]
            break
    use_ekf = (odom_source != 'raw')

    def static_tf(name, parent, child, xyz, rpy='0 0 0'):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{name}',
            arguments=xyz.split() + rpy.split() + [parent, child],
        )

    tf_base        = static_tf('base',        'base_footprint', 'base_link',            '0 0 0.9')
    # 雷达 x 轴朝车体左侧：2026-10-14 实测 Vy+(前进) 时 scan 匹配位移在雷达系 -y 方向，故 yaw=+90°
    # （old-style 参数顺序 x y z yaw pitch roll）
    tf_laser       = static_tf('laser',       'base_link',      'laser',                '0.25 0 -0.1', '1.5708 0 0')
    tf_camera      = static_tf('camera',      'base_link',      'camera_link',          '0.25 0 0.6')
    tf_cam_optical = static_tf('cam_optical', 'camera_link',    'camera_optical_frame', '0 0 0', '-1.5708 0 -1.5708')
    tf_imu         = static_tf('imu',         'base_link',      'imu_link',             '0 0 0')

    bridge = Node(
        package='robot_bridge',
        executable='robot_bridge_node',
        output='screen',
        parameters=[{'config_path': bridge_cfg}],
    )

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port':      lidar_port,
            'serial_baudrate':  lidar_baud,
            'frame_id':         'laser',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        output='screen',
        parameters=[lio_slam_cfg, {
            'use_sim_time': False,
            'publish_tf':   not use_ekf,
        }],
    )

    lio_ekf = Node(
        package='robot_nav',
        executable='lio_ekf_node',
        output='screen',
        parameters=[lio_ekf_cfg, {
            'use_sim_time': False,
            'publish_tf':   use_ekf,
        }],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_cfg, {'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port',     default_value=robot_cfg['lidar_port']),
        DeclareLaunchArgument('lidar_baudrate', default_value=str(robot_cfg['lidar_baudrate'])),
        DeclareLaunchArgument('odom_source',    default_value=robot_cfg['odom_source'],
                              description='slam_toolbox 里程计来源: ekf | raw'),
        DeclareLaunchArgument('slam',           default_value='true',
                              description='true=建图(起 slam_toolbox)，false=导航(配合 nav2.launch.py)'),

        tf_base, tf_laser, tf_camera, tf_cam_optical, tf_imu,
        bridge,
        lidar,
        lio_slam,
        TimerAction(period=1.0, actions=[lio_ekf]),
        TimerAction(period=3.0, actions=[slam_toolbox]),
    ])
