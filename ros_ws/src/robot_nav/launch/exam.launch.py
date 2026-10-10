"""
exam.launch.py — 本地完整建图测试，一键启动：
  robot_state_publisher / robot_bridge / sllidar / lio_slam / lio_ekf / slam_toolbox / rviz2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share  = get_package_share_directory('robot_nav')
    desc_share = get_package_share_directory('photo_nav_description')
    bridge_share = get_package_share_directory('robot_bridge')

    urdf_file    = os.path.join(desc_share, 'urdf', 'photo_nav_robot.urdf.xacro')
    lio_slam_cfg = os.path.join(nav_share,  'config', 'lio_slam.yaml')
    lio_ekf_cfg  = os.path.join(nav_share,  'config', 'lio_ekf.yaml')
    slam_cfg     = os.path.join(nav_share,  'config', 'slam_toolbox.yaml')
    bridge_cfg   = os.path.join(bridge_share, 'config', 'serial.yaml')
    rviz_cfg     = os.path.join(nav_share,  'config', 'mapping.rviz')

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baudrate')
    lidar_port  = LaunchConfiguration('lidar_port')
    lidar_baud  = LaunchConfiguration('lidar_baudrate')

    # ── robot_state_publisher ────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': False,
        }],
    )

    # ── robot_bridge (下位机串口) ─────────────────────────────────────────────
    bridge = Node(
        package='robot_bridge',
        executable='robot_bridge_node',
        name='robot_bridge_node',
        output='screen',
        parameters=[bridge_cfg],
    )

    # ── sllidar ───────────────────────────────────────────────────────────────
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port':     lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id':        'laser',
            'angle_compensate': True,
            'scan_mode':       'Standard',
        }],
    )

    # ── lio_slam (ICP 里程计，不发 TF) ────────────────────────────────────────
    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        name='lio_slam_node',
        output='screen',
        parameters=[lio_slam_cfg, {'use_sim_time': False}],
    )

    # ── lio_ekf (EKF 融合，发 lio_odom->base_footprint TF) ───────────────────
    lio_ekf = Node(
        package='robot_nav',
        executable='lio_ekf_node',
        name='lio_ekf_node',
        output='screen',
        parameters=[lio_ekf_cfg, {
            'use_sim_time': False,
            'publish_tf':   True,
            'yaw_speed_topic': '/robot/yaw_speed',
        }],
    )

    # ── slam_toolbox 建图 ─────────────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg, {'use_sim_time': False}],
    )

    # ── rviz2 ─────────────────────────────────────────────────────────────────
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port',     default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_port',      default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('lidar_baudrate',  default_value='460800'),

        rsp,
        bridge,
        lidar,
        lio_slam,
        # EKF 和 slam_toolbox 延迟 2 秒等 lio_slam 先发布 /lio_odom
        TimerAction(period=2.0, actions=[lio_ekf]),
        TimerAction(period=3.0, actions=[slam_toolbox]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
