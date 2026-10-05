import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('photo_nav_description')
    nav_share  = get_package_share_directory('robot_nav')

    slam_config     = os.path.join(nav_share, 'config', 'slam_toolbox.yaml')
    lio_config      = os.path.join(nav_share, 'config', 'lio_slam.yaml')
    rviz_config     = os.path.join(desc_share, 'rviz', 'robot.rviz')
    default_map_out = os.path.join(nav_share, 'maps', 'map.pcd')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')
    output_path  = LaunchConfiguration('output_path')

    # ── Gazebo + RSP + bridge（复用已有 launch）──────────────────────────────
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'ignition.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'launch_rviz': 'false',
        }.items(),
    )

    # ── LIO scan-matching odometry ──────────────────────────────────────────
    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        name='lio_slam_node',
        output='screen',
        parameters=[lio_config, {'use_sim_time': use_sim_time}],
    )

    # ── slam_toolbox 2D 栅格建图 ────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    # ── PCD 点云地图构建（Ctrl+C 触发保存）──────────────────────────────────
    pcd_map = Node(
        package='robot_nav',
        executable='pcd_map_node',
        name='pcd_map_node',
        output='screen',
        parameters=[{
            'map_frame':    'map',
            'output_path':  output_path,
            'voxel_size':   0.05,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── odom → Path 转发（供 RViz 线条显示）────────────────────────────────
    odom_path = Node(
        package='robot_nav',
        executable='odom_path_node',
        name='odom_path_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── LIO + IMU EKF（无轮式里程计，实车通用）─────────────────────────────
    lio_ekf = Node(
        package='robot_nav',
        executable='lio_ekf_node',
        name='lio_ekf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(desc_share, 'worlds', 'obstacle_world.sdf'),
        ),
        DeclareLaunchArgument(
            'output_path',
            default_value=default_map_out,
            description='PCD map output path',
        ),
        ignition,
        lio_slam,
        slam_toolbox,
        pcd_map,
        odom_path,
        lio_ekf,
        rviz,
    ])
