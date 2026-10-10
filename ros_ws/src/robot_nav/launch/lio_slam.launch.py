import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_nav')
    desc_share = get_package_share_directory('photo_nav_description')
    config    = os.path.join(pkg_share, 'config', 'lio_slam.yaml')
    urdf_file = os.path.join(desc_share, 'urdf', 'photo_nav_robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        name='lio_slam_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        lio_slam,
    ])
