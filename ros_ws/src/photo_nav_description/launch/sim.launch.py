"""
sim.launch.py
纯仿真环境：Gazebo + RSP + bridge + lio_slam（可选发 TF）+ RViz2

默认：lio_slam_node 发 lio_odom->base_footprint TF（纯仿真时必需）。
与建图联用时：
  ros2 launch photo_nav_description sim.launch.py slam_publish_tf:=false
  ros2 launch robot_nav mapping.launch.py          # lio_ekf 发 TF
这样只有一个节点发该 TF，避免双源竞争抖动。

参数：
  use_sim_time      true/false
  world             Gazebo world 文件路径
  launch_rviz       true/false
  slam_publish_tf   lio_slam_node 是否发 TF（联合建图时设 false）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('photo_nav_description')
    nav_share  = get_package_share_directory('robot_nav')

    xacro_file      = os.path.join(desc_share, 'urdf', 'photo_nav_robot.urdf.xacro')
    rviz_config     = os.path.join(desc_share, 'rviz', 'robot.rviz')
    lio_slam_config = os.path.join(nav_share,  'config', 'lio_slam.yaml')

    use_sim_time    = LaunchConfiguration('use_sim_time')
    world           = LaunchConfiguration('world')
    launch_rviz     = LaunchConfiguration('launch_rviz')
    slam_publish_tf = LaunchConfiguration('slam_publish_tf')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'photo_nav_robot',
            '-string', Command(['xacro ', xacro_file]),
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_sensors',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # lio_slam：发布 /lio_odom 话题
    # slam_publish_tf=true（默认）：纯仿真时由它提供 lio_odom->base_footprint TF
    # slam_publish_tf=false：联合建图时关闭，让 lio_ekf_node 发 TF，避免双源竞争
    lio_slam = Node(
        package='robot_nav',
        executable='lio_slam_node',
        name='lio_slam_node',
        output='screen',
        parameters=[lio_slam_config, {
            'use_sim_time': use_sim_time,
            'publish_tf':   slam_publish_tf,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(desc_share, 'worlds', 'obstacle_world.sdf'),
            description='Gazebo world 文件路径',
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='是否启动 RViz2',
        ),
        DeclareLaunchArgument(
            'slam_publish_tf', default_value='true',
            description='lio_slam_node 是否发 TF；联合建图时设 false',
        ),
        gazebo,
        rsp,
        spawn,
        bridge,
        lio_slam,
        rviz,
    ])
