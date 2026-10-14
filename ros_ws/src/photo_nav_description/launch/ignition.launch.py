import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_sim_config(pkg_share):
    config_file = os.path.join(pkg_share, 'config', 'sim.yaml')
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)['sim']


def generate_launch_description():
    pkg_share = get_package_share_directory('photo_nav_description')
    cfg = _load_sim_config(pkg_share)

    xacro_file  = os.path.join(pkg_share, 'urdf', 'photo_nav_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    world_file  = os.path.join(pkg_share, 'worlds', cfg['world'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz  = LaunchConfiguration('launch_rviz')

    robot_description_cmd   = Command(['xacro ', xacro_file])
    robot_description_param = ParameterValue(robot_description_cmd, value_type=str)

    # 追加模型路径到 IGN_GAZEBO_RESOURCE_PATH
    models_path = cfg.get('models_path', '').strip()
    launch_env = dict(os.environ)
    if models_path:
        existing = launch_env.get('IGN_GAZEBO_RESOURCE_PATH', '')
        launch_env['IGN_GAZEBO_RESOURCE_PATH'] = (
            existing + os.pathsep + models_path if existing else models_path
        )

    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': launch_env.get('IGN_GAZEBO_RESOURCE_PATH', '')},
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'photo_nav_robot',
            '-string', robot_description_cmd,
            '-x', cfg.get('spawn_x', '0.0'),
            '-y', cfg.get('spawn_y', '0.0'),
            '-z', cfg.get('spawn_z', '0.1'),
            '-Y', cfg.get('spawn_yaw', '0.0'),
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Set false to suppress RViz'),
        ignition_gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz2,
    ])
