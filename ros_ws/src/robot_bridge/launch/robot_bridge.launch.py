from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_bridge"), "config", "serial.yaml"]
        ),
        description="Path to serial config YAML",
    )

    robot_bridge_node = Node(
        package="robot_bridge",
        executable="robot_bridge_node",
        name="robot_bridge_node",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("config_path")}],
    )

    return LaunchDescription([
        config_path_arg,
        robot_bridge_node,
    ])
