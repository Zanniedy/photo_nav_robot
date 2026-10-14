"""
nav2.launch.py — map_server + AMCL 定位 + nav2 完整栈

默认参数来自 robot_nav/config/launch_config.yaml，命令行可覆盖。

仿真:  ros2 launch robot_nav nav2.launch.py
实机:  ros2 launch robot_nav nav2.launch.py use_sim_time:=false map:=/path/to/map.yaml

数据流:
  map_server(pgm) + /scan + lio_odom TF
    → amcl                         →  map→lio_odom TF
  /initialpose_rough (RViz) + /scan
    → relocalize_node (扫描精配)    →  /initialpose → amcl
    → nav2 (controller/planner)    →  cmd_vel

注：slam_toolbox localization 模式需要 serialize_map 产出的 .posegraph，
    maps/ 下只有 pgm/yaml，所以用 AMCL。
    velocity_smoother 不启用：底盘是开环离散指令，平滑无意义。
"""
import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml

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
    cfg      = lc_mod.load()
    nav_share = get_package_share_directory('robot_nav')

    default_map  = lc_mod.resolve_map(cfg)
    nav2_params  = os.path.join(nav_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_arg      = LaunchConfiguration('map')
    params_arg   = LaunchConfiguration('params_file')
    launch_rviz  = LaunchConfiguration('launch_rviz')
    localizer    = LaunchConfiguration('localizer')

    configured_params = RewrittenYaml(
        source_file=params_arg,
        root_key='',
        param_rewrites={
            'use_sim_time':  use_sim_time,
            'yaml_filename': map_arg,
            # 去掉 Spin/BackUp 恢复的行为树，见 xml 头注释
            'default_nav_to_pose_bt_xml': os.path.join(nav_share, 'config', 'nav_to_pose_no_spin.xml'),
        },
        convert_types=True,
    )

    def lc(pkg, exe, name):
        return LifecycleNode(
            package=pkg, executable=exe, name=name, namespace='',
            output='screen', parameters=[configured_params],
        )

    map_server = lc('nav2_map_server',        'map_server',        'map_server')
    controller = lc('nav2_controller',        'controller_server', 'controller_server')
    planner    = lc('nav2_planner',           'planner_server',    'planner_server')
    smoother   = lc('nav2_smoother',          'smoother_server',   'smoother_server')
    behavior   = lc('nav2_behaviors',         'behavior_server',   'behavior_server')
    bt_nav     = lc('nav2_bt_navigator',      'bt_navigator',      'bt_navigator')
    waypoint   = lc('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower')
    amcl       = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl', namespace='',
        output='screen', parameters=[configured_params],
        condition=LaunchConfigurationEquals('localizer', 'amcl'),
    )

    # scan-to-map 定位：替代 AMCL 发布 map→lio_odom（不是 lifecycle 节点）
    map_localizer = Node(
        package='robot_nav', executable='map_localizer_node', name='map_localizer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('localizer', 'scan_match'),
    )

    def localization_manager(nodes, value):
        return Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    True,
                'node_names':   nodes,
            }],
            condition=LaunchConfigurationEquals('localizer', value),
        )

    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   [
                'controller_server', 'planner_server', 'smoother_server',
                'behavior_server', 'bt_navigator', 'waypoint_follower',
            ],
        }],
    )

    # RViz 2D Pose Estimate → /initialpose_rough → 扫描精配 → /initialpose → AMCL
    relocalize = Node(
        package='robot_nav', executable='relocalize_node.py', name='relocalize_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'pose_topic': PythonExpression(["'/amcl_pose' if '", localizer, "' == 'amcl' else '/localizer/pose'"])}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    default_use_sim  = 'true' if cfg['common']['use_sim_time'] else 'false'
    default_rviz     = 'true' if cfg['nav2']['launch_rviz'] else 'false'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value=default_use_sim),
        DeclareLaunchArgument('map',           default_value=default_map,
                              description='地图 yaml 路径'),
        DeclareLaunchArgument('params_file',   default_value=nav2_params,
                              description='nav2 参数文件路径'),
        DeclareLaunchArgument('launch_rviz',   default_value=default_rviz,
                              description='是否启动 rviz2'),
        DeclareLaunchArgument('localizer',     default_value='scan_match',
                              description='scan_match（map_localizer_node）| amcl'),

        map_server, amcl, map_localizer, relocalize,
        localization_manager(['map_server', 'amcl'], 'amcl'),
        localization_manager(['map_server'], 'scan_match'),
        controller, planner, smoother, behavior, bt_nav, waypoint,
        TimerAction(period=5.0, actions=[navigation_manager]),
        rviz,
    ])
