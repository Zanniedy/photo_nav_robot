"""
rviz.launch.py — 实机 rviz2，静态 TF 由这里发
默认参数来自 robot_nav/config/launch_config.yaml，命令行可覆盖。
"""
import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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


def _stf(name, parent, child, xyz, rpy='0 0 0'):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{name}',
        arguments=xyz.split() + rpy.split() + [parent, child],
    )


def generate_launch_description():
    cfg      = lc_mod.load()
    rviz_cfg = lc_mod.resolve_rviz(cfg)

    tf_base        = _stf('base',        'base_footprint', 'base_link',            '0 0 0.9')
    tf_laser       = _stf('laser',       'base_link',      'laser',                '0.25 0 -0.1')
    tf_camera      = _stf('camera',      'base_link',      'camera_link',          '0.25 0 0.62')
    tf_cam_optical = _stf('cam_optical', 'camera_link',    'camera_optical_frame', '0 0 0', '-1.5708 0 -1.5708')
    tf_imu         = _stf('imu',         'base_link',      'imu_link',             '0 0 0')


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
    )

    return LaunchDescription([
        tf_base, tf_laser, tf_camera, tf_cam_optical, tf_imu,
        rviz,
    ])
