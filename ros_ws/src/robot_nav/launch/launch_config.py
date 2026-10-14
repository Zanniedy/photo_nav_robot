"""
launch_config.py — 统一读取 robot_nav/config/launch_config.yaml
用 importlib 从绝对路径加载，不依赖 Python path。

在 launch 文件里这样使用：
    import importlib.util, os
    from ament_index_python.packages import get_package_share_directory
    _spec = importlib.util.spec_from_file_location(
        "launch_config",
        os.path.join(get_package_share_directory('robot_nav'), 'launch', 'launch_config.py')
    )
    lc_mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(lc_mod)
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load() -> dict:
    nav_share   = get_package_share_directory('robot_nav')
    config_file = os.path.join(nav_share, 'config', 'launch_config.yaml')
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def resolve_map(cfg: dict) -> str:
    map_val = cfg['nav2']['map']
    if os.path.isabs(map_val):
        return map_val
    nav_share = get_package_share_directory('robot_nav')
    return os.path.join(nav_share, 'maps', map_val)


def resolve_rviz(cfg: dict) -> str:
    rviz_val = cfg['rviz']['config']
    if os.path.isabs(rviz_val):
        return rviz_val
    nav_share = get_package_share_directory('robot_nav')
    return os.path.join(nav_share, 'config', rviz_val)
