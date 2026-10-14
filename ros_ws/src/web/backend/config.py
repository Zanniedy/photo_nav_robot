import os
import yaml

_cfg_path = os.environ.get(
    "ROBOT_UI_CONFIG", os.path.join(os.path.dirname(__file__), "config.yaml"))
with open(_cfg_path) as f:
    _cfg = yaml.safe_load(f)

PORT: int = int(_cfg.get("port", 8000))
ROS_DOMAIN_ID: int = int(_cfg.get("ros_domain_id", 0))
ROS2_SETUP: str = _cfg["ros2_setup"]
WORKSPACE_SETUP: str = os.path.expanduser(_cfg["workspace_setup"])
MAPS_DIR: str = os.path.expanduser(_cfg["maps_dir"])
LOG_BUFFER_SIZE: int = int(_cfg.get("log_buffer_size", 2000))
AUTOSTART: list[str] = list(_cfg.get("autostart", []))
PROCESSES: dict[str, str] = dict(_cfg["processes"])
SERIAL_DEVICES: dict[str, str] = dict(_cfg.get("serial_devices", {}))
