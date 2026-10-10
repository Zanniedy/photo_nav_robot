import os
import yaml

_cfg_path = os.path.join(os.path.dirname(__file__), "config.yaml")
with open(_cfg_path) as f:
    _cfg = yaml.safe_load(f)

ROBOT_IP: str = _cfg["robot_ip"]
ROS_DOMAIN_ID: int = int(_cfg["ros_domain_id"])
MAPS_DIR: str = os.path.expanduser(_cfg["maps_dir"])
ROS2_SETUP: str = _cfg["ros2_setup"]
WORKSPACE_SETUP: str = os.path.expanduser(_cfg["workspace_setup"])
LOG_BUFFER_SIZE: int = int(_cfg["log_buffer_size"])
LIDAR_SERIAL_PORT: str = _cfg.get("lidar_serial_port", "/dev/ttyUSB0")
LIDAR_BAUDRATE: int = int(_cfg.get("lidar_baudrate", 460800))
