import os
import yaml

_cfg_path = os.environ.get(
    "ROBOT_UI_CONFIG", os.path.join(os.path.dirname(__file__), "config.yaml"))
with open(_cfg_path) as f:
    _cfg = yaml.safe_load(f)

_x = os.path.expanduser

PORT: int = int(_cfg.get("port", 8000))
ROS_DOMAIN_ID: int = int(_cfg.get("ros_domain_id", 0))
ROS2_SETUP: str = _cfg["ros2_setup"]
WORKSPACE_SETUP: str = _x(_cfg["workspace_setup"])
# 兼容旧配置的单个 maps_dir
MAP_DIRS: list[str] = [_x(d) for d in _cfg.get("map_dirs") or [_cfg.get("maps_dir", "~/maps")]]
STATE_FILE: str = _x(_cfg.get("state_file", "~/.robot_ui_state.json"))
ANNOT_DIR: str = _x(_cfg.get("annotations_dir", "~/max_robot/annotations"))
# 相机照片目录，要和 person_locator/config/camera.yaml 的 capture_dir 一致
CAPTURE_DIR: str = _x(_cfg.get("captures_dir", "~/max_robot/captures"))
LOG_BUFFER_SIZE: int = int(_cfg.get("log_buffer_size", 2000))
AUTOSTART: list[str] = list(_cfg.get("autostart", []))
# {web} → web 目录（backend 的上一级）
WEB_DIR: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROCESSES: dict[str, str] = {k: v.replace("{web}", WEB_DIR) for k, v in _cfg["processes"].items()}
ROSBRIDGE_URL: str = _cfg.get("rosbridge_url", "ws://127.0.0.1:9090")
SERIAL_DEVICES: dict[str, str] = dict(_cfg.get("serial_devices", {}))

_auth = _cfg.get("auth") or {}
AUTH_USER: str = str(_auth.get("username", "max_robot"))
AUTH_PASSWORD: str = str(_auth.get("password", ""))   # 空 = 不启用登录
SESSION_HOURS: float = float(_auth.get("session_hours", 72))
