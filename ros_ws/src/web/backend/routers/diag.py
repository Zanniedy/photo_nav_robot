from fastapi import APIRouter
import glob, os, stat, subprocess, time
import process_mgr
from config import ROS2_SETUP, WORKSPACE_SETUP

router = APIRouter(prefix="/api/diag")

# 期望的串口设备（按 by-id 关键字识别）
_EXPECTED_DEVICES = {
    "bridge": {"hint": ["ch340", "ch341"], "desc": "下位机 (CH340)"},
    "lidar":  {"hint": ["cp210"],          "desc": "激光雷达 (CP2102)"},
}


def _find_port_by_hint(hints: list[str]) -> str | None:
    by_id = "/dev/serial/by-id"
    if not os.path.isdir(by_id):
        return None
    for name in os.listdir(by_id):
        nl = name.lower()
        if any(h in nl for h in hints):
            try:
                return os.path.realpath(os.path.join(by_id, name))
            except OSError:
                pass
    # fallback: just return first ttyUSB/ACM
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def _check_port_permission(path: str) -> dict:
    if not os.path.exists(path):
        return {"ok": False, "reason": "设备不存在"}
    try:
        s = os.stat(path)
        mode = s.st_mode
        # check group/other rw
        if mode & (stat.S_IRGRP | stat.S_IWGRP):
            return {"ok": True, "reason": "权限正常"}
        if mode & (stat.S_IROTH | stat.S_IWOTH):
            return {"ok": True, "reason": "权限正常"}
        return {"ok": False, "reason": f"无读写权限 (mode={oct(mode)[-3:]}), 请运行: sudo chmod a+rw {path}"}
    except OSError as e:
        return {"ok": False, "reason": str(e)}


def _check_ros_topic(topic: str, timeout: float = 2.0) -> dict:
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    parts.append(f"ros2 topic echo {topic} --once --no-daemon 2>&1")
    try:
        r = subprocess.run(
            ["bash", "-c", " && ".join(parts)],
            capture_output=True, text=True, timeout=timeout + 0.5
        )
        if r.returncode == 0 and "---" in r.stdout:
            return {"ok": True, "reason": "收到数据"}
        return {"ok": False, "reason": "无数据 (底盘未通信或bridge未启动)"}
    except subprocess.TimeoutExpired:
        return {"ok": False, "reason": f"超时 {timeout}s 无数据"}


@router.get("/check")
def run_diag():
    results = {}

    # 1. 串口设备检测 + 权限
    for key, dev in _EXPECTED_DEVICES.items():
        port = _find_port_by_hint(dev["hint"])
        if port is None:
            results[f"serial_{key}"] = {
                "label": dev["desc"],
                "ok": False,
                "reason": "未检测到设备，请检查USB连接",
                "port": None,
            }
        else:
            perm = _check_port_permission(port)
            results[f"serial_{key}"] = {
                "label": dev["desc"],
                "ok": perm["ok"],
                "reason": perm["reason"],
                "port": port,
            }

    # 2. 进程状态
    for name in ["robot_bridge", "lidar", "lio_slam", "slam_toolbox"]:
        running = process_mgr.is_running(name)
        results[f"proc_{name}"] = {
            "label": f"进程: {name}",
            "ok": running,
            "reason": "运行中" if running else "未启动",
        }

    # 3. 底盘通信（只在 bridge 运行时检测）
    if process_mgr.is_running("robot_bridge"):
        yaw = _check_ros_topic("/robot/yaw", timeout=2.0)
        results["chassis_yaw"] = {
            "label": "底盘 /robot/yaw",
            "ok": yaw["ok"],
            "reason": yaw["reason"],
        }
        yaw_rate = _check_ros_topic("/robot/yaw_speed", timeout=2.0)
        results["chassis_yaw_speed"] = {
            "label": "底盘 /robot/yaw_speed",
            "ok": yaw_rate["ok"],
            "reason": yaw_rate["reason"],
        }
    else:
        for k, label in [("chassis_yaw", "底盘 /robot/yaw"), ("chassis_yaw_speed", "底盘 /robot/yaw_speed")]:
            results[k] = {"label": label, "ok": False, "reason": "bridge 未启动，跳过检测"}

    all_ok = all(v["ok"] for v in results.values())
    return {"ok": all_ok, "items": results}
