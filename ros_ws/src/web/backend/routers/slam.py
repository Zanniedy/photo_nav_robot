from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import subprocess, os, time
import process_mgr
from config import ROS2_SETUP, WORKSPACE_SETUP, ROS_DOMAIN_ID, MAPS_DIR, LIDAR_SERIAL_PORT, LIDAR_BAUDRATE

router = APIRouter(prefix="/api/slam")


class SaveRequest(BaseModel):
    name: str


def _wait_topic(topic: str, timeout: float = 8.0) -> bool:
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    parts.append(f"ros2 topic echo {topic} --once --no-daemon 2>&1")
    try:
        r = subprocess.run(
            ["bash", "-c", " && ".join(parts)],
            capture_output=True, text=True, timeout=timeout + 1
        )
        return r.returncode == 0 and "---" in r.stdout
    except subprocess.TimeoutExpired:
        return False


@router.post("/start")
def slam_start():
    process_mgr.stop_process("map_server")

    # 阶段1: 启动 robot_bridge，等底盘通信
    process_mgr.start_process("robot_bridge",
        "ros2 launch robot_bridge robot_bridge.launch.py")
    time.sleep(1.5)
    if not _wait_topic("/robot/yaw", timeout=8.0):
        process_mgr.stop_process("robot_bridge")
        raise HTTPException(503, "底盘通信超时：未收到 /robot/yaw，请检查串口连接和下位机电源")

    # 阶段2: 启动雷达
    process_mgr.start_process("lidar",
        f"ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:={LIDAR_SERIAL_PORT} serial_baudrate:={LIDAR_BAUDRATE}")
    time.sleep(1.5)

    # 阶段3: LIO SLAM + EKF
    process_mgr.start_process("lio_slam",
        "ros2 launch robot_nav lio_slam.launch.py use_sim_time:=false")

    # 阶段4: slam_toolbox 建图
    process_mgr.start_process("slam_toolbox",
        "ros2 launch robot_nav mapping.launch.py use_sim_time:=false")

    return {"status": "ok"}


@router.post("/stop")
def slam_stop():
    for name in ["slam_toolbox", "lio_slam", "lidar", "robot_bridge"]:
        process_mgr.stop_process(name)
    return {"status": "ok"}


@router.post("/save")
def slam_save(req: SaveRequest):
    if not req.name or "/" in req.name:
        raise HTTPException(400, "Invalid map name")
    dest_dir = os.path.join(MAPS_DIR, req.name)
    os.makedirs(dest_dir, exist_ok=True)
    out_path = os.path.join(dest_dir, "map")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    parts.append(
        f"ros2 service call /slam_toolbox/save_map"
        f" slam_toolbox/srv/SaveMap"
        f" \"{{name: {{data: '{out_path}'}}}}\" "
    )
    result = subprocess.run(
        ["bash", "-c", " && ".join(parts)],
        capture_output=True, text=True, timeout=15, env=env
    )
    if result.returncode != 0:
        raise HTTPException(500, result.stderr[-500:])
    return {"status": "ok", "map": req.name}



@router.post("/stop")
def slam_stop():
    for name in ["slam_toolbox", "lio_slam", "lidar", "robot_bridge"]:
        process_mgr.stop_process(name)
    return {"status": "ok"}


@router.post("/save")
def slam_save(req: SaveRequest):
    if not req.name or "/" in req.name:
        raise HTTPException(400, "Invalid map name")
    dest_dir = os.path.join(MAPS_DIR, req.name)
    os.makedirs(dest_dir, exist_ok=True)
    out_path = os.path.join(dest_dir, "map")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    parts.append(
        f"ros2 service call /slam_toolbox/save_map"
        f" slam_toolbox/srv/SaveMap"
        f" \"{{name: {{data: '{out_path}'}}}}\" "
    )
    result = subprocess.run(
        ["bash", "-c", " && ".join(parts)],
        capture_output=True, text=True, timeout=15, env=env
    )
    if result.returncode != 0:
        raise HTTPException(500, result.stderr[-500:])
    return {"status": "ok", "map": req.name}
