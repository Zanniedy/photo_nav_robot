from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import subprocess, os
import process_mgr
from config import ROS2_SETUP, WORKSPACE_SETUP, ROS_DOMAIN_ID, MAPS_DIR

router = APIRouter(prefix="/api/slam")


class SaveRequest(BaseModel):
    name: str


@router.post("/start")
def slam_start():
    process_mgr.stop_process("map_server")
    cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
    process_mgr.start_process("slam_toolbox", cmd)
    return {"status": "ok"}


@router.post("/stop")
def slam_stop():
    process_mgr.stop_process("slam_toolbox")
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
