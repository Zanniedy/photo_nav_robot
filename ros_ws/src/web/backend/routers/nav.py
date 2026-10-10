from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import os
import process_mgr, map_mgr

router = APIRouter(prefix="/api/nav")


class StartRequest(BaseModel):
    map_name: str


@router.post("/start")
def nav_start(req: StartRequest):
    yaml_path = map_mgr.map_yaml_path(req.map_name)
    if not os.path.exists(yaml_path):
        raise HTTPException(404, f"Map not found: {req.map_name}")
    process_mgr.stop_process("nav2")
    process_mgr.stop_process("slam_toolbox")
    map_cmd = (
        f"ros2 run nav2_map_server map_server --ros-args"
        f" -p yaml_filename:={yaml_path}"
        f" -p use_sim_time:=false"
    )
    process_mgr.start_process("map_server", map_cmd)
    nav_cmd = "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"
    process_mgr.start_process("nav2", nav_cmd)
    return {"status": "ok", "map": req.map_name}


@router.post("/stop")
def nav_stop():
    process_mgr.stop_process("nav2")
    process_mgr.stop_process("map_server")
    return {"status": "ok"}
