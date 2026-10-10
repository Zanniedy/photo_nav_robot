from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import os
import map_mgr, process_mgr

router = APIRouter(prefix="/api/maps")


class SwitchRequest(BaseModel):
    name: str


@router.get("")
def get_maps():
    return map_mgr.list_maps()


@router.post("/switch")
def switch_map(req: SwitchRequest):
    yaml_path = map_mgr.map_yaml_path(req.name)
    if not os.path.exists(yaml_path):
        raise HTTPException(404, f"Map not found: {req.name}")
    process_mgr.stop_process("map_server")
    cmd = (
        f"ros2 run nav2_map_server map_server --ros-args"
        f" -p yaml_filename:={yaml_path}"
        f" -p use_sim_time:=false"
    )
    process_mgr.start_process("map_server", cmd)
    return {"status": "ok", "map": req.name}
