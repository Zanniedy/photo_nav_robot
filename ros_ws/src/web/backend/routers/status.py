from fastapi import APIRouter
from fastapi.responses import StreamingResponse
import asyncio, json
import process_mgr
from config import ROBOT_IP

router = APIRouter(prefix="/api")


def _current_mode() -> str:
    if process_mgr.is_running("slam_toolbox"):
        return "mapping"
    if process_mgr.is_running("nav2"):
        return "navigating"
    return "idle"


@router.get("/status")
def get_status():
    return {
        "mode": _current_mode(),
        "processes": {
            name: process_mgr.is_running(name)
            for name in ["slam_toolbox", "nav2", "map_server", "lidar"]
        },
    }


@router.get("/config")
def get_config():
    return {"robot_ip": ROBOT_IP}


@router.get("/process/logs")
async def stream_logs():
    async def generator():
        last_count = 0
        while True:
            logs = process_mgr.get_logs()
            if len(logs) > last_count:
                for line in logs[last_count:]:
                    data = json.dumps({"line": line})
                    yield f"data: {data}\n\n"
                last_count = len(logs)
            await asyncio.sleep(0.5)
    return StreamingResponse(generator(), media_type="text/event-stream")
