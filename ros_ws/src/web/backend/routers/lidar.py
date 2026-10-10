from fastapi import APIRouter
import process_mgr
from config import LIDAR_SERIAL_PORT, LIDAR_BAUDRATE

router = APIRouter(prefix="/api/lidar")


@router.post("/start")
def lidar_start():
    cmd = (
        f"ros2 launch sllidar_ros2 sllidar_c1_launch.py"
        f" serial_port:={LIDAR_SERIAL_PORT}"
        f" serial_baudrate:={LIDAR_BAUDRATE}"
    )
    process_mgr.start_process("lidar", cmd)
    return {"status": "ok"}


@router.post("/stop")
def lidar_stop():
    process_mgr.stop_process("lidar")
    return {"status": "ok"}
