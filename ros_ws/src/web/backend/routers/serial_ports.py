from fastapi import APIRouter
import glob
import os

router = APIRouter(prefix="/api/serial")

# 已知设备的识别规则（按 USB vendor/product 或 by-id 路径匹配）
_DEVICE_HINTS = {
    "下位机 (CH340)":  ["ttyUSB*", "ttyACM*"],
    "激光雷达":        ["ttyUSB*"],
    "USB 相机":        ["video*"],
}

def _list_ports():
    ports = []
    for p in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        info = {"port": p, "label": _guess_label(p)}
        ports.append(info)
    return ports

def _guess_label(port: str) -> str:
    # 尝试从 /dev/serial/by-id 找到符号链接名称来猜设备
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        for name in os.listdir(by_id):
            try:
                target = os.path.realpath(os.path.join(by_id, name))
                if target == os.path.realpath(port):
                    n = name.lower()
                    if "ch340" in n or "ch341" in n:
                        return "下位机 (CH340)"
                    if "cp210" in n:
                        return "激光雷达 (CP2102)"
                    if "ftdi" in n:
                        return "FTDI 设备"
                    return name[:40]
            except OSError:
                pass
    return ""

@router.get("/ports")
def list_serial_ports():
    return {"ports": _list_ports()}
