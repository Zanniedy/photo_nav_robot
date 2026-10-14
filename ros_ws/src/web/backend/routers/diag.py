"""系统自检：串口设备 + 权限"""
import os
from fastapi import APIRouter
from config import SERIAL_DEVICES

router = APIRouter(prefix="/api/diag")

_LABELS = {"bridge": "下位机 (CH340)", "lidar": "激光雷达 (CP2102)"}


@router.get("")
def diag():
    items = []
    for key, path in SERIAL_DEVICES.items():
        label = _LABELS.get(key, key)
        if not os.path.exists(path):
            items.append({"label": label, "ok": False, "reason": "未检测到设备，检查 USB"})
        elif not os.access(path, os.R_OK | os.W_OK):
            real = os.path.realpath(path)
            items.append({"label": label, "ok": False,
                          "reason": f"{real} 无读写权限，把用户加入 dialout 组"})
        else:
            items.append({"label": label, "ok": True, "reason": os.path.realpath(path)})
    return {"ok": all(i["ok"] for i in items), "items": items}
