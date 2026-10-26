"""系统自检：串口 / 相机 / 声卡设备 + 权限，人脸检测（person）、LED 屏（led）"""
import glob, os
import yaml
from fastapi import APIRouter
import process_mgr, runtime
from config import SERIAL_DEVICES, WEB_DIR

router = APIRouter(prefix="/api/diag")

_LABELS = {"bridge": "下位机 (CH340)", "lidar": "激光雷达 (CP2102)", "camera": "相机 (AX USB2.0)",
           "speaker": "USB 声卡 (HECATE G1200)"}
_GROUPS = {"camera": "video", "speaker": "audio"}           # 其余是串口 dialout
_SRC = os.path.dirname(WEB_DIR)                              # ros_ws/src
PERSON_YAML = os.path.join(_SRC, "person_locator", "config", "person_tracker.yaml")
LED_YAML = os.path.join(_SRC, "led_display", "config", "led.yaml")


def _item(label: str, ok: bool, reason: str) -> dict:
    return {"label": label, "ok": ok, "reason": reason}


def _params(path: str, node: str) -> dict:
    try:
        with open(path) as f:
            return (yaml.safe_load(f) or {}).get(node, {}).get("ros__parameters", {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _device(key: str, path: str) -> dict:
    label = _LABELS.get(key, key)
    if not os.path.exists(path):
        return _item(label, False, "未检测到设备，检查 USB")
    if not os.access(path, os.R_OK | os.W_OK):
        return _item(label, False, f"{os.path.realpath(path)} 无读写权限，把用户加入 {_GROUPS.get(key, 'dialout')} 组")
    return _item(label, True, os.path.realpath(path))


def _person() -> dict:
    label = "人脸检测 (person)"
    p = _params(PERSON_YAML, "person_tracker")
    model = os.path.expanduser(p.get("model_path", "~/max_robot/models/yolo26s_seg_bpu.bin"))
    face = os.path.expanduser(p.get("face_model", "~/max_robot/models/face_detection_yunet_2023mar.onnx"))
    missing = [f for f in (model, os.path.splitext(model)[0] + "_outputs.json", face) if not os.path.isfile(f)]
    if missing:
        return _item(label, False, "缺模型文件: " + "、".join(missing))
    if not process_mgr.is_running("person"):
        return _item(label, False, "进程没在跑")
    st = runtime.nav.person_info() if runtime.nav is not None else None
    if st is None:
        return _item(label, False, "进程在跑，1s 内没收到 /person/state（刚启动要等模型加载）")
    if not st.get("face_model"):
        return _item(label, False, "人脸模型没加载，看 person 日志")
    det = st.get("det") or {}
    if (det.get("hz") or 0) < 0.5:
        return _item(label, False, f"检测没在出结果（{det.get('hz', 0)}Hz），看 camera 进程")
    return _item(label, True, f"检测 {det['hz']}Hz，{det.get('ms', 0)}ms/帧，人脸 {det.get('face_ms', 0)}ms")


def _bt_blocked() -> bool:
    for r in glob.glob("/sys/class/rfkill/rfkill*"):
        try:
            if open(f"{r}/type").read().strip() == "bluetooth" and \
                    "1" in (open(f"{r}/soft").read().strip(), open(f"{r}/hard").read().strip()):
                return True
        except OSError:
            pass
    return False


def _led() -> dict:
    label = "LED 屏 (led)"
    if not glob.glob("/sys/class/bluetooth/hci*"):
        return _item(label, False, "没有蓝牙适配器")
    if _bt_blocked():
        return _item(label, False, "蓝牙被 rfkill 屏蔽：sudo rfkill unblock bluetooth && bluetoothctl power on")
    if not process_mgr.is_running("led"):
        return _item(label, False, "进程没在跑")
    st = runtime.nav.led_state() if runtime.nav is not None else None
    addr = _params(LED_YAML, "led_display").get("address", "")
    if not st or not st.get("connected"):
        err = (st or {}).get("error") or "还没连上（屏同一时间只能连一台，手机 App / 别的电脑连着就连不上）"
        return _item(label, False, f"{addr} {err}".strip())
    return _item(label, True, f"已连接 {addr}，显示 {st.get('showing') or '-'}")


@router.get("")
def diag():
    items = [_device(k, p) for k, p in SERIAL_DEVICES.items()] + [_person(), _led()]
    return {"ok": all(i["ok"] for i in items), "items": items}
