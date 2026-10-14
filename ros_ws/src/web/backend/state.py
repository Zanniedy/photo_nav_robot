"""
持久化的运行时状态：默认地图、每张地图保存的初始位姿。
{"default_map": "/abs/path.yaml" | null, "poses": {"/abs/path.yaml": {"x":..,"y":..,"yaw":..}}}
"""
import json, os, threading
from config import STATE_FILE

_lock = threading.Lock()


def load() -> dict:
    try:
        with open(STATE_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    d.setdefault("default_map", None)
    d.setdefault("poses", {})
    return d


def update(fn) -> dict:
    """读-改-写，原子替换文件"""
    with _lock:
        d = load()
        fn(d)
        os.makedirs(os.path.dirname(STATE_FILE) or ".", exist_ok=True)
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=2, ensure_ascii=False)
        os.replace(tmp, STATE_FILE)
        return d
