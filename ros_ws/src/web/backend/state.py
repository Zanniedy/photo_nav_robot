"""
持久化的运行时状态（web 后端和 points_server 共用，fcntl 锁）：
{"default_map": id | null,            # 开机自动导航的地图
 "current_map": id | null,            # 正在导航的地图，后端维护，points_server 读
 "poses": {id: {"x":..,"y":..,"yaw":..}},
 "estop": bool,                      # 软件急停（后端重启后保持，要手动解除）}
"""
import contextlib, fcntl, json, os
from config import STATE_FILE


@contextlib.contextmanager
def _locked():
    os.makedirs(os.path.dirname(STATE_FILE) or ".", exist_ok=True)
    with open(STATE_FILE + ".lock", "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)


def _read() -> dict:
    try:
        with open(STATE_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    d.setdefault("default_map", None)
    d.setdefault("current_map", None)
    d.setdefault("poses", {})
    d.setdefault("estop", False)
    return d


def load() -> dict:
    with _locked():
        return _read()


def update(fn) -> dict:
    """读-改-写，原子替换文件"""
    with _locked():
        d = _read()
        fn(d)
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=2, ensure_ascii=False)
        os.replace(tmp, STATE_FILE)
        return d
