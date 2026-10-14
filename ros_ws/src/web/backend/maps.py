"""
地图文件管理。兼容两种布局：
  maps_dir/<name>.yaml            （save_map.sh / 本 UI 保存的平铺格式）
  maps_dir/<name>/map.yaml        （旧 UI 的目录格式）
"""
import os, re
from config import MAPS_DIR

_NAME_RE = re.compile(r"^[A-Za-z0-9_\-.]{1,64}$")


def valid_name(name: str) -> bool:
    return bool(_NAME_RE.match(name)) and not name.startswith(".")


def list_maps() -> list[dict]:
    os.makedirs(MAPS_DIR, exist_ok=True)
    out = []
    for entry in os.scandir(MAPS_DIR):
        if entry.is_file() and entry.name.endswith(".yaml"):
            out.append({"name": entry.name[:-5], "yaml": entry.path, "mtime": entry.stat().st_mtime})
        elif entry.is_dir():
            y = os.path.join(entry.path, "map.yaml")
            if os.path.exists(y):
                out.append({"name": entry.name, "yaml": y, "mtime": os.stat(y).st_mtime})
    out.sort(key=lambda m: m["mtime"], reverse=True)
    return out


def yaml_path(name: str) -> str | None:
    if not valid_name(name):
        return None
    for m in list_maps():
        if m["name"] == name:
            return m["yaml"]
    return None


def save_prefix(name: str) -> str:
    """slam_toolbox save_map 的输出前缀（不带扩展名）"""
    os.makedirs(MAPS_DIR, exist_ok=True)
    return os.path.join(MAPS_DIR, name)
