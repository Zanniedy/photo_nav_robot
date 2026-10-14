"""
地图文件管理：在 map_dirs 下递归找 map_server 格式的 yaml（含 image: 字段）。
前端用 yaml 绝对路径作为地图 id，后端只接受出现在搜索结果里的路径（防目录穿越）。
"""
import os, re
import yaml
from config import MAP_DIRS

_NAME_RE = re.compile(r"^[A-Za-z0-9_\-.]{1,64}$")
_MAX_DEPTH = 3


def valid_name(name: str) -> bool:
    return bool(_NAME_RE.match(name)) and not name.startswith(".")


def _is_map_yaml(path: str) -> bool:
    try:
        if os.path.getsize(path) > 4096:
            return False
        with open(path) as f:
            d = yaml.safe_load(f)
        return isinstance(d, dict) and "image" in d and "resolution" in d
    except Exception:
        return False


def _walk(root: str, depth: int, out: list) -> None:
    try:
        entries = list(os.scandir(root))
    except OSError:
        return
    for e in entries:
        if e.name.startswith("."):
            continue
        if e.is_dir(follow_symlinks=False) and depth < _MAX_DEPTH:
            _walk(e.path, depth + 1, out)
        elif e.is_file() and e.name.endswith((".yaml", ".yml")) and _is_map_yaml(e.path):
            out.append(e.path)


def list_maps() -> list[dict]:
    os.makedirs(MAP_DIRS[0], exist_ok=True)
    seen, out = set(), []
    for d in MAP_DIRS:
        paths: list[str] = []
        _walk(d, 0, paths)
        for p in paths:
            real = os.path.realpath(p)
            if real in seen:
                continue
            seen.add(real)
            base = os.path.basename(p).rsplit(".", 1)[0]
            # 旧目录格式 <name>/map.yaml 用目录名
            name = os.path.basename(os.path.dirname(p)) if base == "map" else base
            out.append({"id": os.path.abspath(p), "name": name, "dir": d,
                        "mtime": os.stat(p).st_mtime})
    out.sort(key=lambda m: m["mtime"], reverse=True)
    return out


def resolve(map_id: str) -> dict | None:
    """map_id（yaml 路径）或地图名 → 地图条目；不在搜索结果里返回 None"""
    ms = list_maps()
    for m in ms:
        if m["id"] == map_id:
            return m
    for m in ms:
        if m["name"] == map_id:
            return m
    return None


def save_prefix(name: str) -> str:
    """slam_toolbox save_map 的输出前缀（不带扩展名），存到第一个搜索目录"""
    os.makedirs(MAP_DIRS[0], exist_ok=True)
    return os.path.join(MAP_DIRS[0], name)
