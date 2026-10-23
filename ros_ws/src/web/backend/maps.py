"""
地图文件管理：在 map_dirs 下递归找 map_server 格式的 yaml（含 image: 字段）。
前端用 yaml 绝对路径作为地图 id，后端只接受出现在搜索结果里的路径（防目录穿越）。
对外（/api/v1）用 map_id = 地图名、version = yaml + 图片内容哈希，换机器人、换目录都不变。
"""
import hashlib, os, re
import yaml
from config import MAP_DIRS

_NAME_RE = re.compile(r"^[A-Za-z0-9_\-.]{1,64}$")
_MAX_DEPTH = 3
_version_cache: dict = {}     # (yaml 路径, yaml 的 mtime/size, 图片路径, 图片的 mtime/size) → version


def name_of(path: str) -> str:
    """yaml 路径 → 地图名（对外 map_id）；旧目录格式 <name>/map.yaml 用目录名"""
    base = os.path.basename(path).rsplit(".", 1)[0]
    return os.path.basename(os.path.dirname(path)) if base == "map" else base


def _stat(p: str | None):
    try:
        s = os.stat(p)
        return (s.st_mtime_ns, s.st_size)
    except (OSError, TypeError):
        return None


def _image_path(yaml_path: str) -> str | None:
    try:
        with open(yaml_path) as f:
            img = (yaml.safe_load(f) or {}).get("image")
    except Exception:
        return None
    if not img:
        return None
    return img if os.path.isabs(img) else os.path.join(os.path.dirname(yaml_path), img)


def version(yaml_path: str) -> str:
    """yaml + 图片内容 sha1 前 12 位。按文件 mtime/size 缓存，status 每秒都要算"""
    img = _image_path(yaml_path)
    key = (yaml_path, _stat(yaml_path), img, _stat(img))
    v = _version_cache.get(key)
    if v is None:
        h = hashlib.sha1()
        for p in (yaml_path, img):
            if not p:
                continue
            try:
                with open(p, "rb") as f:
                    for chunk in iter(lambda: f.read(1 << 20), b""):
                        h.update(chunk)
            except OSError:
                pass
        v = h.hexdigest()[:12]
        if len(_version_cache) > 200:
            _version_cache.clear()
        _version_cache[key] = v
    return v


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
            name = name_of(p)
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


def by_map_id(map_id: str) -> dict:
    """对外 map_id（地图名）→ 地图条目。找不到 KeyError；多个目录里有同名地图 ValueError（让用户改名）"""
    hits = [m for m in list_maps() if m["name"] == map_id]
    if not hits:
        raise KeyError(map_id)
    if len(hits) > 1:
        raise ValueError(f"有 {len(hits)} 张同名地图 {map_id}：" + "、".join(m["id"] for m in hits))
    return hits[0]
