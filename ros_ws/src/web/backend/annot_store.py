"""
每张地图的标注（绘图形状 + 命名点位），按地图 id（yaml 绝对路径）存一个 JSON：
  annotations_dir/<地图名>_<sha1(id)[:8]>.json
  {"map": id, "shapes": [...], "points": [{id, name, x, y, yaw, note}, ...]}

web 后端和 ROS points_server 节点都读写这里，用 fcntl 文件锁串行化。纯 Python，无 ROS 依赖。
"""
import contextlib, fcntl, hashlib, json, math, os, re, secrets
from config import ANNOT_DIR

MAX_POINTS = 500
MAX_SHAPES = 2000
_NAME_RE = re.compile(r"^[^\x00-\x1f/\\]{1,64}$")


class StoreError(ValueError):
    pass


def _path(map_id: str) -> str:
    base = os.path.basename(map_id).rsplit(".", 1)[0]
    if base == "map":
        base = os.path.basename(os.path.dirname(map_id))
    base = re.sub(r"[^A-Za-z0-9_\-.]", "_", base)[:48] or "map"
    h = hashlib.sha1(map_id.encode()).hexdigest()[:8]
    return os.path.join(ANNOT_DIR, f"{base}_{h}.json")


@contextlib.contextmanager
def _locked():
    os.makedirs(ANNOT_DIR, exist_ok=True)
    with open(os.path.join(ANNOT_DIR, ".lock"), "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)


def _read(map_id: str) -> dict:
    try:
        with open(_path(map_id)) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    return {"map": map_id, "shapes": list(d.get("shapes", [])), "points": list(d.get("points", []))}


def _write(d: dict) -> None:
    p = _path(d["map"])
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=1, ensure_ascii=False)
    os.replace(tmp, p)


def load(map_id: str) -> dict:
    with _locked():
        return _read(map_id)


# ── 形状：前端整体覆盖保存 ──────────────────────────────────────────────

def save_shapes(map_id: str, shapes: list[dict]) -> None:
    if len(shapes) > MAX_SHAPES:
        raise StoreError(f"形状最多 {MAX_SHAPES} 个")
    with _locked():
        d = _read(map_id)
        d["shapes"] = shapes
        _write(d)


# ── 点位：增删改查 ───────────────────────────────────────────────────────

def _check_point(name: str, x: float, y: float, yaw: float) -> None:
    if not _NAME_RE.match(name or "") or name.strip() != name:
        raise StoreError("点名 1~64 字符，不能含 / \\ 或首尾空格")
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise StoreError("坐标必须是有限数")


def list_points(map_id: str, query: str = "") -> list[dict]:
    pts = load(map_id)["points"]
    q = query.strip().lower()
    if q:
        pts = [p for p in pts if q in p["name"].lower() or q in p.get("note", "").lower()]
    return pts


def get_point(map_id: str, key: str) -> dict | None:
    """key 可以是点 id 或点名（名字精确匹配优先，再不区分大小写）"""
    pts = load(map_id)["points"]
    for p in pts:
        if p["id"] == key or p["name"] == key:
            return p
    for p in pts:
        if p["name"].lower() == key.lower():
            return p
    return None


def create_point(map_id: str, name: str, x: float, y: float, yaw: float = 0.0, note: str = "") -> dict:
    _check_point(name, x, y, yaw)
    with _locked():
        d = _read(map_id)
        if any(p["name"] == name for p in d["points"]):
            raise StoreError(f"点名已存在: {name}")
        if len(d["points"]) >= MAX_POINTS:
            raise StoreError(f"点位最多 {MAX_POINTS} 个")
        p = {"id": secrets.token_hex(4), "name": name, "x": x, "y": y, "yaw": yaw, "note": note[:200]}
        d["points"].append(p)
        _write(d)
        return p


def update_point(map_id: str, key: str, **fields) -> dict:
    with _locked():
        d = _read(map_id)
        p = next((p for p in d["points"] if p["id"] == key or p["name"] == key), None)
        if p is None:
            raise KeyError(key)
        new = {**p, **{k: v for k, v in fields.items() if v is not None and k in ("name", "x", "y", "yaw", "note")}}
        _check_point(new["name"], new["x"], new["y"], new["yaw"])
        if new["name"] != p["name"] and any(q["name"] == new["name"] for q in d["points"]):
            raise StoreError(f"点名已存在: {new['name']}")
        new["note"] = str(new.get("note", ""))[:200]
        p.update(new)
        _write(d)
        return p


def upsert_point(map_id: str, name: str, x: float, y: float, yaw: float = 0.0, note: str = "") -> dict:
    """按名字：有就改坐标，没有就新建（ROS SetPoint 用）"""
    try:
        return update_point(map_id, name, x=x, y=y, yaw=yaw, note=note)
    except KeyError:
        return create_point(map_id, name, x, y, yaw, note)


def delete_point(map_id: str, key: str) -> bool:
    with _locked():
        d = _read(map_id)
        n = len(d["points"])
        d["points"] = [p for p in d["points"] if p["id"] != key and p["name"] != key]
        if len(d["points"]) == n:
            return False
        _write(d)
        return True


def reorder_points(map_id: str, ids: list[str]) -> None:
    with _locked():
        d = _read(map_id)
        order = {i: k for k, i in enumerate(ids)}
        d["points"].sort(key=lambda p: order.get(p["id"], len(order)))
        _write(d)
