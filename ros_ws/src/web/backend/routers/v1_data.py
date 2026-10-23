"""
/api/v1：地图、点位、巡航路线。map_id 是地图名，不传 = 当前导航地图；点位 / 路线的 key 是 id 或名字。
"""
from fastapi import APIRouter
from pydantic import BaseModel, Field
import annot_store, maps, runtime, state
from api_errors import ApiError, ok
from routers import stack

router = APIRouter(prefix="/api/v1")


def map_path(map_id: str | None) -> str:
    """对外 map_id → yaml 路径（内部仍用路径找文件）；空 = 当前导航地图"""
    if map_id:
        try:
            return maps.by_map_id(map_id)["id"]
        except KeyError:
            raise ApiError("NOT_FOUND", f"地图不存在: {map_id}")
        except ValueError as e:
            raise ApiError("AMBIGUOUS", str(e))
    cur = stack.current_map()
    if cur is None:
        raise ApiError("NAV_NOT_RUNNING", "没有正在导航的地图，请指定 map_id")
    return cur


def find_point(path: str, key: str) -> dict:
    """id 或名字精确匹配；否则名字不区分大小写，唯一才算，不猜"""
    pts = annot_store.load(path)["points"]
    for p in pts:
        if p["id"] == key or p["name"] == key:
            return p
    hits = [p for p in pts if p["name"].lower() == key.lower()]
    if len(hits) == 1:
        return hits[0]
    if hits:
        raise ApiError("AMBIGUOUS", f"「{key}」匹配到多个点: " + "、".join(p["name"] for p in hits))
    raise ApiError("NOT_FOUND", f"点不存在: {key}")


def _store(fn, *args, **kw):
    try:
        return fn(*args, **kw)
    except annot_store.StoreError as e:
        raise ApiError("BAD_REQUEST", str(e))
    except KeyError as e:
        raise ApiError("NOT_FOUND", f"不存在: {e.args[0]}")


def _route_out(path: str, r: dict) -> dict:
    names = {p["id"]: p["name"] for p in annot_store.load(path)["points"]}
    return {**r, "point_names": [names.get(i, i) for i in r["points"]]}


_F = dict(allow_inf_nan=False)          # 坐标必须是有限数


class PointIn(BaseModel):
    name: str
    x: float | None = Field(None, **_F)
    y: float | None = Field(None, **_F)
    yaw: float = Field(0.0, **_F)
    note: str = ""
    from_current_pose: bool = False      # true：用机器人当前位姿（只能是当前导航地图）
    map_id: str | None = None


class PointPatch(BaseModel):
    name: str | None = None
    x: float | None = Field(None, **_F)
    y: float | None = Field(None, **_F)
    yaw: float | None = Field(None, **_F)
    note: str | None = None
    map_id: str | None = None


class RouteIn(BaseModel):
    name: str
    points: list[str] = Field(max_length=annot_store.MAX_ROUTE_POINTS)
    loop: bool = True
    map_id: str | None = None


class RoutePatch(BaseModel):
    name: str | None = None
    points: list[str] | None = Field(None, max_length=annot_store.MAX_ROUTE_POINTS)
    loop: bool | None = None
    map_id: str | None = None


# ── 地图 ────────────────────────────────────────────────────────────────

@router.get("/maps")
def list_maps():
    ms = maps.list_maps()
    cur, default = stack.current_map(), state.load()["default_map"]
    count: dict[str, int] = {}
    for m in ms:
        count[m["name"]] = count.get(m["name"], 0) + 1
    return ok([{"map_id": m["name"], "version": maps.version(m["id"]), "current": m["id"] == cur,
                "default": m["id"] == default, "duplicate": count[m["name"]] > 1} for m in ms])


# ── 点位 ────────────────────────────────────────────────────────────────

@router.get("/points")
def list_points(q: str = "", map_id: str | None = None):
    path = map_path(map_id)
    return ok({"map_id": maps.name_of(path), "points": annot_store.list_points(path, q)})


@router.post("/points")
def create_point(req: PointIn):
    path = map_path(req.map_id)
    x, y, yaw = req.x, req.y, req.yaw
    if req.from_current_pose:
        if path != stack.current_map():
            raise ApiError("MAP_MISMATCH", "只能把当前位置存到当前导航地图")
        pose = runtime.nav.pose() if runtime.nav else None
        if pose is None:
            raise ApiError("BAD_REQUEST", "拿不到机器人当前位姿（定位没起来？）")
        x, y, yaw = pose
    elif x is None or y is None:
        raise ApiError("BAD_REQUEST", "要给 x、y，或者 from_current_pose: true")
    return ok(_store(annot_store.create_point, path, req.name, x, y, yaw, req.note))


@router.patch("/points/{key}")
def update_point(key: str, req: PointPatch):
    path = map_path(req.map_id)
    p = find_point(path, key)
    return ok(_store(annot_store.update_point, path, p["id"], **req.model_dump(exclude={"map_id"})))


@router.delete("/points/{key}")
def delete_point(key: str, map_id: str | None = None):
    path = map_path(map_id)
    annot_store.delete_point(path, find_point(path, key)["id"])
    return ok()


# ── 巡航路线 ─────────────────────────────────────────────────────────────

@router.get("/routes")
def list_routes(map_id: str | None = None):
    path = map_path(map_id)
    return ok({"map_id": maps.name_of(path), "routes": [_route_out(path, r) for r in annot_store.list_routes(path)]})


@router.post("/routes")
def create_route(req: RouteIn):
    path = map_path(req.map_id)
    return ok(_route_out(path, _store(annot_store.create_route, path, req.name, req.points, req.loop)))


@router.put("/routes/{key}")
def update_route(key: str, req: RoutePatch):
    path = map_path(req.map_id)
    r = _store(annot_store.update_route, path, key, name=req.name, points=req.points, loop=req.loop)
    return ok(_route_out(path, r))


@router.delete("/routes/{key}")
def delete_route(key: str, map_id: str | None = None):
    if not annot_store.delete_route(map_path(map_id), key):
        raise ApiError("NOT_FOUND", f"路线不存在: {key}")
    return ok()
