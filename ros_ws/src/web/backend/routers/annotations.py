"""
地图标注 REST：绘图形状（整体保存）+ 命名点位（增删改查、搜索、导航过去）
所有接口的 map 参数都是地图 id（yaml 路径）或地图名；点位接口的 key 是点 id 或点名。
外部程序示例：
  curl -b cookie -X POST :8000/api/points_goto -d '{"names":["门口"]}' -H 'Content-Type: application/json'
"""
import math
from typing import Literal
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, field_validator
import annot_store, maps
from routers import stack

router = APIRouter(prefix="/api")


def _mid(map_ref: str | None) -> str:
    """map 参数 → 地图 id；空 = 当前导航地图"""
    if not map_ref:
        cur = stack.current_map()
        if cur is None:
            raise HTTPException(409, "没有正在导航的地图，请指定 map")
        return cur
    m = maps.resolve(map_ref)
    if m is None:
        raise HTTPException(404, f"地图不存在: {map_ref}")
    return m["id"]


# ── 形状 ────────────────────────────────────────────────────────────────

class Shape(BaseModel):
    id: str = Field(max_length=32)
    type: Literal["line", "polyline", "polygon", "rect", "circle", "arrow", "text", "freehand"]
    pts: list[tuple[float, float]] = Field(max_length=5000)   # map 系坐标；circle: [圆心, 圆上一点]
    color: str = Field("#ffcc00", pattern=r"^#[0-9a-fA-F]{3,8}$")
    width: float = Field(2, ge=0.5, le=20)                     # 线宽 px
    fill: bool = False
    text: str = Field("", max_length=200)
    hidden: bool = False

    @field_validator("pts")
    @classmethod
    def finite(cls, v):
        if not all(math.isfinite(a) and math.isfinite(b) for a, b in v):
            raise ValueError("坐标必须是有限数")
        return v


class ShapesReq(BaseModel):
    map: str
    shapes: list[Shape]


@router.get("/annotations")
def get_annotations(map: str = ""):
    mid = _mid(map)
    d = annot_store.load(mid)
    return {"map": mid, "shapes": d["shapes"], "points": d["points"]}


@router.put("/annotations/shapes")
def put_shapes(req: ShapesReq):
    mid = _mid(req.map)
    try:
        annot_store.save_shapes(mid, [s.model_dump() for s in req.shapes])
    except annot_store.StoreError as e:
        raise HTTPException(400, str(e))
    return {"ok": True, "count": len(req.shapes)}


@router.post("/annotations/shapes_beacon")
def beacon_shapes(req: ShapesReq):
    """页面关闭时 navigator.sendBeacon 只能 POST"""
    return put_shapes(req)


# ── 点位 ────────────────────────────────────────────────────────────────

class PointIn(BaseModel):
    map: str = ""
    name: str
    x: float
    y: float
    yaw: float = 0.0
    note: str = ""


class PointPatch(BaseModel):
    map: str = ""
    name: str | None = None
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    note: str | None = None


class ReorderReq(BaseModel):
    map: str = ""
    ids: list[str]


class GotoReq(BaseModel):
    map: str = ""
    names: list[str] = Field(min_length=1, max_length=100)   # 点名或 id，多个按顺序走


@router.get("/points")
def list_points(map: str = "", q: str = ""):
    mid = _mid(map)
    return {"map": mid, "points": annot_store.list_points(mid, q)}


@router.get("/points/{key}")
def get_point(key: str, map: str = ""):
    p = annot_store.get_point(_mid(map), key)
    if p is None:
        raise HTTPException(404, f"点不存在: {key}")
    return p


@router.post("/points")
def create_point(req: PointIn):
    try:
        return annot_store.create_point(_mid(req.map), req.name, req.x, req.y, req.yaw, req.note)
    except annot_store.StoreError as e:
        raise HTTPException(400, str(e))


@router.patch("/points/{key}")
def update_point(key: str, req: PointPatch):
    try:
        return annot_store.update_point(_mid(req.map), key, **req.model_dump(exclude={"map"}))
    except KeyError:
        raise HTTPException(404, f"点不存在: {key}")
    except annot_store.StoreError as e:
        raise HTTPException(400, str(e))


@router.delete("/points/{key}")
def delete_point(key: str, map: str = ""):
    if not annot_store.delete_point(_mid(map), key):
        raise HTTPException(404, f"点不存在: {key}")
    return {"ok": True}


@router.post("/points_reorder")
def reorder_points(req: ReorderReq):
    annot_store.reorder_points(_mid(req.map), req.ids)
    return {"ok": True}


@router.post("/points_goto")
def goto_points(req: GotoReq):
    """导航到一个或依次多个点（只能用当前导航地图的点）"""
    cur = stack.current_map()
    if cur is None:
        raise HTTPException(409, "导航未启动")
    if req.map and _mid(req.map) != cur:
        raise HTTPException(400, "指定的地图不是当前导航地图")
    pts = []
    for n in req.names:
        p = annot_store.get_point(cur, n)
        if p is None:
            raise HTTPException(404, f"点不存在: {n}")
        pts.append(p)
    t = stack.submit_legacy([{k: p[k] for k in ("id", "name", "x", "y", "yaw")} for p in pts])
    return {"ok": True, "names": [p["name"] for p in pts], "task_id": t["id"]}
