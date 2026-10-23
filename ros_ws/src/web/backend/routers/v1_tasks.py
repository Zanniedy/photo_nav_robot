"""
/api/v1：任务。goto / guide / patrol，点名在当前导航地图里查（带了 map_id 就先核对是不是当前地图）。
给上层（语音、平板、云）用的最常见调用是 POST /api/v1/guide {"point": "A会议室"}。
"""
from typing import Literal
from fastapi import APIRouter, Request
from pydantic import BaseModel, Field
import annot_store, maps, process_mgr, runtime
from api_errors import ApiError, ok
from routers.v1_data import find_point, map_path
from task_runner import TaskError

router = APIRouter(prefix="/api/v1")


class TaskReq(BaseModel):
    type: Literal["goto", "guide", "patrol"]
    points: list[str] = Field(default_factory=list, max_length=100)   # goto：依次去的点
    point: str | None = None                                           # guide：一个点（goto 也可用）
    route: str | None = None                                           # patrol：路线 id 或名字
    loop: bool | None = None                                           # patrol：不给就按路线设置
    max_speed: float | None = Field(None, gt=0, le=0.5)
    map_id: str | None = None


class GuideReq(BaseModel):
    point: str
    max_speed: float | None = Field(None, gt=0, le=0.5)
    map_id: str | None = None


def _runner():
    if runtime.runner is None:
        raise ApiError("INTERNAL", "任务层没启动")
    return runtime.runner


def _call(fn, *args, **kw):
    try:
        return fn(*args, **kw)
    except TaskError as e:
        raise ApiError(e.code, e.msg)


def _snapshot(p: dict) -> dict:
    return {"id": p["id"], "name": p["name"], "x": p["x"], "y": p["y"], "yaw": p.get("yaw", 0.0)}


def _submit(req: TaskReq, request: Request) -> dict:
    r = _runner()
    _call(r.check, req.map_id)                     # 先核对状态和地图，再查点，错误码更准
    path = map_path(None)
    loop, route_id = False, None
    if req.type == "patrol":
        if not req.route:
            raise ApiError("BAD_REQUEST", "patrol 要指定 route")
        route = annot_store.get_route(path, req.route)
        if route is None:
            raise ApiError("NOT_FOUND", f"路线不存在: {req.route}")
        by_id = {p["id"]: p for p in annot_store.load(path)["points"]}
        pts = [by_id[i] for i in route["points"] if i in by_id]
        if not pts:
            raise ApiError("BAD_REQUEST", f"路线 {route['name']} 里没有点")
        loop = route["loop"] if req.loop is None else req.loop
        route_id = route["id"]
    else:
        keys = req.points or ([req.point] if req.point else [])
        pts = [find_point(path, k) for k in keys]
    # 没带 map_id 也用查点的那张地图核对，防止查点和下发之间导航换了地图
    t = _call(r.submit, req.type, [_snapshot(p) for p in pts], map_id=req.map_id or maps.name_of(path),
              max_speed=req.max_speed, loop=loop, route_id=route_id)
    process_mgr.log("ui", f"[{getattr(request.state, 'caller', '?')}] {req.type} → "
                          + " → ".join(t["points"]) + f"（任务 {t['id']}）")
    return t


@router.post("/tasks")
def create_task(req: TaskReq, request: Request):
    return ok(_submit(req, request))


@router.post("/guide")
def guide(req: GuideReq, request: Request):
    return ok(_submit(TaskReq(type="guide", point=req.point, max_speed=req.max_speed, map_id=req.map_id), request))


@router.get("/tasks/current")
def current_task():
    return ok(_runner().current())


@router.get("/tasks/{task_id}")
def get_task(task_id: str):
    t = _runner().get(task_id)
    if t is None:
        raise ApiError("NOT_FOUND", f"任务不存在: {task_id}")
    return ok(t)


@router.post("/tasks/{task_id}/cancel")
def cancel_task(task_id: str):
    return ok(_call(_runner().cancel, task_id))


@router.post("/tasks/{task_id}/wait")
def wait_task(task_id: str):
    return ok(_call(_runner().wait, task_id))


@router.post("/tasks/{task_id}/resume")
def resume_task(task_id: str):
    return ok(_call(_runner().resume, task_id))
