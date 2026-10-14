"""进程编排：底盘 / 建图 / 导航模式切换、地图、航点、日志流"""
import asyncio, json, math, shlex, time
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import maps, process_mgr
from config import PROCESSES

router = APIRouter(prefix="/api")

# 前端展示顺序；waypoints 是 FollowWaypoints action 客户端进程
PROC_NAMES = ["rosbridge", "base", "slam", "nav", "waypoints"]
_state = {"map": None}


class MapReq(BaseModel):
    name: str


class Pose2D(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class WaypointsReq(BaseModel):
    poses: list[Pose2D]


def _mode() -> str:
    if process_mgr.is_running("slam"):
        return "mapping"
    if process_mgr.is_running("nav"):
        return "navigation"
    if process_mgr.is_running("base"):
        return "base"
    return "idle"


def _ensure_base() -> None:
    if not process_mgr.is_running("base"):
        process_mgr.start("base", PROCESSES["base"])


def _map_or_404(name: str) -> str:
    path = maps.yaml_path(name)
    if path is None:
        raise HTTPException(404, f"地图不存在: {name}")
    return path


@router.get("/status")
def status():
    return {
        "mode": _mode(),
        "map": _state["map"] if process_mgr.is_running("nav") else None,
        "processes": {n: process_mgr.info(n) for n in PROC_NAMES},
        "time": time.time(),
    }


@router.post("/proc/{name}/start")
def proc_start(name: str):
    if name not in ("rosbridge", "base"):
        raise HTTPException(400, "只能单独启动 rosbridge / base，建图导航用对应接口")
    process_mgr.start(name, PROCESSES[name])
    return {"ok": True}


@router.post("/proc/{name}/stop")
def proc_stop(name: str):
    if name not in PROC_NAMES:
        raise HTTPException(404, name)
    process_mgr.stop(name)
    return {"ok": True}


@router.post("/mapping/start")
def mapping_start():
    process_mgr.stop("waypoints")
    process_mgr.stop("nav")
    _ensure_base()
    process_mgr.start("slam", PROCESSES["slam"])
    return {"ok": True}


@router.post("/mapping/save")
def mapping_save(req: MapReq):
    if not maps.valid_name(req.name):
        raise HTTPException(400, "地图名只能包含字母数字 _ - .")
    if not process_mgr.is_running("slam"):
        raise HTTPException(409, "当前不在建图模式")
    prefix = maps.save_prefix(req.name)
    r = process_mgr.run_once(
        ["ros2", "service", "call", "/slam_toolbox/save_map", "slam_toolbox/srv/SaveMap",
         json.dumps({"name": {"data": prefix}})], timeout=20)
    if "data=0" not in r["output"]:
        raise HTTPException(500, "save_map 失败:\n" + r["output"][-800:])
    # 顺带存 posegraph，以后想用 slam_toolbox localization 模式可以直接用
    process_mgr.run_once(
        ["ros2", "service", "call", "/slam_toolbox/serialize_map",
         "slam_toolbox/srv/SerializePoseGraph", json.dumps({"filename": prefix})], timeout=20)
    return {"ok": True, "yaml": prefix + ".yaml"}


@router.post("/nav/start")
def nav_start(req: MapReq):
    path = _map_or_404(req.name)
    process_mgr.stop("waypoints")
    process_mgr.stop("slam")
    _ensure_base()
    process_mgr.start("nav", PROCESSES["nav"].replace("{map}", shlex.quote(path)))
    _state["map"] = req.name
    return {"ok": True}


@router.post("/nav/switch_map")
def nav_switch_map(req: MapReq):
    """导航中热切换地图（map_server load_map），不在导航就直接以该地图启动导航"""
    path = _map_or_404(req.name)
    if not process_mgr.is_running("nav"):
        return nav_start(req)
    r = process_mgr.run_once(
        ["ros2", "service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap",
         json.dumps({"map_url": path})], timeout=15)
    if "result=0" not in r["output"]:
        raise HTTPException(500, "load_map 失败:\n" + r["output"][-800:])
    _state["map"] = req.name
    return {"ok": True}


@router.post("/nav/waypoints")
def nav_waypoints(req: WaypointsReq):
    if not process_mgr.is_running("nav"):
        raise HTTPException(409, "导航未启动")
    if not req.poses:
        raise HTTPException(400, "航点为空")
    goal = {"poses": [{
        "header": {"frame_id": "map"},
        "pose": {"position": {"x": p.x, "y": p.y, "z": 0.0},
                 "orientation": {"x": 0.0, "y": 0.0,
                                 "z": math.sin(p.yaw / 2), "w": math.cos(p.yaw / 2)}},
    } for p in req.poses]}
    process_mgr.start("waypoints",
        "ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "
        + shlex.quote(json.dumps(goal)) + " --feedback")
    return {"ok": True}


@router.post("/nav/cancel")
def nav_cancel():
    process_mgr.stop("waypoints")   # SIGINT 会让 action 客户端取消自己的 goal
    out = []
    for action in ("/navigate_to_pose", "/follow_waypoints"):
        r = process_mgr.run_once(
            ["ros2", "service", "call", f"{action}/_action/cancel_goal",
             "action_msgs/srv/CancelGoal", "{}"], timeout=6)
        out.append(r["output"][-300:])
    return {"ok": True, "output": "\n".join(out)}


@router.post("/stop")
def stop_mode():
    """退出建图/导航，保留底盘和 rosbridge"""
    for n in ("waypoints", "nav", "slam"):
        process_mgr.stop(n)
    return {"ok": True}


@router.post("/stop_all")
def stop_all():
    for n in ("waypoints", "nav", "slam", "base"):
        process_mgr.stop(n)
    return {"ok": True}


@router.get("/maps")
def list_maps():
    return [{"name": m["name"], "mtime": m["mtime"]} for m in maps.list_maps()]


@router.get("/logs")
async def stream_logs(since: int = 0):
    async def gen():
        seq, idle = since, 0
        while True:
            entries = process_mgr.logs_since(seq)
            for s, name, line in entries:
                yield f"data: {json.dumps({'seq': s, 'proc': name, 'line': line}, ensure_ascii=False)}\n\n"
                seq = s
            idle = 0 if entries else idle + 1
            if idle and idle % 30 == 0:
                yield ": ping\n\n"   # 保活，顺便让断开的连接尽早报错退出
            await asyncio.sleep(0.3)
    return StreamingResponse(gen(), media_type="text/event-stream")
