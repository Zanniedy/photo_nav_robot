"""进程编排：底盘 / 建图 / 导航模式切换、地图、初始位姿、航点、日志流"""
import asyncio, json, math, shlex, threading, time
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import maps, patrol_file, process_mgr, runtime, state, tasklog
from api_errors import STATUS
from config import PROCESSES
from task_runner import TaskError

router = APIRouter(prefix="/api")

# 前端展示顺序；waypoints 是 FollowWaypoints action 客户端进程
PROC_NAMES = ["rosbridge", "base", "slam", "nav", "waypoints", "points", "camera", "calib"]
# 可以单独启动 / 按配置命令重启的进程（nav/slam/waypoints 走各自接口）
STANDALONE = ("rosbridge", "base", "points", "camera", "calib", "person")
_state = {"map": None}       # 当前导航地图 id（yaml 绝对路径）


class MapReq(BaseModel):
    map: str                 # 地图 id（yaml 路径）或地图名


class SaveReq(BaseModel):
    name: str


class DefaultReq(BaseModel):
    map: str | None = None   # None = 取消默认地图


class Pose2D(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class MapPoseReq(Pose2D):
    map: str


class InitPoseReq(Pose2D):
    refine: bool = False     # True → /initialpose_rough 经 relocalize_node 精配


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


def _map_or_404(map_id: str) -> dict:
    m = maps.resolve(map_id)
    if m is None:
        raise HTTPException(404, f"地图不存在: {map_id}")
    return m


def _quat(yaw: float) -> dict:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def publish_initialpose(x: float, y: float, yaw: float, refine: bool = False, wait: float = 3) -> dict:
    """经 ros2 topic pub 发初始位姿；wait>0 时等到有订阅者（定位节点起来）再发"""
    topic = "/initialpose_rough" if refine else "/initialpose"
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25
    cov[35] = 0.0685
    msg = {"header": {"frame_id": "map"},
           "pose": {"pose": {"position": {"x": x, "y": y, "z": 0.0}, "orientation": _quat(yaw)},
                    "covariance": cov}}
    r = process_mgr.run_once(
        ["ros2", "topic", "pub", "--once", "-w", "1", topic,
         "geometry_msgs/msg/PoseWithCovarianceStamped", json.dumps(msg)], timeout=wait + 5)
    ok = not r["timed_out"] and r["rc"] == 0
    process_mgr.log("ui", f"初始位姿 {topic} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) "
                                  + ("已发送" if ok else "发送失败"))
    return {"ok": ok, "topic": topic, "output": r["output"][-500:]}


def _apply_saved_pose_async(map_id: str) -> None:
    """导航启动/换图后，若该地图存了初始位姿，等定位节点起来后自动发一次"""
    pose = state.load()["poses"].get(map_id)
    if not pose:
        return

    def job():
        # 等 nav launch 起定位节点；-w 1 会一直等到有订阅者为止（上限 60s）
        time.sleep(2)
        if _state["map"] == map_id and process_mgr.is_running("nav"):
            publish_initialpose(pose["x"], pose["y"], pose["yaw"], wait=60)
    threading.Thread(target=job, daemon=True).start()


def set_current_map(map_id: str) -> None:
    """当前导航地图：内存一份，state_file 一份给 points_server 读"""
    _state["map"] = map_id
    state.update(lambda d: d.__setitem__("current_map", map_id))


def current_map() -> str | None:
    return _state["map"] if process_mgr.is_running("nav") else None


def current_map_id() -> str | None:
    """对外的当前地图 id（地图名），不在导航返回 None"""
    cur = current_map()
    return maps.name_of(cur) if cur else None


def submit_legacy(points: list[dict]) -> dict:
    """旧接口（points_goto / nav/waypoints）下发 goto；错误转成旧格式的 HTTPException"""
    if runtime.runner is None:
        raise HTTPException(500, "任务层没启动")
    try:
        return runtime.runner.submit("goto", points)
    except TaskError as e:
        raise HTTPException(STATUS.get(e.code, 400), e.msg)


def start_nav(m: dict) -> None:
    runtime.cancel_all("切换地图")
    process_mgr.stop("waypoints")
    process_mgr.stop("slam")
    _ensure_base()
    process_mgr.start("nav", PROCESSES["nav"].replace("{map}", shlex.quote(m["id"])))
    set_current_map(m["id"])
    patrol_file.sync(m["id"], log=lambda s: process_mgr.log("ui", s))   # 改了路线文件，重新进导航就生效
    _apply_saved_pose_async(m["id"])


def autostart_default_map() -> None:
    """后端启动时调用：配置了默认地图就直接进导航"""
    mid = state.load()["default_map"]
    if not mid:
        return
    m = maps.resolve(mid)
    if m is None:
        process_mgr.log("ui", f"默认地图不存在，跳过自动导航: {mid}")
        return
    process_mgr.log("ui", f"自动以默认地图 {m['name']} 启动导航")
    start_nav(m)


@router.get("/status")
def status():
    nav = process_mgr.is_running("nav")
    return {
        "mode": _mode(),
        "map": _state["map"] if nav else None,
        "default_map": state.load()["default_map"],
        "processes": {n: process_mgr.info(n) for n in PROC_NAMES},
        "time": time.time(),
    }


@router.post("/proc/{name}/start")
def proc_start(name: str):
    if name not in STANDALONE:
        raise HTTPException(400, f"只能单独启动 {' / '.join(STANDALONE)}，建图导航用对应接口")
    process_mgr.start(name, PROCESSES[name])
    return {"ok": True}


@router.post("/proc/{name}/restart")
def proc_restart(name: str):
    """按上次的启动命令重启（nav 保留当前地图）。base 重启后上层的定位需要重新给初始位姿"""
    if name not in PROC_NAMES or name == "waypoints":
        raise HTTPException(400, f"不能重启 {name}")
    cmd = process_mgr.last_cmd(name) or (PROCESSES[name] if name in STANDALONE + ("slam",) else None)
    if cmd is None:
        raise HTTPException(409, f"{name} 还没启动过")
    process_mgr.start(name, cmd)
    if name == "nav" and _state["map"]:
        _apply_saved_pose_async(_state["map"])
    return {"ok": True}


@router.post("/proc/{name}/stop")
def proc_stop(name: str):
    if name not in PROC_NAMES:
        raise HTTPException(404, name)
    process_mgr.stop(name)
    return {"ok": True}


@router.post("/mapping/start")
def mapping_start():
    runtime.cancel_all("切换到建图")
    process_mgr.stop("waypoints")
    process_mgr.stop("nav")
    _ensure_base()
    process_mgr.start("slam", PROCESSES["slam"])
    return {"ok": True}


@router.post("/mapping/save")
def mapping_save(req: SaveReq):
    if not maps.valid_name(req.name):
        raise HTTPException(400, "地图名只能包含字母数字 _ - .")
    if not process_mgr.is_running("slam"):
        raise HTTPException(409, "当前不在建图模式")
    prefix = maps.save_prefix(req.name)
    r = process_mgr.run_once(
        ["ros2", "service", "call", "/slam_toolbox/save_map", "slam_toolbox/srv/SaveMap",
         json.dumps({"name": {"data": prefix}})], timeout=45)   # 内部起 map_saver_cli，ARM 上冷启动慢
    # 响应形如 slam_toolbox.srv.SaveMap_Response(result=0)
    if "result=0" not in r["output"]:
        raise HTTPException(500, "save_map 失败:\n" + r["output"][-800:])
    # 顺带存 posegraph，以后想用 slam_toolbox localization 模式可以直接用；失败不影响地图本身
    r2 = process_mgr.run_once(
        ["ros2", "service", "call", "/slam_toolbox/serialize_map",
         "slam_toolbox/srv/SerializePoseGraph", json.dumps({"filename": prefix})], timeout=45)
    posegraph = "result=0" in r2["output"]
    if not posegraph:
        process_mgr.log("ui", "serialize_map 失败:\n" + r2["output"][-400:])
    return {"ok": True, "yaml": prefix + ".yaml", "posegraph": posegraph}


@router.post("/nav/start")
def nav_start(req: MapReq):
    start_nav(_map_or_404(req.map))
    return {"ok": True}


@router.post("/nav/switch_map")
def nav_switch_map(req: MapReq):
    """导航中热切换地图（map_server load_map），不在导航就直接以该地图启动导航"""
    m = _map_or_404(req.map)
    runtime.cancel_all("切换地图")                 # 地图确实存在才取消任务
    if not process_mgr.is_running("nav"):
        return nav_start(req)
    r = process_mgr.run_once(
        ["ros2", "service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap",
         json.dumps({"map_url": m["id"]})], timeout=15)
    if "result=0" not in r["output"]:
        raise HTTPException(500, "load_map 失败:\n" + r["output"][-800:])
    process_mgr.stop("waypoints")
    set_current_map(m["id"])
    _apply_saved_pose_async(m["id"])
    return {"ok": True}


@router.post("/nav/initialpose")
def nav_initialpose(req: InitPoseReq):
    if not process_mgr.is_running("nav"):
        raise HTTPException(409, "导航未启动")
    r = publish_initialpose(req.x, req.y, req.yaw, req.refine)
    if not r["ok"]:
        raise HTTPException(500, f"发送 {r['topic']} 失败（定位节点没起来？）\n{r['output']}")
    return r


@router.post("/nav/waypoints")
def nav_waypoints(req: WaypointsReq):
    """地图上点的航点：经任务层依次 NavigateToPose（原来是 FollowWaypoints 子进程，拿不到结果）"""
    if not process_mgr.is_running("nav"):
        raise HTTPException(409, "导航未启动")
    if not req.poses:
        raise HTTPException(400, "航点为空")
    t = submit_legacy([{"id": f"wp{i}", "name": f"航点{i}", "x": p.x, "y": p.y, "yaw": p.yaw}
                       for i, p in enumerate(req.poses, 1)])
    return {"ok": True, "task_id": t["id"]}


@router.post("/nav/cancel")
def nav_cancel():
    runtime.cancel_all("网页取消导航")
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
    runtime.cancel_all("退出导航")
    for n in ("waypoints", "nav", "slam"):
        process_mgr.stop(n)
    return {"ok": True}


@router.post("/stop_all")
def stop_all():
    runtime.cancel_all("退出导航")
    for n in ("waypoints", "nav", "slam", "base"):
        process_mgr.stop(n)
    return {"ok": True}


@router.get("/maps")
def list_maps():
    st = state.load()
    return [{**m, "default": m["id"] == st["default_map"], "pose": st["poses"].get(m["id"])}
            for m in maps.list_maps()]


@router.post("/maps/default")
def set_default_map(req: DefaultReq):
    mid = _map_or_404(req.map)["id"] if req.map else None
    state.update(lambda d: d.__setitem__("default_map", mid))
    return {"ok": True, "default_map": mid}


@router.post("/maps/pose")
def save_map_pose(req: MapPoseReq):
    mid = _map_or_404(req.map)["id"]
    state.update(lambda d: d["poses"].__setitem__(mid, {"x": req.x, "y": req.y, "yaw": req.yaw}))
    return {"ok": True}


@router.delete("/maps/pose")
def clear_map_pose(map: str):
    mid = _map_or_404(map)["id"]
    state.update(lambda d: d["poses"].pop(mid, None))
    return {"ok": True}


@router.get("/tasklog")
def task_log(since: int = 0):
    """网页「任务」页签：当前任务 / 打招呼状态 + since 之后的任务日志"""
    r, g, m = runtime.runner, runtime.greeter, runtime.mission
    return {"current": r.current() if r is not None else None,
            "greeter": g.status() if g is not None else None,
            "mission": {**m.status(), "line": m.report()} if m is not None else None,
            "lines": tasklog.since(since)}


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
