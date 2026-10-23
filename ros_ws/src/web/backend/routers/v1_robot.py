"""/api/v1：机器人身份、调用 token、状态、急停、事件推送（WebSocket）"""
import asyncio, time
from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
import auth, maps, process_mgr, robot_config, runtime
from api_errors import ApiError, ok
from routers import stack

router = APIRouter(prefix="/api/v1")


def web_only(request: Request) -> None:
    """改身份、管 token 只能网页登录后做，token 调用方不行"""
    if getattr(request.state, "caller", None) != "web":
        raise ApiError("UNAUTHORIZED", "只能在网页登录后操作")


def _cfg(fn, *args):
    try:
        return fn(*args)
    except robot_config.ConfigError as e:
        raise ApiError(e.code, str(e))


class RobotPut(BaseModel):
    robot_id: str | None = None
    name: str | None = None


class TokenReq(BaseModel):
    name: str


@router.get("/robot")
def get_robot():
    return ok(robot_config.public_info())


@router.put("/robot")
def put_robot(req: RobotPut, request: Request):
    web_only(request)
    return ok(_cfg(robot_config.set_identity, req.robot_id, req.name))


@router.post("/robot/tokens")
def create_token(req: TokenReq, request: Request):
    web_only(request)
    tok = _cfg(robot_config.create_token, req.name)
    process_mgr.log("ui", f"生成 API token: {req.name}")
    return ok({"name": req.name, "token": tok})


@router.delete("/robot/tokens/{name}")
def delete_token(name: str, request: Request):
    web_only(request)
    if not _cfg(robot_config.delete_token, name):
        raise ApiError("NOT_FOUND", f"token 不存在: {name}")
    process_mgr.log("ui", f"删除 API token: {name}")
    return ok()


def status_snapshot() -> dict:
    r, nav = runtime.runner, runtime.nav
    cur = stack.current_map()
    pose = nav.pose() if nav else None
    score = nav.loc_score() if nav else None
    min_score = r.cfg.min_score if r else 0.0
    info = robot_config.public_info()
    return {
        "robot_id": info["robot_id"], "name": info["name"], "mode": stack._mode(),
        "map": {"map_id": maps.name_of(cur), "version": maps.version(cur)} if cur else None,
        "pose": {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 3)} if pose else None,
        # reliable：没设门限（min_score=0）时为 None
        "localization": {"score": score,
                         "reliable": (score is not None and score >= min_score) if min_score > 0 else None},
        "estop": r.estopped if r else False,
        "tasks": r.current() if r else {"foreground": None, "patrol": None},
        "time": time.time(),
    }


@router.get("/status")
def status():
    return ok(status_snapshot())


@router.post("/estop")
def estop(request: Request):
    if runtime.runner is None:
        raise ApiError("INTERNAL", "任务层没启动")
    runtime.set_estop(True)
    process_mgr.log("ui", f"[{getattr(request.state, 'caller', '?')}] 急停")
    return ok({"estop": True})


@router.post("/estop/release")
def estop_release(request: Request):
    if runtime.runner is None:
        raise ApiError("INTERNAL", "任务层没启动")
    runtime.set_estop(False)
    process_mgr.log("ui", f"[{getattr(request.state, 'caller', '?')}] 解除急停")
    return ok({"estop": False})


@router.websocket("/events")
async def events(ws: WebSocket):
    """每秒推一次 {"type": "status", "status": ...}；任务事件立即推。认证：cookie、Bearer 头或 ?token="""
    if auth.identify_ws(ws) is None:
        await ws.close(code=4401)
        return
    await ws.accept()
    sub = runtime.hub.subscribe()
    last = 0.0
    try:
        while True:
            wait = 1.0 - (time.monotonic() - last)
            if wait > 0:
                try:
                    ev = await asyncio.wait_for(sub[1].get(), timeout=wait)
                    await ws.send_json({"robot_id": robot_config.robot_id(), **ev})
                    continue
                except asyncio.TimeoutError:
                    pass
            last = time.monotonic()
            await ws.send_json({"type": "status", "robot_id": robot_config.robot_id(),
                                "status": status_snapshot()})
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        runtime.hub.unsubscribe(sub)
