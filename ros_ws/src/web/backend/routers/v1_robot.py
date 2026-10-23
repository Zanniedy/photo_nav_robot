"""/api/v1：机器人身份、调用 token（之后加 status、急停、事件推送）"""
from fastapi import APIRouter, Request
from pydantic import BaseModel
import process_mgr, robot_config
from api_errors import ApiError, ok

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
    if not robot_config.delete_token(name):
        raise ApiError("NOT_FOUND", f"token 不存在: {name}")
    process_mgr.log("ui", f"删除 API token: {name}")
    return ok()
