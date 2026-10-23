"""
/api/v1 统一返回格式：
  成功 {"ok": true,  "robot_id": ..., "data": ...}
  失败 {"ok": false, "robot_id": ..., "code": "...", "msg": "..."}
HTTP 状态码按 code 定（见 STATUS），调用方以 code 为准。旧 /api 接口不受影响。
"""
import traceback
from fastapi import FastAPI, Request
from fastapi.exception_handlers import request_validation_exception_handler
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
import robot_config

STATUS = {
    "UNAUTHORIZED": 401, "BAD_REQUEST": 400, "NOT_FOUND": 404, "AMBIGUOUS": 409,
    "NAV_NOT_RUNNING": 409, "LOCALIZATION_UNRELIABLE": 409, "MAP_MISMATCH": 409,
    "ESTOPPED": 409, "BUSY": 409, "NAV_FAILED": 502, "ROBOT_NOT_CONFIGURED": 409, "INTERNAL": 500,
}


class ApiError(Exception):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code if code in STATUS else "INTERNAL"
        self.msg = msg

    @property
    def status(self) -> int:
        return STATUS[self.code]


def ok(data=None) -> dict:
    return {"ok": True, "robot_id": robot_config.robot_id(), "data": data}


def fail_body(code: str, msg: str) -> dict:
    return {"ok": False, "robot_id": robot_config.robot_id(), "code": code, "msg": msg}


def fail_response(code: str, msg: str) -> JSONResponse:
    return JSONResponse(fail_body(code, msg), status_code=STATUS.get(code, 500))


def install(app: FastAPI) -> None:
    @app.exception_handler(ApiError)
    async def _api_error(_req: Request, e: ApiError):
        return fail_response(e.code, e.msg)

    @app.exception_handler(RequestValidationError)
    async def _validation(req: Request, e: RequestValidationError):
        if not req.url.path.startswith("/api/v1/"):
            return await request_validation_exception_handler(req, e)   # 旧接口保持 422
        msg = "; ".join(f"{'.'.join(str(x) for x in err['loc'][1:])}: {err['msg']}" for err in e.errors())
        return fail_response("BAD_REQUEST", msg or "参数错误")

    @app.exception_handler(Exception)
    async def _unexpected(req: Request, e: Exception):
        # 没预料到的异常：/api/v1 也按统一格式返回，traceback 进 ui 日志
        if not req.url.path.startswith("/api/v1/"):
            return JSONResponse({"detail": "Internal Server Error"}, status_code=500)
        import process_mgr                         # 放这里避免循环 import
        for line in traceback.format_exception(type(e), e, e.__traceback__):
            for ln in line.rstrip().splitlines():
                process_mgr.log("ui", ln)
        return fail_response("INTERNAL", f"内部错误: {type(e).__name__}")
