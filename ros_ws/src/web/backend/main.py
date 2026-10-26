import os, threading
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

import auth, api_errors, patrol_file, process_mgr, runtime
from config import AUTOSTART, PROCESSES
from routers import annotations, captures, commands, diag, rosproxy, stack, v1_robot, v1_data, v1_tasks


def _autostart():
    for name in AUTOSTART:
        process_mgr.start(name, PROCESSES[name])
    # 底盘先起来几秒，再按默认地图进导航
    threading.Timer(3.0, stack.autostart_default_map).start()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    runtime.start()          # 任务层先起来，autostart 进导航后就能接任务
    patrol_file.sync(log=lambda s: process_mgr.log("ui", s))
    _autostart()
    yield
    try:
        runtime.stop()
    finally:
        # 后端退出时把自己拉起的 ROS 进程全部收掉，不留孤儿占串口（runtime.stop 出错也要收）
        process_mgr.stop_all()


app = FastAPI(title="MAX_NAV_ROBOT", lifespan=lifespan)


@app.middleware("http")
async def require_login(request: Request, call_next):
    # 静态页面放行（登录页在前端里）；/api/v1 认 cookie 或 Bearer token，其余 /api 除登录接口外要 cookie
    p = request.url.path
    if p.startswith("/api/v1/"):
        who = auth.identify(request)
        if who is None:
            return api_errors.fail_response("UNAUTHORIZED", "未登录或 token 无效")
        request.state.caller = who
    elif p.startswith("/api/") and not p.startswith("/api/auth/") \
            and not auth.check_token(request.cookies.get(auth.COOKIE)):
        return JSONResponse({"detail": "未登录"}, status_code=401)
    return await call_next(request)


api_errors.install(app)

app.include_router(auth.router)
app.include_router(rosproxy.router)
app.include_router(stack.router)
app.include_router(annotations.router)
app.include_router(captures.router)
app.include_router(commands.router)
app.include_router(diag.router)
app.include_router(v1_robot.router)
app.include_router(v1_data.router)
app.include_router(v1_tasks.router)

static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
