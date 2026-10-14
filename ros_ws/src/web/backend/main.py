import os
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

import process_mgr
from config import AUTOSTART, PROCESSES
from routers import commands, diag, stack


@asynccontextmanager
async def lifespan(_app: FastAPI):
    for name in AUTOSTART:
        process_mgr.start(name, PROCESSES[name])
    yield
    # 后端退出时把自己拉起的 ROS 进程全部收掉，不留孤儿占串口
    process_mgr.stop_all()


app = FastAPI(title="Robot UI", lifespan=lifespan)
app.include_router(stack.router)
app.include_router(commands.router)
app.include_router(diag.router)

static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
