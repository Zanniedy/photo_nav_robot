from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import os

from routers import maps, slam, nav, status, lidar, serial_ports, diag

app = FastAPI(title="Robot Debug UI")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(maps.router)
app.include_router(slam.router)
app.include_router(nav.router)
app.include_router(status.router)
app.include_router(lidar.router)
app.include_router(serial_ports.router)
app.include_router(diag.router)

static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
