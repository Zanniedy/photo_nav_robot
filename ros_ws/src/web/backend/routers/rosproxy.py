"""
/ros：带登录校验的 rosbridge WebSocket 代理。rosbridge 本身只监听 127.0.0.1。
"""
import asyncio
import websockets
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import auth
from config import ROSBRIDGE_URL

router = APIRouter()


@router.websocket("/ros")
async def ros_proxy(ws: WebSocket):
    if not auth.check_token(ws.cookies.get(auth.COOKIE)):
        await ws.close(code=4401)
        return
    await ws.accept()
    try:
        upstream = await websockets.connect(ROSBRIDGE_URL, max_size=None, open_timeout=3)
    except Exception:
        await ws.close(code=1011)   # rosbridge 没起，前端会自动重连
        return

    async def down():   # 浏览器 → rosbridge
        while True:
            msg = await ws.receive()
            if msg["type"] == "websocket.disconnect":
                return
            if msg.get("text") is not None:
                await upstream.send(msg["text"])
            elif msg.get("bytes") is not None:
                await upstream.send(msg["bytes"])

    async def up():     # rosbridge → 浏览器
        async for m in upstream:
            if isinstance(m, str):
                await ws.send_text(m)
            else:
                await ws.send_bytes(m)

    tasks = [asyncio.create_task(down()), asyncio.create_task(up())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except WebSocketDisconnect:
        pass
    finally:
        for t in tasks:
            t.cancel()
        await upstream.close()
        try:
            await ws.close()
        except Exception:
            pass
