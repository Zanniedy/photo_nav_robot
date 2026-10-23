"""事件推送：任务层（任意线程）→ 各个 /api/v1/events WebSocket（asyncio）。慢的客户端丢最旧的事件"""
import asyncio, threading


def _put(q: asyncio.Queue, ev: dict) -> None:
    if q.full():
        q.get_nowait()
    q.put_nowait(ev)


class EventHub:
    def __init__(self, maxsize: int = 200):
        self._subs: set[tuple] = set()
        self._lock = threading.Lock()
        self._maxsize = maxsize

    def subscribe(self) -> tuple:
        """在 asyncio 协程里调用；返回 (loop, queue)，退出时 unsubscribe"""
        sub = (asyncio.get_running_loop(), asyncio.Queue(self._maxsize))
        with self._lock:
            self._subs.add(sub)
        return sub

    def unsubscribe(self, sub: tuple) -> None:
        with self._lock:
            self._subs.discard(sub)

    def publish(self, ev: dict) -> None:
        with self._lock:
            subs = list(self._subs)
        for loop, q in subs:
            try:
                loop.call_soon_threadsafe(_put, q, ev)
            except RuntimeError:          # 事件循环已关闭
                self.unsubscribe((loop, q))
