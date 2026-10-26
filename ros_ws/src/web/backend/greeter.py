"""巡逻中遇到人 → 走过去 → LED 打招呼 → 继续巡逻。纯 Python，不依赖 ROS（单测 tests/test_greeter.py）

    idle ──看到人脸且距离 <= detect_distance──▶ search（暂停巡逻，/person/approach 走向这个人；屏不变）
    search ──跟踪距离 <= greet_distance──▶ greet（屏显示 text）
    search ──锁定后人丢了 / 一直没锁定 / 导航结束还没进 greet_distance / 超时──▶ idle（继续巡逻）
      （/person/approach 刚开始时 person_tracker 还在 searching，没有锁定目标，这段不算「丢了」）
    greet ──hold 秒──▶ idle（屏换回眼睛，取消 approach，继续巡逻）
    进入 greet 时同时播一句语音（voice.py 的 greet_<语言>.wav）
    回到 idle 后 cooldown 秒内不再触发

近处人脸常常拍不全，所以只有「进入 search」要看到脸；之后的距离用 person_tracker 的轨迹（相机丢了靠激光继续跟）。
LED 由 led_display 的 led_node 驱动（/led/text、/led/eyes），这里只决定什么时候发。
"""
import time
from dataclasses import dataclass, fields
from typing import Callable, Protocol


class Port(Protocol):
    def nearest_person(self, require_face: bool = False) -> float | None: ...
    def target_distance(self) -> float | None: ...         # person_tracker 当前锁定目标的距离，没有锁定 → None
    def approach_start(self, distance: float, timeout: float) -> bool: ...
    def approach_cancel(self) -> None: ...
    def approach_state(self) -> str: ...                     # idle / running / succeeded / failed
    def led_text(self, text: str) -> None: ...
    def led_eyes(self) -> None: ...


class PatrolPort(Protocol):
    def active(self) -> bool: ...       # 有 running / waiting 的巡逻
    def pause(self) -> None: ...
    def resume(self) -> str: ...        # 返回空串 = 成功，否则是原因


@dataclass
class GreeterConfig:
    enabled: bool = True
    only_patrol: bool = True        # false：不巡逻也触发（调试用，不暂停/恢复巡逻）
    text: str = "hello i am max"
    detect_distance: float = 5.0    # 看到人脸、这个距离以内 → search
    greet_distance: float = 2.0     # 跟踪距离到这个以内 → greet
    approach_distance: float = 1.5  # /person/approach 停在离人多远（要小于 greet_distance）
    search_timeout: float = 40.0    # search 最长多久 s
    lost_timeout: float = 3.0       # search 中锁定的目标丢了多久算丢 s
    lock_timeout: float = 8.0       # approach 开始后多久还没锁定就放弃 s（person_tracker 自己 search_timeout 5s）
    hold: float = 8.0               # 打招呼显示多久 s
    cooldown: float = 20.0          # 回到 idle 后多久内不再触发 s
    voice: str = "greet"            # 打招呼时播的语音，空串 = 不播

    @classmethod
    def from_dict(cls, d: dict) -> "GreeterConfig":
        out = {}
        for f in fields(cls):
            if f.name in (d or {}):
                v = d[f.name]
                t = f.type if isinstance(f.type, str) else f.type.__name__
                out[f.name] = bool(v) if t == "bool" else str(v) if t == "str" else float(v)
        return cls(**out)


class Greeter:
    def __init__(self, port: Port, patrol: PatrolPort, cfg: GreeterConfig | None = None,
                 emit: Callable[[dict], None] = lambda ev: None, clock: Callable[[], float] = time.monotonic,
                 say: Callable[[str], object] = lambda key: None):
        self.port = port
        self.patrol = patrol
        self.cfg = cfg or GreeterConfig()
        self._emit = emit
        self._clock = clock
        self._say = say
        self.state = "idle"
        self.message = ""
        self.distance: float | None = None
        self._t0 = 0.0              # 进入当前状态的时间
        self._seen = 0.0            # search 中最近一次有目标距离
        self._locked = False        # search 中 person_tracker 锁定过目标
        self._cool_until = 0.0
        self._paused = False        # 是我们暂停的巡逻，结束时要恢复

    # ── 对外 ────────────────────────────────────────────────────────────

    def tick(self) -> None:
        """外部每 0.5s 调一次"""
        now = self._clock()
        if self._paused and not self.patrol.active():      # 用户取消了巡逻
            self._paused = False
            self._finish(now, "巡逻已结束")
            return
        getattr(self, f"_tick_{self.state}")(now)

    def cancel(self, reason: str = "已取消") -> None:
        if self.state != "idle":
            self._finish(self._clock(), reason)

    def status(self) -> dict:
        now = self._clock()
        return {"enabled": self.cfg.enabled, "state": self.state, "message": self.message,
                "distance": None if self.distance is None else round(self.distance, 2),
                "elapsed_s": round(now - self._t0, 1) if self.state != "idle" else None,
                "cooldown_s": round(max(0.0, self._cool_until - now), 1)}

    # ── 状态 ────────────────────────────────────────────────────────────

    def _tick_idle(self, now: float) -> None:
        c = self.cfg
        if not c.enabled or now < self._cool_until:
            return
        on_patrol = self.patrol.active()
        if c.only_patrol and not on_patrol:
            return
        d = self.port.nearest_person(True)
        if d is None or d > c.detect_distance:
            return
        self.distance = d
        if on_patrol:
            self.patrol.pause()
            self._paused = True
        if d <= c.greet_distance:               # 已经很近了，不用走
            self._enter(now, "search", f"看到人脸 {d:.1f}m")
            self._greet(now, d)
            return
        if not self.port.approach_start(c.approach_distance, c.search_timeout):
            self._finish(now, "approach 启动失败")
            return
        self._seen, self._locked = now, False
        self._enter(now, "search", f"看到人脸 {d:.1f}m，走过去")

    def _tick_search(self, now: float) -> None:
        c = self.cfg
        d = self.port.target_distance()
        if d is not None:
            self._seen, self.distance, self._locked = now, d, True
            if d <= c.greet_distance:
                self._greet(now, d)
                return
        st = self.port.approach_state()
        if self._locked and now - self._seen > c.lost_timeout:
            self._finish(now, "目标丢了")
        elif st in ("succeeded", "failed"):
            self._finish(now, f"approach 已结束（{st}），没进 {c.greet_distance:.1f}m")
        elif not self._locked and now - self._t0 > c.lock_timeout:
            self._finish(now, f"{c.lock_timeout:.0f}s 没锁定目标")
        elif now - self._t0 > c.search_timeout:
            self._finish(now, "搜索超时")

    def _tick_greet(self, now: float) -> None:
        d = self.port.target_distance()
        if d is not None:
            self.distance = d
        if now - self._t0 >= self.cfg.hold:
            self._finish(now, "打完招呼")

    # ── 内部 ────────────────────────────────────────────────────────────

    def _enter(self, now: float, state: str, msg: str) -> None:
        self.state, self._t0, self.message = state, now, msg
        self._emit({"type": f"greet_{state}", "message": msg,
                    "distance": None if self.distance is None else round(self.distance, 2)})

    def _greet(self, now: float, d: float) -> None:
        self.port.led_text(self.cfg.text)
        if self.cfg.voice:
            self._say(self.cfg.voice)
        self._enter(now, "greet", f"距离 {d:.1f}m，打招呼")

    def _finish(self, now: float, reason: str) -> None:
        if self.state == "greet":
            self.port.led_eyes()
        if self.port.approach_state() == "running":
            self.port.approach_cancel()
        if self._paused:
            self._paused = False
            err = self.patrol.resume()
            if err:
                reason += f"；巡逻没能恢复: {err}"
        self._cool_until = now + self.cfg.cooldown
        self._enter(now, "idle", reason)
