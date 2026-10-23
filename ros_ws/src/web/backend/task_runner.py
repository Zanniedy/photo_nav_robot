"""
任务层：goto / guide / patrol 状态机。纯 Python，不依赖 ROS；导航通过 NavPort 驱动
（生产环境 ros_link.RosLink，测试 tests/fakes.FakeNav）。

  goto    依次走到一个或多个点
  guide   带路：限速的单点 goto（默认 guide_speed）
  patrol  按路线巡航，可循环。多一个 waiting 状态：
            moving ──发现人 / wait()──▶ waiting ──resume() / 超时──▶ moving（去下一个没到的点）
          waiting 期间可以执行 goto / guide 子任务，做完仍在 waiting，等上层 resume

同一时间最多一个前台任务（goto / guide）和一个巡航：新前台任务顶掉旧前台任务，但只把巡航挂起；
新巡航顶掉旧巡航和它的子任务。
导航回调可能来自 ROS 线程，状态都在 self._lock 下改；取消 / 换目标后晚到的旧回调靠 _gen 丢掉。
"""
import math, secrets, threading, time
from collections import OrderedDict
from dataclasses import dataclass, field, fields
from typing import Callable, Protocol

HISTORY = 50                        # 保留最近多少个任务（含已结束的）
TERMINAL = ("succeeded", "failed", "canceled", "preempted")


class TaskError(Exception):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code
        self.msg = msg


class NavPort(Protocol):
    def nav_running(self) -> bool: ...
    def mapping(self) -> bool: ...
    def current_map_id(self) -> str | None: ...
    def loc_score(self) -> float | None: ...
    def pose(self) -> tuple[float, float, float] | None: ...
    def nearest_person(self) -> float | None: ...
    def send_goal(self, x: float, y: float, yaw: float, on_done: Callable[[bool, str], None]) -> None: ...
    def cancel_goal(self) -> None: ...
    def set_speed_limit(self, v: float | None) -> None: ...
    def set_estop(self, on: bool) -> None: ...


@dataclass
class RunnerConfig:
    default_speed: float = 0.25     # 估算剩余时间用
    guide_speed: float = 0.15       # guide 默认限速
    min_score: float = 0.0          # 定位分数门限，0 = 不检查
    wait_on_person: bool = False    # 巡航中发现人自动进入 waiting
    person_distance: float = 3.0
    wait_timeout: float = 120.0     # 0 = 一直等
    person_cooldown: float = 30.0   # resume 后多久内不因为发现人再停

    @classmethod
    def from_dict(cls, d: dict) -> "RunnerConfig":
        names = {f.name for f in fields(cls)}
        return cls(**{k: (bool(v) if k == "wait_on_person" else float(v))
                      for k, v in (d or {}).items() if k in names})


@dataclass
class Task:
    id: str
    type: str                        # goto / guide / patrol
    points: list[dict]               # 下发时的点位快照 {id, name, x, y, yaw}
    map_id: str | None
    max_speed: float | None = None
    loop: bool = False
    route_id: str | None = None
    parent: str | None = None        # 巡航中下发的子任务：所属巡航 id
    state: str = "pending"
    index: int = 0                   # 正在去（或 resume 后要去）的点
    laps: int = 0
    fails: int = 0                   # 巡航连续去不了的点数
    skipped: list[str] = field(default_factory=list)
    message: str = ""
    created: float = 0.0             # 墙钟时间，给人看
    updated: float = 0.0
    wait_since: float | None = None  # clock() 时间，算超时
    wait_reason: str = ""            # manual / person / subtask


class TaskRunner:
    def __init__(self, nav: NavPort, cfg: RunnerConfig | None = None,
                 emit: Callable[[dict], None] = lambda ev: None,
                 clock: Callable[[], float] = time.monotonic, estopped: bool = False):
        self.nav = nav
        self.cfg = cfg or RunnerConfig()
        self.estopped = estopped
        self._emit = emit
        self._clock = clock
        self._lock = threading.RLock()
        self._fg: Task | None = None          # goto / guide
        self._patrol: Task | None = None
        self._driving: Task | None = None     # 当前导航目标属于哪个任务
        self._gen = 0                         # 每发 / 撤一次目标 +1，旧回调对不上就丢
        self._cooldown_until = 0.0
        self._history: OrderedDict[str, Task] = OrderedDict()

    # ── 对外 ────────────────────────────────────────────────────────────

    def check(self, map_id: str | None = None) -> None:
        """下发 / 恢复前的安全检查，不通过抛 TaskError"""
        if self.estopped:
            raise TaskError("ESTOPPED", "急停中，先解除急停")
        if self.nav.mapping():
            raise TaskError("BUSY", "建图模式下不能下发任务")
        if not self.nav.nav_running():
            raise TaskError("NAV_NOT_RUNNING", "导航未启动")
        cur = self.nav.current_map_id()
        if map_id and map_id != cur:
            raise TaskError("MAP_MISMATCH", f"当前地图是 {cur}，不是 {map_id}")
        if self.cfg.min_score > 0:
            s = self.nav.loc_score()
            if s is None or s < self.cfg.min_score:
                raise TaskError("LOCALIZATION_UNRELIABLE",
                                f"定位不可靠（得分 {s}，门限 {self.cfg.min_score}）")

    def submit(self, type: str, points: list[dict], map_id: str | None = None,
               max_speed: float | None = None, loop: bool = False, route_id: str | None = None) -> dict:
        if type not in ("goto", "guide", "patrol"):
            raise TaskError("BAD_REQUEST", f"未知任务类型: {type}")
        if not points:
            raise TaskError("BAD_REQUEST", "没有要去的点")
        if type == "guide":
            if len(points) != 1:
                raise TaskError("BAD_REQUEST", "guide 只能去一个点")
            max_speed = max_speed or self.cfg.guide_speed
        with self._lock:
            self.check(map_id)
            now = time.time()
            t = Task(id=secrets.token_hex(4), type=type, points=[dict(p) for p in points],
                     map_id=self.nav.current_map_id(), max_speed=max_speed,
                     loop=bool(loop) and type == "patrol", route_id=route_id, created=now, updated=now)
            if self._fg is not None:
                self._finish(self._fg, "preempted", "被新任务顶掉")
            if type == "patrol":
                if self._patrol is not None:
                    self._finish(self._patrol, "preempted", "被新的巡航顶掉")
                self._patrol = t
            else:
                if self._patrol is not None:
                    self._suspend(self._patrol, "subtask")
                    t.parent = self._patrol.id
                self._fg = t
            self._remember(t)
            self._set_state(t, "running")
            self._drive(t)
            return self._summary(t)

    def cancel(self, task_id: str) -> dict:
        with self._lock:
            t = self._active(task_id)
            self._finish(t, "canceled", "已取消")
            return self._summary(t)

    def wait(self, task_id: str) -> dict:
        with self._lock:
            t = self._active_patrol(task_id)
            self._suspend(t, "manual")
            return self._summary(t)

    def resume(self, task_id: str) -> dict:
        with self._lock:
            t = self._active_patrol(task_id)
            if t.state != "waiting":
                return self._summary(t)
            if self._fg is not None:
                raise TaskError("BUSY", "子任务还在执行，等它结束或先取消")
            self.check()
            self._resume(t)
            return self._summary(t)

    def get(self, task_id: str) -> dict | None:
        with self._lock:
            t = self._history.get(task_id)
            return self._summary(t) if t else None

    def current(self) -> dict:
        with self._lock:
            return {"foreground": self._summary(self._fg) if self._fg else None,
                    "patrol": self._summary(self._patrol) if self._patrol else None}

    def estop(self) -> None:
        with self._lock:
            self.estopped = True
            self.nav.set_estop(True)
            self.cancel_all("急停")
            self._emit({"type": "estop", "on": True})

    def release_estop(self) -> None:
        with self._lock:
            self.estopped = False
            self.nav.set_estop(False)
            self._emit({"type": "estop", "on": False})

    def cancel_all(self, reason: str) -> None:
        self._end_all("canceled", reason)

    def tick(self) -> None:
        """外部每 0.5s 调一次：导航停了就结束任务、等待超时、巡航中发现人"""
        with self._lock:
            if (self._fg or self._patrol) and not self.nav.nav_running():
                self._end_all("failed", "导航已停止")
                return
            p = self._patrol
            if p is None:
                return
            now = self._clock()
            if p.state == "waiting":
                if self._fg is None and self.cfg.wait_timeout > 0 and now - p.wait_since >= self.cfg.wait_timeout:
                    try:
                        self.check()
                    except TaskError as e:
                        p.wait_since = now                 # 过一个超时周期再试
                        p.message = f"等待超时但不能继续: {e.msg}"
                        return
                    self._emit_task("wait_timeout", p)
                    self._resume(p)
            elif self._driving is p and self.cfg.wait_on_person and now >= self._cooldown_until:
                d = self.nav.nearest_person()
                if d is not None and d <= self.cfg.person_distance:
                    self._emit_task("person_found", p, distance=round(d, 2))
                    self._suspend(p, "person")

    # ── 内部（调用方已持有 self._lock）──────────────────────────────────

    def _end_all(self, state: str, reason: str) -> None:
        with self._lock:
            for t in (self._fg, self._patrol):     # 先前台再巡航
                if t is not None and t.state not in TERMINAL:
                    self._finish(t, state, reason)

    def _active(self, task_id: str) -> Task:
        for t in (self._fg, self._patrol):
            if t is not None and t.id == task_id:
                return t
        old = self._history.get(task_id)
        if old is not None:
            raise TaskError("BAD_REQUEST", f"任务已结束（{old.state}）")
        raise TaskError("NOT_FOUND", f"任务不存在: {task_id}")

    def _active_patrol(self, task_id: str) -> Task:
        t = self._active(task_id)
        if t.type != "patrol":
            raise TaskError("BAD_REQUEST", "只有巡航任务能 wait / resume")
        return t

    def _remember(self, t: Task) -> None:
        self._history[t.id] = t
        while len(self._history) > HISTORY:
            old = next((k for k, v in self._history.items() if v.state in TERMINAL), None)
            if old is None:
                break
            del self._history[old]

    def _set_state(self, t: Task, state: str) -> None:
        t.state = state
        t.updated = time.time()
        self._emit_task("task_state", t)

    def _emit_task(self, type: str, t: Task, **extra) -> None:
        self._emit({"type": type, "task": self._summary(t), **extra})

    def _stop_nav(self) -> None:
        self._gen += 1
        self._driving = None
        self.nav.cancel_goal()
        self.nav.set_speed_limit(None)

    def _drive(self, t: Task) -> None:
        """让 t 去它的第 index 个点"""
        p = t.points[t.index]
        self._gen += 1
        g = self._gen
        self._driving = t
        self.nav.set_speed_limit(t.max_speed)
        self.nav.send_goal(float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0)),
                           lambda ok, msg: self._on_leg_done(g, ok, msg))

    def _suspend(self, p: Task, reason: str) -> None:
        if p.state == "waiting":
            return
        if self._driving is p:
            self._stop_nav()
        p.state = "waiting"
        p.wait_since = self._clock()
        p.wait_reason = reason
        p.updated = time.time()
        self._emit_task("waiting", p, reason=reason)

    def _resume(self, p: Task) -> None:
        p.state = "running"
        p.wait_since = None
        p.wait_reason = ""
        p.updated = time.time()
        self._cooldown_until = self._clock() + self.cfg.person_cooldown
        self._emit_task("resumed", p)
        self._drive(p)

    def _on_leg_done(self, g: int, ok: bool, msg: str) -> None:
        with self._lock:
            if g != self._gen or self._driving is None:
                return                                   # 已取消 / 已换目标的旧回调
            t = self._driving
            self._driving = None
            self.nav.set_speed_limit(None)
            name = t.points[t.index]["name"]
            if ok:
                t.fails = 0
                t.message = f"已到达 {name}"
            elif t.type == "patrol":
                # 巡航里某个点去不了：跳过；连续一整圈都去不了才算失败
                t.fails += 1
                t.skipped = (t.skipped + [name])[-20:]
                if t.fails >= len(t.points):
                    self._finish(t, "failed", f"巡航失败：连续 {t.fails} 个点都去不了（{msg}）")
                    return
                t.message = f"去 {name} 失败，跳过（{msg}）"
            else:
                self._finish(t, "failed", f"去 {name} 失败: {msg}")
                return
            t.index += 1
            if t.index >= len(t.points):
                if not t.loop:
                    self._finish(t, "succeeded", t.message if ok else f"巡航结束，{t.message}")
                    return
                t.index = 0
                t.laps += 1
            t.updated = time.time()
            self._emit_task("progress", t)
            self._drive(t)

    def _finish(self, t: Task, state: str, msg: str) -> None:
        if self._driving is t:
            self._stop_nav()
        t.state = state
        t.message = msg
        t.wait_since = None
        t.updated = time.time()
        if t is self._fg:
            self._fg = None
            if self._patrol is not None and t.parent == self._patrol.id:
                self._patrol.wait_since = self._clock()   # 子任务结束后重新计等待超时
        if t is self._patrol:
            self._patrol = None
            if self._fg is not None and self._fg.parent == t.id:
                self._finish(self._fg, state, f"所属巡航已结束: {msg}")
        self._emit_task("task_state", t)

    def _summary(self, t: Task) -> dict:
        active = t.state in ("running", "waiting")
        target = t.points[t.index] if active and t.index < len(t.points) else None
        remaining = None
        pose = self.nav.pose() if target else None
        if target and pose:
            remaining = math.hypot(target["x"] - pose[0], target["y"] - pose[1])
            rest = t.points[t.index:]
            remaining += sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rest, rest[1:]))
        speed = t.max_speed or self.cfg.default_speed
        wait_left = None
        if t.state == "waiting" and self.cfg.wait_timeout > 0 and self._fg is None and t.wait_since is not None:
            wait_left = round(max(0.0, self.cfg.wait_timeout - (self._clock() - t.wait_since)), 1)
        return {
            "id": t.id, "type": t.type, "state": t.state, "map_id": t.map_id,
            "points": [p["name"] for p in t.points], "index": t.index,
            "target": target["name"] if target else None,
            "loop": t.loop, "laps": t.laps, "route_id": t.route_id, "parent": t.parent,
            "max_speed": t.max_speed, "message": t.message, "skipped": list(t.skipped),
            "remaining_m": round(remaining, 2) if remaining is not None else None,
            "eta_s": round(remaining / speed, 1) if remaining is not None else None,
            "wait_reason": t.wait_reason or None if t.state == "waiting" else None,
            "wait_left_s": wait_left,
            "created": t.created, "updated": t.updated,
        }
