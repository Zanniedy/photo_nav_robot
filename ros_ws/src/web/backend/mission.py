"""任务总状态机：把巡逻（task_runner）、定位（map_localizer 的 /localizer/state）、遇人（greeter）合成一个状态。
纯 Python，不依赖 ROS（单测 tests/test_mission.py）

    空闲 ──开始巡逻──▶ 巡逻 ──定位 lost──▶ 丢失 ──定位 recovering──▶ 恢复 ──定位 tracking──▶ 巡逻（自动继续）
                      巡逻 ──greeter search──▶ 人脸（走向人）──greeter greet──▶ 等待（屏显示 hello）──▶ 巡逻
    丢失/恢复期间：巡逻暂停（导航停下，免得带着错的位姿乱开），遇人状态机取消且不 tick；定位只要不是 lost/recovering
    （包括没开导航、没有数据）就照常
每 report_every 秒调一次 report() 打印当前状态（runtime 里接到任务日志和进程日志）。
"""
import time
from typing import Callable

LOST = ("lost", "recovering")


class Mission:
    def __init__(self, runner, greeter, localizer_state: Callable[[], dict | None],
                 log: Callable[[str, str], None] = lambda kind, text: None,
                 clock: Callable[[], float] = time.monotonic, report_every: float = 3.0):
        self.runner = runner
        self.greeter = greeter
        self.localizer_state = localizer_state     # () -> /localizer/state 的 dict，没有 / 太旧 → None
        self._log = log
        self._clock = clock
        self.report_every = report_every
        self.state = "空闲"
        self._since = clock()
        self._paused = False                       # 巡逻是因为定位丢失被我们暂停的
        self._last_report = -1e9

    # ── 对外 ────────────────────────────────────────────────────────────

    def tick(self) -> None:
        """外部每 0.5s 调一次（在 runner.tick 之后）；遇人状态机（greeter）也由这里驱动"""
        loc = self.localizer_state()
        lst = loc.get("state") if loc else None
        if lst in LOST:
            if not self._paused:
                if self.greeter.state != "idle":
                    self.greeter.cancel("定位丢失")      # 它暂停的巡逻会先被它恢复
                if self.runner.pause_patrol("lost"):
                    self._paused = True
                    self._log("mission", f"定位{'丢失' if lst == 'lost' else '恢复中'}，暂停巡逻")
        else:
            if self._paused:
                self._paused = False
                err = self.runner.resume_patrol()
                self._log("mission", "定位恢复，继续巡逻" if not err else f"定位恢复，但巡逻没能继续: {err}")
            self.greeter.tick()
        self._set(self._derive(lst, self._patrol()))
        now = self._clock()
        if now - self._last_report >= self.report_every:
            self._last_report = now
            self._log("tick", self.report())

    def report(self) -> str:
        """一行当前状态，例：[巡逻 12s] 巡逻 2/5 → 门口 | 定位 tracking 0.55 动态 3% | 遇人 空闲"""
        loc = self.localizer_state()
        patrol = self._patrol()
        g = self.greeter.status()
        parts = [f"[{self.state} {self._clock() - self._since:.0f}s]"]
        if patrol is not None:
            n = len(patrol.get("points") or [])
            tgt = f" → {patrol['target']}" if patrol.get("target") else ""
            wr = f"（{patrol['wait_reason']}）" if patrol.get("wait_reason") else ""
            parts.append(f"巡逻 {patrol['state']}{wr} {patrol['index'] + 1}/{n}{tgt}")
        else:
            parts.append("没有巡逻")
        if loc:
            s = f"定位 {loc.get('state')} {loc.get('score', 0):.2f} 动态 {100 * loc.get('dynamic', 0):.0f}%"
            if loc.get("state") == "recovering":
                rc = loc.get("recover") or {}
                s += f" 搜到 {rc.get('radius', 0):.1f}m 最好 {rc.get('score', 0):.2f}{' 确认中' if rc.get('confirming') else ''}"
            parts.append(s)
        else:
            parts.append("定位 无数据")
        gs = {"idle": "空闲", "search": "走向人", "greet": "打招呼"}.get(g["state"], g["state"])
        if g.get("distance") is not None and g["state"] != "idle":
            gs += f" {g['distance']:.1f}m"
        parts.append(f"遇人 {gs}")
        return " | ".join(parts)

    def status(self) -> dict:
        return {"state": self.state, "since_s": round(self._clock() - self._since, 1),
                "localizer": self.localizer_state(), "paused_for_lost": self._paused}

    # ── 内部 ────────────────────────────────────────────────────────────

    def _patrol(self) -> dict | None:
        return self.runner.current().get("patrol")

    def _derive(self, lst: str | None, patrol: dict | None) -> str:
        if lst == "lost":
            return "丢失"
        if lst == "recovering":
            return "恢复"
        g = self.greeter.state
        if g == "search":
            return "人脸"
        if g == "greet":
            return "等待"
        if patrol is not None and patrol["state"] == "running":
            return "巡逻"
        if patrol is not None and patrol["state"] == "waiting":
            return "暂停"
        return "空闲"

    def _set(self, st: str) -> None:
        if st == self.state:
            return
        self._log("mission", f"状态机 {self.state} → {st}")
        self.state, self._since = st, self._clock()
