"""测试替身：临时地图文件、任务层假导航 / 时钟 / 点位"""
import os
from conftest import TMP


def make_map(name: str, pixels: bytes = b"\x00" * 16, subdir: str = "") -> str:
    """在临时 maps 目录写一张 4x4 的 pgm + yaml，返回 yaml 路径"""
    d = os.path.join(TMP, "maps", subdir)
    os.makedirs(d, exist_ok=True)
    pgm = os.path.join(d, f"{name}.pgm")
    y = os.path.join(d, f"{name}.yaml")
    # 本机连续两次写有时会落在同一毫秒，st_mtime_ns 不变，maps.version 的缓存就刷不掉
    pre = {p: (os.stat(p).st_mtime_ns if os.path.exists(p) else None) for p in (pgm, y)}
    with open(pgm, "wb") as f:
        f.write(b"P5 4 4 255\n" + pixels)
    with open(y, "w") as f:
        f.write(f"image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    for p, old in pre.items():
        if old is not None and os.stat(p).st_mtime_ns <= old:
            os.utime(p, ns=(old + 1_000_000, old + 1_000_000))
    return y


def pt(name: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> dict:
    return {"id": f"id-{name}", "name": name, "x": x, "y": y, "yaw": yaw}


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeNav:
    """task_runner.NavPort 的替身：记录收到的目标，测试里手动 arrive() / fail()"""
    def __init__(self):
        self.running, self.is_mapping, self.map_id = True, False, "office"
        self.score: float | None = 0.9
        self.robot_pose: tuple | None = (0.0, 0.0, 0.0)
        self.person: float | None = None
        self.goals: list[tuple] = []
        self.callback = None
        self.cancels = 0
        self.speed: float | None = None
        self.estop = False

    def nav_running(self): return self.running
    def mapping(self): return self.is_mapping
    def current_map_id(self): return self.map_id
    def loc_score(self): return self.score
    def pose(self): return self.robot_pose
    def nearest_person(self): return self.person

    def send_goal(self, x, y, yaw, on_done):
        self.goals.append((x, y, yaw))
        self.callback = on_done

    def cancel_goal(self):
        self.cancels += 1

    def set_speed_limit(self, v): self.speed = v
    def set_estop(self, on): self.estop = on

    def arrive(self):
        cb, self.callback = self.callback, None
        x, y, yaw = self.goals[-1]
        self.robot_pose = (x, y, yaw)
        cb(True, "到达")

    def fail(self, msg: str = "导航失败"):
        cb, self.callback = self.callback, None
        cb(False, msg)
