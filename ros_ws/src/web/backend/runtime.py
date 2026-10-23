"""
后端进程里的单例：任务层（TaskRunner）+ 导航接口（RosLink）+ 事件推送（EventHub）。
start() 在 FastAPI lifespan 里调；测试里用 install(FakeNav()) 代替，不起 ROS。
后端重启时正在跑的任务会中断（导航进程本来就归后端管）。急停状态写在 state 文件里，重启后保持。
"""
import os, threading
import process_mgr, state
from config import ROS_DOMAIN_ID, TASKS
from events import EventHub
from task_runner import RunnerConfig, TaskRunner

hub = EventHub()
runner: TaskRunner | None = None
nav = None
_stop = threading.Event()


class NullNav:
    """ROS 不可用时的占位：导航永远算没启动，任务都会被拒绝"""
    def nav_running(self): return False
    def mapping(self): return False
    def current_map_id(self): return None
    def loc_score(self): return None
    def pose(self): return None
    def nearest_person(self): return None
    def send_goal(self, x, y, yaw, on_done): on_done(False, "ROS 不可用")
    def cancel_goal(self): pass
    def set_speed_limit(self, v): pass
    def set_estop(self, on): pass
    def shutdown(self): pass


def install(nav_port, cfg: RunnerConfig | None = None, estopped: bool = False) -> TaskRunner:
    global runner, nav
    nav = nav_port
    runner = TaskRunner(nav_port, cfg or RunnerConfig.from_dict(TASKS), emit=hub.publish, estopped=estopped)
    nav_port.set_estop(estopped)
    return runner


def start() -> None:
    from routers import stack           # stack 也 import runtime，放这里避免循环 import
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    try:
        from ros_link import RosLink
        link = RosLink(nav_running=lambda: process_mgr.is_running("nav"),
                       mapping=lambda: process_mgr.is_running("slam"),
                       current_map_id=stack.current_map_id)
    except Exception as e:
        process_mgr.log("ui", f"任务层连不上 ROS（{e}），/api/v1/tasks 不可用")
        link = NullNav()
    install(link, estopped=state.load()["estop"])
    _stop.clear()
    threading.Thread(target=_ticker, daemon=True).start()


def _ticker() -> None:
    while not _stop.wait(0.5):
        try:
            runner.tick()
        except Exception as e:
            process_mgr.log("ui", f"任务层 tick 出错: {e}")


def stop() -> None:
    _stop.set()
    if runner is not None:
        runner.cancel_all("后端退出")
    if nav is not None and hasattr(nav, "shutdown"):
        nav.shutdown()


def cancel_all(reason: str) -> None:
    if runner is not None:
        runner.cancel_all(reason)


def set_estop(on: bool) -> None:
    """急停 / 解除，同时写进 state 文件"""
    runner.estop() if on else runner.release_estop()
    state.update(lambda d: d.__setitem__("estop", bool(on)))
    process_mgr.log("ui", "软件急停" if on else "解除急停")
