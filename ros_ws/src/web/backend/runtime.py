"""
后端进程里的单例：任务层（TaskRunner）+ 导航接口（RosLink）+ 事件推送（EventHub）+ LED/语音打招呼（Greeter、Voice）。
start() 在 FastAPI lifespan 里调；测试里用 install(FakeNav()) 代替，不起 ROS。
后端重启时正在跑的任务会中断（导航进程本来就归后端管）。急停状态写在 state 文件里，重启后保持。
"""
import os, threading
import process_mgr, state, tasklog
from config import GREET, ROS_DOMAIN_ID, TASKS, VOICE
from events import EventHub
from greeter import Greeter, GreeterConfig
from mission import Mission
from task_runner import RunnerConfig, TaskRunner
from voice import Voice, VoiceConfig

hub = EventHub()


def _emit(ev: dict) -> None:
    """事件同时推给 /api/v1/events 和任务日志"""
    hub.publish(ev)
    tasklog.on_event(ev)

runner: TaskRunner | None = None
greeter: Greeter | None = None
mission: Mission | None = None
voice = Voice(VoiceConfig.from_dict(VOICE), log=lambda s: process_mgr.log("ui", s))
nav = None
_stop = threading.Event()
_ticker_thread: threading.Thread | None = None
_estop_lock = threading.Lock()          # 急停 / 解除并发时，内存和 state 文件保持一致


class NullNav:
    """ROS 不可用时的占位：导航永远算没启动，任务都会被拒绝"""
    def nav_running(self): return False
    def mapping(self): return False
    def current_map_id(self): return None
    def loc_score(self): return None
    def pose(self): return None
    def nearest_person(self, require_face=False): return None
    def target_distance(self): return None
    def approach_start(self, distance, timeout): return False
    def approach_cancel(self): pass
    def approach_state(self): return "idle"
    def localizer_state(self): return None
    def led_text(self, text): pass
    def led_eyes(self): pass
    def person_info(self): return None
    def led_state(self): return None
    def send_goal(self, x, y, yaw, on_done): on_done(False, "ROS 不可用")
    def cancel_goal(self): pass
    def set_speed_limit(self, v): pass
    def set_estop(self, on): pass
    def shutdown(self): pass


class _Patrol:
    """greeter.PatrolPort：接到任务层的巡逻"""
    def active(self) -> bool:
        p = runner.current().get("patrol") if runner is not None else None
        return p is not None and p["state"] in ("running", "waiting")

    def pause(self) -> None:
        runner.pause_patrol("person")

    def resume(self) -> str:
        return runner.resume_patrol()


def install(nav_port, cfg: RunnerConfig | None = None, estopped: bool = False,
            greet_cfg: GreeterConfig | None = None) -> TaskRunner:
    global runner, nav, greeter
    nav = nav_port
    runner = TaskRunner(nav_port, cfg or RunnerConfig.from_dict(TASKS), emit=_emit, estopped=estopped)
    greeter = Greeter(nav_port, _Patrol(), greet_cfg or GreeterConfig.from_dict(GREET), emit=_emit,
                      say=voice.say)
    global mission
    mission = Mission(runner, greeter, nav_port.localizer_state, log=_mission_log)
    nav_port.set_estop(estopped)
    return runner


def _mission_log(kind: str, text: str) -> None:
    """状态机：状态变化记进任务日志；每 3s 的状态行另外存（网页「任务」页签单独一栏）并打到后端输出"""
    tasklog.add(kind, text)
    if kind == "tick":
        print(f"[状态机] {text}", flush=True)


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
    global _ticker_thread
    _ticker_thread = threading.Thread(target=_ticker, daemon=True)
    _ticker_thread.start()


def _ticker() -> None:
    while not _stop.wait(0.5):
        try:
            runner.tick()
        except Exception as e:
            process_mgr.log("ui", f"任务层 tick 出错: {e}")
        try:
            mission.tick()
        except Exception as e:
            process_mgr.log("ui", f"任务状态机 / 打招呼出错: {e}")



def stop() -> None:
    _stop.set()
    # 先 join ticker，避免正在跑的 tick 碰到已销毁的节点
    if _ticker_thread is not None:
        _ticker_thread.join(timeout=1.0)
    if runner is not None:
        runner.cancel_all("后端退出")
    if nav is not None and hasattr(nav, "shutdown"):
        nav.shutdown()


def cancel_all(reason: str) -> None:
    if runner is not None:
        runner.cancel_all(reason)


def ros_ok() -> bool:
    """任务层真的连着 ROS（不是 NullNav 占位）"""
    return nav is not None and not isinstance(nav, NullNav)


def set_estop(on: bool) -> None:
    """急停 / 解除，同时写进 state 文件"""
    with _estop_lock:
        runner.estop() if on else runner.release_estop()
        state.update(lambda d: d.__setitem__("estop", bool(on)))
    process_mgr.log("ui", "软件急停" if on else "解除急停")
