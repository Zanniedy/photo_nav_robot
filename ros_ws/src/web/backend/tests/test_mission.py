from fakes import FakeClock, FakeNav, pt
from greeter import Greeter, GreeterConfig
from mission import Mission
from task_runner import RunnerConfig, TaskRunner


class Patrol:
    def __init__(self, r): self.r = r
    def active(self):
        p = self.r.current()["patrol"]
        return p is not None and p["state"] in ("running", "waiting")
    def pause(self): self.r.pause_patrol("person")
    def resume(self): return self.r.resume_patrol()


def mk():
    nav, clock, logs = FakeNav(), FakeClock(), []
    r = TaskRunner(nav, RunnerConfig(wait_timeout=5), clock=clock)
    g = Greeter(nav, Patrol(r), GreeterConfig(), clock=clock)
    m = Mission(r, g, nav.localizer_state, log=lambda k, t: logs.append((k, t)), clock=clock)

    def step(dt=0.5):
        clock.advance(dt)
        r.tick()
        m.tick()
    return r, g, m, nav, clock, logs, step


def route():
    return [pt("A", 1, 0), pt("B", 2, 0)]


def test_lost_pauses_then_recovery_resumes_patrol():
    r, g, m, nav, clock, logs, step = mk()
    t = r.submit("patrol", route(), loop=True)
    step()
    assert m.state == "巡逻"
    goals = len(nav.goals)
    nav.loc = {"state": "lost", "score": 0.2}
    step()
    assert m.state == "丢失" and r.get(t["id"])["state"] == "waiting" and nav.cancels >= 1
    nav.loc = {"state": "recovering", "score": 0.2, "recover": {"radius": 2.0, "score": 0.4}}
    for _ in range(20):                                  # 恢复期间超过 wait_timeout 也不能自己走
        step()
    assert m.state == "恢复" and r.get(t["id"])["state"] == "waiting"
    nav.loc = {"state": "tracking", "score": 0.6}
    step()
    assert m.state == "巡逻" and r.get(t["id"])["state"] == "running" and len(nav.goals) == goals + 1
    texts = [x for k, x in logs if k == "mission"]
    assert "状态机 巡逻 → 丢失" in texts and "状态机 恢复 → 巡逻" in texts


def test_face_then_wait_then_patrol_and_no_greet_while_lost():
    r, g, m, nav, clock, logs, step = mk()
    r.submit("patrol", route(), loop=True)
    step()
    nav.person, nav.target = 4.0, 3.0
    step()
    assert m.state == "人脸"
    nav.target = 1.8
    step()
    assert m.state == "等待" and nav.led[-1] == ("text", "hello i am max")
    for _ in range(17):
        step()
    assert m.state == "巡逻" and nav.led[-1] == ("eyes",)
    # 丢失中看到人脸不触发
    nav.loc = {"state": "lost", "score": 0.2}
    clock.advance(30)
    step()
    nav.person = 3.0
    step()
    assert m.state == "丢失" and g.state == "idle"


def test_lost_during_search_cancels_greeter():
    r, g, m, nav, clock, logs, step = mk()
    r.submit("patrol", route(), loop=True)
    step()
    nav.person, nav.target = 4.0, 3.0
    step()
    assert g.state == "search"
    nav.loc = {"state": "lost", "score": 0.1}
    step()
    assert g.state == "idle" and m.state == "丢失"
    p = r.current()["patrol"]
    assert p["state"] == "waiting"


def test_report_every_3s():
    r, g, m, nav, clock, logs, step = mk()
    r.submit("patrol", route(), loop=True)
    for _ in range(13):                                  # 6.5s
        step()
    ticks = [x for k, x in logs if k == "tick"]
    assert len(ticks) == 3
    assert ticks[-1].startswith("[巡逻 ") and "巡逻 running 1/2 → A" in ticks[-1] and "定位 tracking 0.55" in ticks[-1]
