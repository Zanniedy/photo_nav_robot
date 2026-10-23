import pytest
from task_runner import RunnerConfig, TaskError, TaskRunner
from fakes import FakeClock, FakeNav, pt


def mk(**cfg):
    nav, clock, events = FakeNav(), FakeClock(), []
    return TaskRunner(nav, RunnerConfig(**cfg), emit=events.append, clock=clock), nav, clock, events


def test_goto_multi_points():
    r, nav, _, ev = mk()
    t = r.submit("goto", [pt("A", 1, 0), pt("B", 2, 0)])
    assert t["state"] == "running" and t["target"] == "A" and nav.goals[-1][:2] == (1, 0)
    nav.arrive()
    assert r.get(t["id"])["target"] == "B" and nav.goals[-1][:2] == (2, 0)
    nav.arrive()
    assert r.get(t["id"])["state"] == "succeeded"
    assert r.current() == {"foreground": None, "patrol": None}
    assert [e["task"]["state"] for e in ev if e["type"] == "task_state"] == ["running", "succeeded"]
    assert [e["type"] for e in ev].count("progress") == 1
    assert nav.speed is None


def test_goto_failure():
    r, nav, *_ = mk()
    t = r.submit("goto", [pt("A", 1, 0), pt("B", 2, 0)])
    nav.fail("nav2 拒绝了目标")
    s = r.get(t["id"])
    assert s["state"] == "failed" and "nav2 拒绝了目标" in s["message"]


def test_guide_speed_limit():
    r, nav, *_ = mk()
    t = r.submit("guide", [pt("A", 1, 0)])
    assert nav.speed == 0.15 and t["max_speed"] == 0.15
    r.submit("guide", [pt("A", 1, 0)], max_speed=0.1)
    assert nav.speed == 0.1
    nav.arrive()
    assert nav.speed is None
    with pytest.raises(TaskError) as e:
        r.submit("guide", [pt("A"), pt("B")])
    assert e.value.code == "BAD_REQUEST"


def test_bad_requests():
    r, *_ = mk()
    for args in (("fly", [pt("A")]), ("goto", [])):
        with pytest.raises(TaskError) as e:
            r.submit(*args)
        assert e.value.code == "BAD_REQUEST"


def test_preempt_and_stale_callback():
    r, nav, *_ = mk()
    t1 = r.submit("goto", [pt("A", 1, 0)])
    old = nav.callback
    t2 = r.submit("goto", [pt("B", 2, 0)])
    assert r.get(t1["id"])["state"] == "preempted"
    old(True, "到达")                                   # 旧目标的结果晚到，要丢掉
    s = r.get(t2["id"])
    assert s["state"] == "running" and s["target"] == "B"


def test_cancel():
    r, nav, *_ = mk()
    t = r.submit("goto", [pt("A", 1, 0)])
    assert r.cancel(t["id"])["state"] == "canceled"
    assert nav.cancels >= 1
    with pytest.raises(TaskError) as e:
        r.cancel(t["id"])
    assert e.value.code == "BAD_REQUEST"
    with pytest.raises(TaskError) as e:
        r.cancel("nope")
    assert e.value.code == "NOT_FOUND"


@pytest.mark.parametrize("attr,value,code", [
    ("running", False, "NAV_NOT_RUNNING"),
    ("is_mapping", True, "BUSY"),
])
def test_nav_state_checks(attr, value, code):
    r, nav, *_ = mk()
    setattr(nav, attr, value)
    with pytest.raises(TaskError) as e:
        r.submit("goto", [pt("A")])
    assert e.value.code == code


def test_map_mismatch():
    r, *_ = mk()
    with pytest.raises(TaskError) as e:
        r.submit("goto", [pt("A")], map_id="lab")
    assert e.value.code == "MAP_MISMATCH"
    assert r.submit("goto", [pt("A")], map_id="office")["map_id"] == "office"


def test_min_score():
    r, nav, *_ = mk(min_score=0.5)
    for s in (0.3, None):
        nav.score = s
        with pytest.raises(TaskError) as e:
            r.submit("goto", [pt("A")])
        assert e.value.code == "LOCALIZATION_UNRELIABLE"
    nav.score = 0.6
    r.submit("goto", [pt("A")])


def test_estop():
    r, nav, _, ev = mk()
    t = r.submit("goto", [pt("A", 1, 0)])
    r.estop()
    assert r.get(t["id"])["state"] == "canceled" and nav.estop is True and r.estopped
    assert {"type": "estop", "on": True} in ev
    with pytest.raises(TaskError) as e:
        r.submit("goto", [pt("A")])
    assert e.value.code == "ESTOPPED"
    r.release_estop()
    assert nav.estop is False
    r.submit("goto", [pt("A")])


def test_estopped_at_start():
    r = TaskRunner(FakeNav(), estopped=True)
    with pytest.raises(TaskError) as e:
        r.submit("goto", [pt("A")])
    assert e.value.code == "ESTOPPED"


def test_progress_remaining():
    r, nav, *_ = mk()
    t = r.submit("goto", [pt("A", 3, 0), pt("B", 3, 4)])
    assert t["remaining_m"] == 7.0 and t["eta_s"] == 28.0       # 默认 0.25 m/s
    nav.robot_pose = None
    assert r.get(t["id"])["remaining_m"] is None


def test_nav_stopped_fails_tasks():
    r, nav, *_ = mk()
    t = r.submit("goto", [pt("A", 1, 0)])
    nav.running = False
    r.tick()
    assert r.get(t["id"])["state"] == "failed"


def test_cancel_all():
    r, *_ = mk()
    t = r.submit("goto", [pt("A", 1, 0)])
    r.cancel_all("切换到建图")
    s = r.get(t["id"])
    assert s["state"] == "canceled" and s["message"] == "切换到建图"


def test_history_limit():
    r, nav, *_ = mk()
    first = r.submit("goto", [pt("A", 1, 0)])["id"]
    nav.arrive()
    for _ in range(60):
        r.submit("goto", [pt("A", 1, 0)])
        nav.arrive()
    assert r.get(first) is None


def test_config_from_dict():
    c = RunnerConfig.from_dict({"guide_speed": "0.2", "wait_on_person": 1, "unknown": 5})
    assert c.guide_speed == 0.2 and c.wait_on_person is True and c.wait_timeout == 120.0
