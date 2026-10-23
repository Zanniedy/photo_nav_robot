import pytest
from task_runner import RunnerConfig, TaskError, TaskRunner
from fakes import FakeClock, FakeNav, pt


def mk(**cfg):
    nav, clock, events = FakeNav(), FakeClock(), []
    return TaskRunner(nav, RunnerConfig(**cfg), emit=events.append, clock=clock), nav, clock, events


def route():
    return [pt("A", 1, 0), pt("B", 2, 0), pt("C", 3, 0)]


def test_patrol_loop():
    r, nav, *_ = mk()
    t = r.submit("patrol", route(), loop=True)
    for _ in range(3):
        nav.arrive()
    s = r.get(t["id"])
    assert s["state"] == "running" and s["laps"] == 1 and s["target"] == "A"


def test_patrol_once():
    r, nav, *_ = mk()
    t = r.submit("patrol", route(), loop=False)
    for _ in range(3):
        nav.arrive()
    assert r.get(t["id"])["state"] == "succeeded"


def test_patrol_skips_failed_point():
    r, nav, *_ = mk()
    t = r.submit("patrol", route(), loop=True)
    nav.fail("被挡住")
    s = r.get(t["id"])
    assert s["state"] == "running" and s["target"] == "B" and "跳过" in s["message"]
    assert s["skipped"] == ["A"]
    nav.arrive()                                   # 到了 B，连续失败计数清零
    nav.fail()
    nav.fail()
    assert r.get(t["id"])["state"] == "running"


def test_patrol_fails_when_whole_lap_fails():
    r, nav, *_ = mk()
    t = r.submit("patrol", route(), loop=True)
    for _ in range(3):
        nav.fail()
    assert r.get(t["id"])["state"] == "failed"


def test_manual_wait_resume():
    r, nav, _, ev = mk()
    t = r.submit("patrol", route(), loop=True)
    nav.arrive()                                   # 到了 A，正去 B
    cancels = nav.cancels
    s = r.wait(t["id"])
    assert s["state"] == "waiting" and s["wait_reason"] == "manual" and nav.cancels == cancels + 1
    assert r.wait(t["id"])["state"] == "waiting"   # 重复 wait 没副作用
    s = r.resume(t["id"])
    assert s["state"] == "running" and s["target"] == "B" and nav.goals[-1][:2] == (2, 0)
    types = [e["type"] for e in ev]
    assert "waiting" in types and "resumed" in types
    assert r.resume(t["id"])["state"] == "running"


def test_wait_only_for_patrol():
    r, *_ = mk()
    t = r.submit("goto", [pt("A", 1, 0)])
    with pytest.raises(TaskError) as e:
        r.wait(t["id"])
    assert e.value.code == "BAD_REQUEST"


def test_subtask_during_patrol():
    r, nav, *_ = mk()
    p = r.submit("patrol", route(), loop=True)
    g = r.submit("guide", [pt("X", 5, 5)])
    assert r.get(p["id"])["state"] == "waiting" and r.get(p["id"])["wait_reason"] == "subtask"
    assert g["parent"] == p["id"]
    assert r.current()["foreground"]["id"] == g["id"] and r.current()["patrol"]["id"] == p["id"]
    nav.arrive()
    assert r.get(g["id"])["state"] == "succeeded"
    assert r.get(p["id"])["state"] == "waiting"    # 子任务做完仍在等，等上层 resume
    s = r.resume(p["id"])
    assert s["state"] == "running" and s["target"] == "A"


def test_resume_blocked_while_subtask_runs():
    r, *_ = mk()
    p = r.submit("patrol", route())
    r.submit("goto", [pt("X", 5, 5)])
    with pytest.raises(TaskError) as e:
        r.resume(p["id"])
    assert e.value.code == "BUSY"


def test_cancel_subtask_keeps_patrol_and_cancel_patrol_ends_subtask():
    r, *_ = mk()
    p = r.submit("patrol", route())
    g = r.submit("goto", [pt("X", 5, 5)])
    r.cancel(g["id"])
    assert r.get(p["id"])["state"] == "waiting"
    g2 = r.submit("goto", [pt("Y", 6, 6)])
    r.cancel(p["id"])
    assert r.get(g2["id"])["state"] == "canceled" and r.get(p["id"])["state"] == "canceled"


def test_new_patrol_preempts_old():
    r, *_ = mk()
    p1 = r.submit("patrol", route())
    g = r.submit("goto", [pt("X", 5, 5)])
    p2 = r.submit("patrol", route())
    assert r.get(p1["id"])["state"] == "preempted" and r.get(g["id"])["state"] == "preempted"
    assert r.get(p2["id"])["state"] == "running"


def test_wait_timeout():
    r, nav, clock, ev = mk(wait_timeout=120)
    p = r.submit("patrol", route())
    r.wait(p["id"])
    clock.advance(119)
    r.tick()
    assert r.get(p["id"])["state"] == "waiting" and r.get(p["id"])["wait_left_s"] == 1.0
    clock.advance(2)
    r.tick()
    assert r.get(p["id"])["state"] == "running"
    assert "wait_timeout" in [e["type"] for e in ev]


def test_wait_timeout_zero_waits_forever():
    r, nav, clock, _ = mk(wait_timeout=0)
    p = r.submit("patrol", route())
    r.wait(p["id"])
    clock.advance(1e6)
    r.tick()
    assert r.get(p["id"])["state"] == "waiting" and r.get(p["id"])["wait_left_s"] is None


def test_timeout_counts_from_subtask_end():
    r, nav, clock, _ = mk(wait_timeout=120)
    p = r.submit("patrol", route())
    r.wait(p["id"])
    clock.advance(100)
    r.submit("goto", [pt("X", 5, 5)])
    clock.advance(100)
    r.tick()                                       # 子任务在跑，不超时
    nav.arrive()
    r.tick()
    assert r.get(p["id"])["state"] == "waiting"
    clock.advance(121)
    r.tick()
    assert r.get(p["id"])["state"] == "running"


def test_timeout_resume_blocked_keeps_waiting():
    r, nav, clock, _ = mk(wait_timeout=10)
    p = r.submit("patrol", route())
    r.wait(p["id"])
    nav.score = None
    r.cfg.min_score = 0.5                           # 定位不可靠时超时也不能走
    clock.advance(11)
    r.tick()
    s = r.get(p["id"])
    assert s["state"] == "waiting" and "不能继续" in s["message"]


def test_person_triggers_wait_with_cooldown():
    r, nav, clock, ev = mk(wait_on_person=True, person_distance=3, person_cooldown=30)
    p = r.submit("patrol", route())
    nav.person = 5.0
    r.tick()
    assert r.get(p["id"])["state"] == "running"
    nav.person = 2.0
    r.tick()
    s = r.get(p["id"])
    assert s["state"] == "waiting" and s["wait_reason"] == "person"
    found = [e for e in ev if e["type"] == "person_found"]
    assert len(found) == 1 and found[0]["distance"] == 2.0
    r.resume(p["id"])
    r.tick()
    assert r.get(p["id"])["state"] == "running"     # 冷却期内不再停
    clock.advance(31)
    r.tick()
    assert r.get(p["id"])["state"] == "waiting"


def test_person_ignored_when_disabled():
    r, nav, *_ = mk()
    p = r.submit("patrol", route())
    nav.person = 1.0
    r.tick()
    assert r.get(p["id"])["state"] == "running"


def test_estop_cancels_patrol_and_subtask():
    r, *_ = mk()
    p = r.submit("patrol", route())
    g = r.submit("goto", [pt("X", 5, 5)])
    r.estop()
    assert r.get(p["id"])["state"] == "canceled" and r.get(g["id"])["state"] == "canceled"
    assert r.current() == {"foreground": None, "patrol": None}


def test_map_switched_fails_patrol_and_subtask():
    r, nav, *_ = mk()
    p = r.submit("patrol", route(), loop=True)
    g = r.submit("guide", [pt("X", 5, 5)])
    nav.map_id = "lab"
    r.tick()
    assert r.get(p["id"])["state"] == "failed" and r.get(p["id"])["message"] == "地图已切换"
    assert r.get(g["id"])["state"] == "failed"
    assert r.current() == {"foreground": None, "patrol": None}
