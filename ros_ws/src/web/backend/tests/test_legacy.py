import pytest
from fastapi.testclient import TestClient
import auth, main, process_mgr, runtime
from routers import stack
from fakes import FakeNav, make_map
from task_runner import RunnerConfig


@pytest.fixture
def env(clean, monkeypatch):
    y = make_map("office")
    monkeypatch.setattr(stack, "current_map", lambda: y)
    monkeypatch.setattr(process_mgr, "is_running", lambda n: n in ("nav", "base"))
    monkeypatch.setattr(process_mgr, "start", lambda name, cmd: None)
    monkeypatch.setattr(process_mgr, "stop", lambda name: None)
    monkeypatch.setattr(process_mgr, "run_once", lambda args, timeout=10.0: {"rc": 0, "output": "", "timed_out": False})
    nav = FakeNav()
    runtime.install(nav, RunnerConfig())
    c = TestClient(main.app)
    c.post("/api/auth/login", json={"username": auth.AUTH_USER, "password": auth.AUTH_PASSWORD})
    assert c.post("/api/points", json={"map": "office", "name": "A", "x": 1, "y": 0}).status_code == 200
    return c, nav


def test_points_goto_uses_runner(env):
    c, nav = env
    r = c.post("/api/points_goto", json={"names": ["A"]})
    assert r.status_code == 200
    body = r.json()
    assert body["ok"] is True and body["names"] == ["A"] and body["task_id"]
    assert nav.goals[-1][:2] == (1, 0)
    assert runtime.runner.current()["foreground"]["points"] == ["A"]


def test_points_goto_estopped(env):
    c, _ = env
    runtime.set_estop(True)
    r = c.post("/api/points_goto", json={"names": ["A"]})
    assert r.status_code == 409 and "急停" in r.json()["detail"]
    runtime.set_estop(False)


def test_waypoints_uses_runner(env):
    c, nav = env
    r = c.post("/api/nav/waypoints", json={"poses": [{"x": 1, "y": 1}, {"x": 2, "y": 2, "yaw": 1.0}]})
    assert r.status_code == 200 and r.json()["task_id"]
    assert runtime.runner.current()["foreground"]["points"] == ["航点1", "航点2"]
    assert nav.goals[-1] == (1.0, 1.0, 0.0)


def test_nav_cancel_cancels_task(env):
    c, _ = env
    t = c.post("/api/points_goto", json={"names": ["A"]}).json()["task_id"]
    assert c.post("/api/nav/cancel").status_code == 200
    assert runtime.runner.get(t)["state"] == "canceled"


def test_mapping_start_cancels_task(env):
    c, _ = env
    t = c.post("/api/points_goto", json={"names": ["A"]}).json()["task_id"]
    assert c.post("/api/mapping/start").status_code == 200
    s = runtime.runner.get(t)
    assert s["state"] == "canceled" and s["message"] == "切换到建图"


def test_switch_to_missing_map_keeps_task(env):
    c, _ = env
    t = c.post("/api/points_goto", json={"names": ["A"]}).json()["task_id"]
    assert c.post("/api/nav/switch_map", json={"map": "nope"}).status_code == 404
    assert runtime.runner.get(t)["state"] == "running"
