import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
import auth, main, runtime, state
from routers import stack
from fakes import FakeNav, make_map
from task_runner import RunnerConfig


@pytest.fixture
def env(clean, monkeypatch):
    y = make_map("office")
    monkeypatch.setattr(stack, "current_map", lambda: y)
    monkeypatch.setattr(stack, "_mode", lambda: "navigation")
    nav = FakeNav()
    runtime.install(nav, RunnerConfig())
    c = TestClient(main.app)
    assert c.post("/api/auth/login", json={"username": auth.AUTH_USER,
                                           "password": auth.AUTH_PASSWORD}).status_code == 200
    return c, nav


def add(c, name, x, y):
    assert c.post("/api/v1/points", json={"name": name, "x": x, "y": y}).json()["ok"]


def test_status(env):
    c, _ = env
    d = c.get("/api/v1/status").json()["data"]
    assert d["mode"] == "navigation" and d["map"]["map_id"] == "office" and len(d["map"]["version"]) == 12
    assert d["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert d["localization"] == {"score": 0.9, "reliable": None}      # 没设门限
    assert d["estop"] is False and d["tasks"] == {"foreground": None, "patrol": None}


def test_maps(env):
    c, _ = env
    ms = c.get("/api/v1/maps").json()["data"]
    assert [(m["map_id"], m["current"], m["duplicate"]) for m in ms] == [("office", True, False)]


def test_points_crud(env):
    c, nav = env
    nav.robot_pose = (1.5, 2.0, 0.5)
    p = c.post("/api/v1/points", json={"name": "门口", "from_current_pose": True}).json()["data"]
    assert (p["x"], p["y"], p["yaw"]) == (1.5, 2.0, 0.5)
    add(c, "Lab", 3, 4)
    assert [q["name"] for q in c.get("/api/v1/points", params={"q": "门"}).json()["data"]["points"]] == ["门口"]
    assert c.patch("/api/v1/points/lab", json={"note": "实验室"}).json()["data"]["note"] == "实验室"
    assert c.post("/api/v1/points", json={"name": "门口", "x": 0, "y": 0}).json()["code"] == "BAD_REQUEST"
    assert c.post("/api/v1/points", json={"name": "X"}).json()["code"] == "BAD_REQUEST"      # 没坐标
    assert c.delete("/api/v1/points/门口").json()["ok"] is True
    assert c.delete("/api/v1/points/门口").json()["code"] == "NOT_FOUND"
    assert c.get("/api/v1/points", params={"map_id": "nope"}).json()["code"] == "NOT_FOUND"


def test_from_current_pose_needs_pose(env):
    c, nav = env
    nav.robot_pose = None
    assert c.post("/api/v1/points", json={"name": "A", "from_current_pose": True}).json()["code"] == "BAD_REQUEST"


def test_ambiguous_point(env):
    c, _ = env
    add(c, "lab", 1, 0)
    add(c, "LAB", 2, 0)
    r = c.patch("/api/v1/points/Lab", json={"note": "x"})
    assert r.status_code == 409 and r.json()["code"] == "AMBIGUOUS"


def test_routes(env):
    c, _ = env
    add(c, "A", 1, 0)
    add(c, "B", 2, 0)
    r = c.post("/api/v1/routes", json={"name": "日常", "points": ["A", "B"]}).json()["data"]
    assert r["point_names"] == ["A", "B"] and r["loop"] is True
    assert c.put(f"/api/v1/routes/{r['id']}", json={"loop": False}).json()["data"]["loop"] is False
    assert c.get("/api/v1/routes").json()["data"]["routes"][0]["name"] == "日常"
    assert c.delete("/api/v1/routes/日常").json()["ok"] is True
    assert c.post("/api/v1/routes", json={"name": "x", "points": ["Z"]}).json()["code"] == "BAD_REQUEST"
    assert c.put("/api/v1/routes/nope", json={"loop": True}).json()["code"] == "NOT_FOUND"


def test_goto_task_flow(env):
    c, nav = env
    add(c, "A", 1, 0)
    t = c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]}).json()["data"]
    assert t["state"] == "running" and nav.goals[-1][:2] == (1, 0)
    assert c.get("/api/v1/tasks/current").json()["data"]["foreground"]["id"] == t["id"]
    nav.arrive()
    assert c.get(f"/api/v1/tasks/{t['id']}").json()["data"]["state"] == "succeeded"
    assert c.get("/api/v1/tasks/nope").json()["code"] == "NOT_FOUND"


def test_guide_endpoint(env):
    c, nav = env
    add(c, "A", 1, 0)
    t = c.post("/api/v1/guide", json={"point": "A"}).json()["data"]
    assert t["type"] == "guide" and t["max_speed"] == 0.15 and nav.speed == 0.15
    r = c.post("/api/v1/guide", json={"point": "A", "max_speed": 2})
    assert r.status_code == 400 and r.json()["code"] == "BAD_REQUEST"


def test_task_errors(env):
    c, nav = env
    add(c, "A", 1, 0)
    assert c.post("/api/v1/tasks", json={"type": "goto", "points": ["Z"]}).json()["code"] == "NOT_FOUND"
    r = c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"], "map_id": "lab"})
    assert r.json()["code"] == "MAP_MISMATCH"
    assert c.post("/api/v1/tasks", json={"type": "patrol"}).json()["code"] == "BAD_REQUEST"
    nav.map_id = "lab"                             # 导航地图和标注所在地图对不上（切换中）
    assert c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]}).json()["code"] == "MAP_MISMATCH"
    nav.map_id = "office"
    nav.running = False
    r = c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]})
    assert r.status_code == 409 and r.json()["code"] == "NAV_NOT_RUNNING"


def test_patrol_wait_guide_resume(env):
    c, nav = env
    add(c, "A", 1, 0)
    add(c, "B", 2, 0)
    c.post("/api/v1/routes", json={"name": "r", "points": ["A", "B"]})
    p = c.post("/api/v1/tasks", json={"type": "patrol", "route": "r"}).json()["data"]
    assert p["loop"] is True and p["route_id"]
    assert c.post(f"/api/v1/tasks/{p['id']}/wait").json()["data"]["state"] == "waiting"
    g = c.post("/api/v1/guide", json={"point": "B"}).json()["data"]
    assert g["parent"] == p["id"]
    nav.arrive()
    assert c.post(f"/api/v1/tasks/{p['id']}/resume").json()["data"]["state"] == "running"
    assert c.post(f"/api/v1/tasks/{p['id']}/cancel").json()["data"]["state"] == "canceled"


def test_estop_endpoints(env):
    c, nav = env
    add(c, "A", 1, 0)
    t = c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]}).json()["data"]
    assert c.post("/api/v1/estop").json()["data"] == {"estop": True}
    assert nav.estop is True and state.load()["estop"] is True
    assert c.get(f"/api/v1/tasks/{t['id']}").json()["data"]["state"] == "canceled"
    assert c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]}).json()["code"] == "ESTOPPED"
    assert c.post("/api/v1/estop/release").json()["data"] == {"estop": False}
    assert nav.estop is False and state.load()["estop"] is False


def test_token_caller_can_send_tasks(env):
    c, _ = env
    add(c, "A", 1, 0)
    tok = c.post("/api/v1/robot/tokens", json={"name": "voice"}).json()["data"]["token"]
    r = TestClient(main.app).post("/api/v1/guide", json={"point": "A"},
                                  headers={"Authorization": f"Bearer {tok}"})
    assert r.status_code == 200 and r.json()["data"]["type"] == "guide"


def test_events_ws(env):
    c, _ = env
    add(c, "A", 1, 0)
    with c.websocket_connect("/api/v1/events") as ws:
        first = ws.receive_json()
        assert first["type"] == "status" and first["robot_id"].startswith("unset-")
        c.post("/api/v1/tasks", json={"type": "goto", "points": ["A"]})
        for _ in range(5):
            ev = ws.receive_json()
            if ev["type"] == "task_state":
                break
        assert ev["task"]["state"] == "running" and ev["robot_id"] == first["robot_id"]


def test_events_ws_requires_auth(env):
    with pytest.raises(WebSocketDisconnect):
        with TestClient(main.app).websocket_connect("/api/v1/events") as ws:
            ws.receive_json()
