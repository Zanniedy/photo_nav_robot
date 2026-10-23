import pytest
from fastapi.testclient import TestClient
import auth, main


@pytest.fixture
def client(clean):
    c = TestClient(main.app)          # 不用 with：不跑 lifespan，不起 ROS 进程
    r = c.post("/api/auth/login", json={"username": auth.AUTH_USER, "password": auth.AUTH_PASSWORD})
    assert r.status_code == 200
    return c


def test_requires_auth(clean):
    r = TestClient(main.app).get("/api/v1/robot")
    assert r.status_code == 401
    body = r.json()
    assert body["ok"] is False and body["code"] == "UNAUTHORIZED" and "robot_id" in body


def test_get_and_configure(client):
    r = client.get("/api/v1/robot").json()
    assert r["ok"] is True and r["data"]["configured"] is False
    r = client.put("/api/v1/robot", json={"robot_id": "max-001", "name": "一号机"}).json()
    assert r["data"]["robot_id"] == "max-001" and r["robot_id"] == "max-001"
    r = client.put("/api/v1/robot", json={"robot_id": "max-002"})
    assert r.status_code == 409 and r.json()["code"] == "ROBOT_NOT_CONFIGURED"


def test_validation_error_envelope(client):
    r = client.put("/api/v1/robot", json={"robot_id": ["x"]})
    assert r.status_code == 400
    assert r.json()["ok"] is False and r.json()["code"] == "BAD_REQUEST"


def test_token_flow(client):
    tok = client.post("/api/v1/robot/tokens", json={"name": "tablet"}).json()["data"]["token"]
    bare = TestClient(main.app)
    h = {"Authorization": f"Bearer {tok}"}
    assert bare.get("/api/v1/robot", headers=h).status_code == 200
    # token 调用方不能再生成 token
    assert bare.post("/api/v1/robot/tokens", json={"name": "x"}, headers=h).status_code == 401
    assert bare.get("/api/v1/robot", headers={"Authorization": "Bearer nope"}).status_code == 401
    assert client.delete("/api/v1/robot/tokens/tablet").status_code == 200
    assert bare.get("/api/v1/robot", headers=h).status_code == 401
    assert client.delete("/api/v1/robot/tokens/tablet").json()["code"] == "NOT_FOUND"


def test_legacy_api_unchanged(clean):
    r = TestClient(main.app).get("/api/status")
    assert r.status_code == 401 and r.json() == {"detail": "未登录"}
