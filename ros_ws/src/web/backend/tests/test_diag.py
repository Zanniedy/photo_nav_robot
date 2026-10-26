import process_mgr, runtime
from fakes import FakeNav
from routers import diag


def prepare(monkeypatch, tmp, running=("person", "led")):
    for f in ("m.bin", "m_outputs.json", "face.onnx"):
        open(f"{tmp}/{f}", "w").close()
    monkeypatch.setattr(diag, "_params", lambda path, node: {"model_path": f"{tmp}/m.bin", "face_model": f"{tmp}/face.onnx",
                                                             "address": "AA:BB"})
    monkeypatch.setattr(diag.glob, "glob", lambda pat: ["/sys/class/bluetooth/hci0"] if "bluetooth" in pat else [])
    monkeypatch.setattr(process_mgr, "is_running", lambda name: name in running)
    nav = FakeNav()
    monkeypatch.setattr(runtime, "nav", nav)
    return nav


def test_person_and_led_ok(clean, monkeypatch):
    nav = prepare(monkeypatch, clean)
    nav.person_st = {"face_model": True, "det": {"hz": 4.2, "ms": 150.0, "face_ms": 17.0}}
    nav.led_st = {"connected": True, "showing": "eyes", "error": ""}
    p, l = diag._person(), diag._led()
    assert p["ok"] and "4.2Hz" in p["reason"]
    assert l["ok"] and "AA:BB" in l["reason"] and "eyes" in l["reason"]


def test_person_and_led_failures(clean, monkeypatch):
    nav = prepare(monkeypatch, clean)
    assert "没收到" in diag._person()["reason"]
    nav.person_st = {"face_model": False, "det": {"hz": 4}}
    assert "人脸模型" in diag._person()["reason"]
    nav.person_st = {"face_model": True, "det": {"hz": 0.0}}
    assert "camera" in diag._person()["reason"]
    nav.led_st = {"connected": False, "error": "连接失败: x"}
    assert not diag._led()["ok"] and "连接失败" in diag._led()["reason"]
    prepare(monkeypatch, clean, running=())
    assert diag._person()["reason"] == "进程没在跑" and diag._led()["reason"] == "进程没在跑"
    import os
    os.remove(f"{clean}/face.onnx")
    assert "face.onnx" in diag._person()["reason"]
