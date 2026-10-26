from greeter import Greeter, GreeterConfig
from fakes import FakeNav


class Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t


class Patrol:
    def __init__(self): self.on, self.paused, self.resumed, self.err = True, 0, 0, ""
    def active(self): return self.on
    def pause(self): self.paused += 1
    def resume(self): self.resumed += 1; return self.err


def make(**kw):
    nav, pat, clk, evs = FakeNav(), Patrol(), Clock(), []
    g = Greeter(nav, pat, GreeterConfig(**kw), emit=evs.append, clock=clk, say=nav.led.append)
    return g, nav, pat, clk, evs


def test_far_face_searches_then_greets_by_tracked_distance():
    g, nav, pat, clk, evs = make()
    nav.person = 4.0
    g.tick()
    assert g.state == "search" and pat.paused == 1 and nav.approaches == [(1.5, 40.0)]
    assert nav.led == []                                   # search 阶段屏不变
    nav.person = None                                      # 近了以后人脸拍不全
    nav.target = 2.5
    clk.t += 2; g.tick()
    assert g.state == "search"
    nav.target = 1.9
    clk.t += 2; g.tick()
    assert g.state == "greet" and nav.led == [("text", "hello i am max"), "greet"]   # 屏 + 语音
    clk.t += 8; g.tick()
    assert g.state == "idle" and nav.led[-1] == ("eyes",) and pat.resumed == 1 and nav.ap_cancels == 1
    assert [e["type"] for e in evs] == ["greet_search", "greet_greet", "greet_idle"]


def test_face_beyond_detect_distance_ignored_and_cooldown():
    g, nav, pat, clk, _ = make(cooldown=20)
    nav.person = 6.0
    g.tick()
    assert g.state == "idle" and not nav.approaches
    nav.person = 1.5                                       # 已经很近：直接打招呼，不走
    g.tick()
    assert g.state == "greet" and not nav.approaches
    clk.t += 8; g.tick()
    assert g.state == "idle"
    clk.t += 5; g.tick()
    assert g.state == "idle"                               # 冷却中
    clk.t += 16; g.tick()
    assert g.state == "greet"


def test_search_gives_up_when_lost_or_approach_fails():
    g, nav, pat, clk, evs = make()
    nav.person, nav.target = 4.0, 3.5
    g.tick()
    clk.t += 1; g.tick()                                   # 锁定了
    nav.person, nav.target = None, None
    clk.t += 4; g.tick()
    assert g.state == "idle" and "丢" in g.message and pat.resumed == 1 and nav.led == []
    g, nav, pat, clk, _ = make()
    nav.person, nav.target = 4.0, 3.0
    g.tick()
    nav.ap_state = "failed"
    clk.t += 1; g.tick()
    assert g.state == "idle" and pat.resumed == 1


def test_only_patrol_and_patrol_canceled():
    g, nav, pat, clk, _ = make()
    pat.on = False
    nav.person = 3.0
    g.tick()
    assert g.state == "idle"
    pat.on = True
    g.tick()
    assert g.state == "search"
    pat.on = False                                         # 用户取消巡逻
    clk.t += 1; g.tick()
    assert g.state == "idle" and pat.resumed == 0 and nav.ap_cancels == 1


def test_from_dict_types():
    c = GreeterConfig.from_dict({"enabled": 0, "text": 123, "greet_distance": "1.8", "bogus": 1})
    assert c.enabled is False and c.text == "123" and c.greet_distance == 1.8


def test_tasklog_format():
    import tasklog
    assert tasklog.format_event({"type": "progress", "task": {}}) is None
    k, s = tasklog.format_event({"type": "waiting", "reason": "person",
                                 "task": {"type": "patrol", "id": "t1", "points": ["a", "b"], "index": 1,
                                          "state": "waiting", "target": "b"}})
    assert k == "task" and s == "巡逻 t1 2/2 → b 暂停（遇到人）"
    assert tasklog.format_event({"type": "greet_search", "message": "看到人脸 4.0m"}) == ("greet", "[搜索] 看到人脸 4.0m")


def test_voice_off():
    g, nav, pat, clk, _ = make(voice="")
    nav.person = 1.5
    g.tick()
    assert g.state == "greet" and nav.led == [("text", "hello i am max")]


def test_waits_for_lock_before_counting_lost():
    """10-26 实测：person_tracker 还在 searching（没锁定，target 为 None）时 3s 就被当成「目标丢了」取消，巡逻恢复开走"""
    g, nav, pat, clk, _ = make()
    nav.person = 2.3
    g.tick()
    assert g.state == "search"
    nav.person = None
    for _ in range(8):                                     # 4s 还没锁定：继续等 approach 自己的 search_timeout
        clk.t += 0.5; g.tick()
    assert g.state == "search" and nav.ap_cancels == 0
    nav.target = 1.8                                       # 锁定了，已经在 2m 内
    clk.t += 0.5; g.tick()
    assert g.state == "greet"


def test_lock_timeout_and_approach_search_fail():
    g, nav, pat, clk, _ = make(lock_timeout=8)
    nav.person = 3.0
    g.tick()
    nav.person = None
    clk.t += 8.5; g.tick()
    assert g.state == "idle" and "锁定" in g.message and pat.resumed == 1
    g, nav, pat, clk, _ = make()
    nav.person = 3.0
    g.tick()
    nav.ap_state = "failed"                                # person_tracker 5s 内没看到人脸
    clk.t += 5; g.tick()
    assert g.state == "idle" and "failed" in g.message
