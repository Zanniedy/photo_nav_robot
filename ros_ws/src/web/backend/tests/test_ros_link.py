import pytest

pytest.importorskip("rclpy")
from ros_link import RosLink, person_distance


def test_person_distance():
    st = {"frame": "map", "tracks": [{"id": 1, "x": 3.0, "y": 4.0}, {"id": 2, "x": 1.0, "y": 0.0}]}
    assert person_distance(st, (0.0, 0.0, 0.0)) == 1.0
    assert person_distance({"frame": "map", "tracks": []}, (0, 0, 0)) is None
    assert person_distance(st, None) is None
    assert person_distance({**st, "frame": "odom"}, (0, 0, 0)) is None


def test_smoke_without_nav2():
    link = RosLink(nav_running=lambda: True, mapping=lambda: False, current_map_id=lambda: "office")
    try:
        assert link.current_map_id() == "office" and link.nav_running() and not link.mapping()
        assert link.pose() is None and link.loc_score() is None and link.nearest_person() is None
        got = []
        link.send_goal(1.0, 0.0, 0.0, lambda ok, msg: got.append((ok, msg)))
        assert got == [(False, "nav2 action server 不在线")]
        link.set_speed_limit(0.15)
        link.set_speed_limit(None)
        link.set_estop(False)
        link.cancel_goal()
    finally:
        link.shutdown()
