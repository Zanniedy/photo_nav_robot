import numpy as np
import pytest

from person_locator.tracker import Tracker


def test_camera_creates_lidar_only_updates():
    tk = Tracker()
    tk.update_lidar(0.0, [np.array([1.0, 0.0])])
    assert tk.tracks == []                       # 激光不建轨迹
    tk.update_camera(0.0, [np.array([2.0, 0.0])])
    assert len(tk.tracks) == 1
    tk.update_lidar(0.1, [np.array([2.05, 0.0])])
    assert tk.tracks[0].xy == pytest.approx([2.05, 0.0], abs=0.03)


def test_follow_walking_person_with_lidar_only():
    tk = Tracker()
    tk.update_camera(0.0, [np.array([2.0, 0.0])])
    for k in range(1, 40):                       # 1m/s 往前走 4s，相机不再看到（远了）
        tk.update_lidar(k * 0.1, [np.array([2.0 + k * 0.1, 0.0])])
        tk.prune(k * 0.1)
    assert len(tk.tracks) == 1
    tr = tk.tracks[0]
    assert tr.xy == pytest.approx([5.9, 0.0], abs=0.1)
    assert tr.x[2] == pytest.approx(1.0, abs=0.2)


def test_lost_after_timeout():
    tk = Tracker(lost_timeout=2.0)
    tk.update_camera(0.0, [np.array([2.0, 0.0])])
    tk.prune(1.5)
    assert len(tk.tracks) == 1
    tk.prune(2.5)
    assert tk.tracks == []


def test_camera_veto_when_in_view_but_not_seen():
    tk = Tracker(cam_confirm_timeout=3.0)
    tk.update_camera(0.0, [np.array([2.0, 0.0])])
    for k in range(1, 40):                       # 激光一直有个簇（人走了，留下柱子）
        tk.update_lidar(k * 0.1, [np.array([2.0, 0.0])])
        tk.prune(k * 0.1, in_camera_view=lambda xy: True)
    assert tk.tracks == []                       # 相机就在看着却 3s 没检测到 → 丢弃


def test_lock_nearest_then_keep_then_reacquire():
    tk = Tracker()
    tk.update_camera(0.0, [np.array([3.0, 0.0]), np.array([1.5, 1.0])])
    tr, changed = tk.target((0, 0), 0.0)
    assert changed and tr.xy == pytest.approx([1.5, 1.0])
    first = tr.id
    # 另一个人走得更近，也不换目标
    tk.update_camera(0.5, [np.array([1.0, 0.0]), np.array([1.5, 1.0])])
    tr, changed = tk.target((0, 0), 0.5)
    assert tr.id == first and not changed
    # 目标消失后，重新锁当前最近的
    tk.tracks = [t for t in tk.tracks if t.id != first]
    tk.prune(0.6)
    tr, changed = tk.target((0, 0), 0.6)
    assert changed and tr.id != first


def test_nearest_requires_recent_camera():
    tk = Tracker()
    tk.update_camera(0.0, [np.array([2.0, 0.0])])
    tk.update_lidar(1.5, [np.array([2.0, 0.0])])
    assert tk.nearest((0, 0), 1.5, cam_within=1.0) is None


def test_two_people_not_swapped():
    tk = Tracker()
    tk.update_camera(0.0, [np.array([2.0, -0.5]), np.array([2.0, 0.5])])
    ids = {round(t.xy[1], 1): t.id for t in tk.tracks}
    for k in range(1, 10):
        t = k * 0.1
        tk.update_lidar(t, [np.array([2.0 + 0.05 * k, -0.5]), np.array([2.0 - 0.05 * k, 0.5])])
    by = {t.id: t.xy for t in tk.tracks}
    assert by[ids[-0.5]][1] == pytest.approx(-0.5, abs=0.05)
    assert by[ids[0.5]][1] == pytest.approx(0.5, abs=0.05)
