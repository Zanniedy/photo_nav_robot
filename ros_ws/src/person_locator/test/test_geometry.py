import math

import numpy as np
import pytest

from person_locator.geometry import (approach_goal, cluster_scan, measure_in_window, pixel_rays, quat_to_rot,
                                     ray_bearings, scan_points, wrap)

# camera_link → camera_optical_frame 的旋转（robot.launch.py 里 rpy=-90,0,-90）：光学 z 朝前、x 朝右、y 朝下
R_LINK_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
K = [1000, 0, 640, 0, 1000, 360, 0, 0, 1]


def test_wrap():
    assert abs(wrap(3 * math.pi)) == pytest.approx(math.pi)
    assert wrap(-0.5) == pytest.approx(-0.5)


def test_quat_to_rot_yaw():
    yaw = 0.7
    R = quat_to_rot(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert R @ np.array([1, 0, 0]) == pytest.approx([math.cos(yaw), math.sin(yaw), 0])


def test_pixel_bearing_center_and_right():
    rays = pixel_rays([[640, 360], [1640, 360], [640, 700]], K, [0, 0, 0, 0, 0])
    b = ray_bearings(rays, R_LINK_OPT)
    assert b[0] == pytest.approx(0, abs=1e-9)              # 主点 = 正前方
    assert b[1] == pytest.approx(-math.pi / 4)             # 右边 fx 个像素 = 右 45°（车体系 y 朝左，所以为负）
    assert b[2] == pytest.approx(0, abs=1e-9)              # 竖直方向不影响方位


def test_pixel_bearing_with_robot_yaw():
    yaw = math.pi / 2
    R = quat_to_rot(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) @ R_LINK_OPT
    b = ray_bearings(pixel_rays([[640, 360]], K, None), R)
    assert b[0] == pytest.approx(math.pi / 2)


def test_undistort_matches_cv2_projection():
    cv2 = pytest.importorskip('cv2')
    D = np.array([-0.3, 0.1, 0, 0, 0])
    Km = np.array(K, float).reshape(3, 3)
    p3 = np.array([[0.4, 0.1, 1.0]])
    uv, _ = cv2.projectPoints(p3, np.zeros(3), np.zeros(3), Km, D)
    ray = pixel_rays(uv.reshape(-1, 2), K, D)[0]
    assert ray[:2] == pytest.approx([0.4, 0.1], abs=1e-3)


def _ring(cx, cy, r, n, a0=0, a1=2 * math.pi):
    a = np.linspace(a0, a1, n, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], 1)


def test_scan_points_filters_invalid():
    pts = scan_points([1.0, float('inf'), 0.01, 2.0], 0.0, math.pi / 2, 0.15, 12.0)
    assert pts == pytest.approx(np.array([[1, 0], [0, -2]]), abs=1e-9)


def test_cluster_scan_splits_and_wraps():
    # 一圈 5m 的墙，中间 1m 处有个人（扫描顺序是按角度的）
    ang = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    r = np.full(720, 5.0)
    person = np.abs(ang - 0.3) < 0.12
    r[person] = 1.0
    pts = scan_points(r, -math.pi, 2 * math.pi / 720, 0.1, 12)
    labels = cluster_scan(pts, jump=0.15)
    assert len(set(labels[person])) == 1 and labels[person][0] >= 0
    # 墙首尾相接，应该是同一簇（不是被 ±pi 切开的两簇）
    wall = ~person
    assert len(set(labels[wall])) <= 2


def test_measure_in_window_picks_person_not_wall():
    # 相机在原点朝 +x，人在 (2, 0.3)，背后是 x=5 的墙
    wall = np.stack([np.full(200, 5.0), np.linspace(-3, 3, 200)], 1)
    person = _ring(2.0, 0.3, 0.18, 40, math.pi / 2, 3 * math.pi / 2)   # 只看得到朝向机器人的半圈
    pts = np.concatenate([wall[wall[:, 1] < 0.3 - 0.6], person, wall[wall[:, 1] > 0.3 + 0.6]])
    order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
    pts = pts[order]
    labels = cluster_scan(pts, 0.15)
    center = math.atan2(0.3, 2.0)
    m = measure_in_window(pts, labels, (0, 0), center, 0.15, body_radius=0.12)
    assert m is not None
    xy, dist = m
    assert xy == pytest.approx([2.0, 0.3], abs=0.12)
    assert dist == pytest.approx(math.hypot(2.0, 0.3), abs=0.12)


def test_measure_in_window_rejects_wall_only():
    wall = np.stack([np.full(200, 3.0), np.linspace(-3, 3, 200)], 1)
    labels = cluster_scan(wall, 0.15)
    assert measure_in_window(wall, labels, (0, 0), 0.0, 0.1) is None      # 整面墙宽度太大，不是人


def test_measure_in_window_empty():
    assert measure_in_window(np.zeros((0, 2)), np.zeros(0, int), (0, 0), 0, 0.1) is None


def test_approach_goal_on_line_facing_person():
    x, y, yaw = approach_goal((5, 0), (0, 0), 3.0)
    assert (x, y) == pytest.approx((2, 0))
    assert yaw == pytest.approx(0)
    x, y, yaw = approach_goal((0, 4), (3, 0), 3.0)            # 斜着：停在连线上离人 3m
    assert math.hypot(x, y - 4) == pytest.approx(3.0)
    assert yaw == pytest.approx(math.atan2(4 - y, -x))


def test_approach_goal_too_close_turns_in_place():
    x, y, yaw = approach_goal((1, 1), (0, 0), 3.0)
    assert (x, y) == (0, 0)
    assert yaw == pytest.approx(math.pi / 4)
