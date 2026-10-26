import math

import numpy as np
import pytest

from person_locator.extcal import coverage, match_lidar, solve
from person_locator.geometry import cluster_scan


def person_blob(cx, cy, r=0.15, n=12):
    """激光打到的人：朝向原点那一侧的半圆弧"""
    a0 = math.atan2(-cy, -cx)
    a = a0 + np.linspace(-1.2, 1.2, n)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], 1)


def test_match_lidar_picks_nearest_bearing():
    pts = np.concatenate([person_blob(2.0, 0.3), person_blob(3.0, -1.5)])
    labels = cluster_scan(pts, 0.15)
    m = match_lidar(pts, labels, (0, 0), math.atan2(0.3, 2.0) + math.radians(3))
    assert m is not None
    assert m[0] == pytest.approx(math.atan2(0.3, 2.0), abs=math.radians(1.5))
    assert m[1] == pytest.approx(math.hypot(2.0, 0.3), abs=0.2)


def test_match_lidar_ambiguous_and_out_of_window():
    pts = np.concatenate([person_blob(2.0, 0.2), person_blob(2.0, -0.2)])
    labels = cluster_scan(pts, 0.1)
    assert match_lidar(pts, labels, (0, 0), 0.0) is None                      # 两个人分不清
    assert match_lidar(pts, labels, (0, 0), math.radians(60)) is None         # 窗口里没有


def synth(yaw_err, fx_true, fx_used, n=40, width=1280, seed=0):
    """真实相机朝向偏了 yaw_err；用 fx_used 算出相机方位 θc（当前外参 yaw=0）"""
    rng = np.random.default_rng(seed)
    tl = np.radians(rng.uniform(-30, 30, n))                 # 人的真实方位（=雷达方位）
    u = width / 2 + fx_true * np.tan(yaw_err - tl)           # 实际成像位置
    tc = -np.arctan((u - width / 2) / fx_used)               # 按当前参数算出的方位
    tl = tl + np.radians(rng.normal(0, 0.3, n))              # 雷达方位噪声
    return np.stack([u, tc, tl], 1)


def test_solve_recovers_yaw():
    r = solve(synth(math.radians(4), 1050, 1050), current_yaw=0.0, width=1280, fx_now=1050)
    assert r['ok']
    assert r['yaw_deg'] == pytest.approx(4.0, abs=0.3)
    assert r['rms_deg'] < 0.6
    assert r['fx_pinhole'] == pytest.approx(1050, rel=0.08)
    assert r['message'] == ''


def test_solve_flags_wrong_fx():
    r = solve(synth(math.radians(2), 1050, 2770), current_yaw=0.0, width=1280, fx_now=2770)
    assert r['ok']
    assert r['fx_pinhole'] == pytest.approx(1050, rel=0.1)
    assert 'fx' in r['message']


def test_solve_adds_to_current_yaw_and_rejects_outliers():
    s = synth(math.radians(-3), 1050, 1050, seed=1)
    s[:, 1] += math.radians(1.0)        # 当前外参已经有 +1° 的 yaw，θc 跟着偏
    s[:3, 2] += math.radians(25)        # 3 个错配
    r = solve(s, current_yaw=math.radians(1.0), width=1280, fx_now=1050)
    assert r['inliers'] == len(s) - 3
    assert r['yaw_deg'] == pytest.approx(-3.0, abs=0.3)


def test_solve_needs_samples_and_spread():
    assert not solve(np.zeros((3, 3)), 0.0, 1280)['ok']
    s = synth(0.0, 1050, 1050)
    s[:, 1:] *= 0.1                     # 只在正前方 ±3° 里
    r = solve(s, 0.0, 1280, fx_now=1050)
    assert r['ok'] and r['fx_pinhole'] is None and '覆盖' in r['message']


def test_coverage():
    assert coverage(np.radians([-35, 5, 6, 39, 80])) == [1, 0, 0, 0, 2, 0, 0, 1]
