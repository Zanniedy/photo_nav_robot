import numpy as np

from person_locator.face import frontal_ratio, head_region, is_frontal
from person_locator.tracker import Tracker


def face(rex, lex, nx):
    f = np.zeros(15)
    f[4], f[6], f[8] = rex, lex, nx
    return f


def test_head_region_upper_part_clamped():
    assert head_region([100, 50, 200, 350], 640, 360) == (85, 35, 215, 185)
    assert head_region([0, 0, 100, 200], 640, 360)[:2] == (0, 0)
    assert head_region([600, 10, 640, 300], 640, 360)[2] == 640


def test_frontal():
    assert frontal_ratio(face(100, 140, 120)) == 0.5
    assert is_frontal(face(100, 140, 125))
    assert not is_frontal(face(100, 140, 139))          # 鼻尖贴着一只眼：侧脸
    assert frontal_ratio(face(100, 100, 100)) is None


def test_tracker_face_gating():
    tk = Tracker()
    tk.update_camera(0.0, [np.array([1.0, 0.0]), np.array([3.0, 0.0])], [False, True])
    assert tk.nearest((0, 0), 0.5).xy[0] == 1.0                      # 不要求人脸：最近的
    assert tk.nearest((0, 0), 0.5, face_within=1.5).xy[0] == 3.0     # 要求人脸：看到过脸的
    assert tk.nearest((0, 0), 2.0, face_within=1.5) is None          # 太久没看到脸
    tk.update_camera(2.0, [np.array([1.0, 0.0])], [True])
    tr, _ = tk.target((0, 0), 2.0, face_within=1.5)
    assert tr.xy[0] == 1.0
