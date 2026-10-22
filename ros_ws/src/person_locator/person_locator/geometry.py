"""人员定位的几何计算（不依赖 ROS，单元测试见 test/test_geometry.py）

坐标约定：所有平面量都在跟踪坐标系（一般是 map）下，xy 单位 m，角度单位 rad。
"""
import math

import numpy as np


def wrap(a):
    """角度归一化到 (-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pixel_rays(uv, K, D):
    """像素 (N,2) → 光学系下的射线 (N,3)（z=1，未归一化）。D 为空或全 0 时不做去畸变"""
    uv = np.asarray(uv, np.float64).reshape(-1, 2)
    K = np.asarray(K, np.float64).reshape(3, 3)
    if D is not None and len(D) and np.any(np.asarray(D) != 0):
        import cv2
        xy = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, np.asarray(D, np.float64)).reshape(-1, 2)
    else:
        xy = np.stack([(uv[:, 0] - K[0, 2]) / K[0, 0], (uv[:, 1] - K[1, 2]) / K[1, 1]], 1)
    return np.concatenate([xy, np.ones((len(xy), 1))], 1)


def ray_bearings(rays_opt, R_track_opt):
    """光学系射线 → 跟踪系下的水平方位角（只看 xy 分量）"""
    d = np.asarray(rays_opt) @ np.asarray(R_track_opt).T
    return np.arctan2(d[:, 1], d[:, 0])


def scan_points(ranges, angle_min, angle_inc, range_min, range_max):
    """LaserScan → 雷达系 (N,2) 点，只保留有效距离，顺序和扫描顺序一致"""
    r = np.asarray(ranges, np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)


def cluster_scan(pts, jump=0.15, min_pts=3):
    """按扫描顺序做断点聚类：相邻点距离 > jump 就断开，首尾相连（360° 雷达）。
    返回 labels (N,)，-1 表示点数不足 min_pts 的碎片"""
    n = len(pts)
    labels = np.full(n, -1, int)
    if n == 0:
        return labels
    gap = np.linalg.norm(np.diff(pts, axis=0), axis=1) > jump
    seg = np.concatenate([[0], np.cumsum(gap)])
    if n > 1 and np.linalg.norm(pts[0] - pts[-1]) <= jump and seg[-1] > 0:
        seg[seg == seg[-1]] = 0     # 首尾是同一簇
    next_id = 0
    for s in np.unique(seg):
        idx = np.nonzero(seg == s)[0]
        if len(idx) >= min_pts:
            labels[idx] = next_id
            next_id += 1
    return labels


def cluster_stats(pts, labels):
    """每个簇的 (质心 (M,2), 宽度 (M,))，宽度 = 簇内两端点距离的上界（包围盒对角线）"""
    ids = np.unique(labels[labels >= 0])
    cents, widths = [], []
    for i in ids:
        p = pts[labels == i]
        cents.append(p.mean(0))
        widths.append(float(np.linalg.norm(p.max(0) - p.min(0))))
    return np.asarray(cents).reshape(-1, 2), np.asarray(widths), ids


def measure_in_window(pts, labels, origin, center, half_width, body_radius=0.12, quantile=0.2,
                      min_width=0.1, max_width=1.0):
    """在以 origin 为顶点、方位 center±half_width 的扇形里找人。

    pts/labels: 跟踪系下的扫描点和聚类结果；origin: 相机在跟踪系下的 xy。
    做法：扇形内距离的低分位数（去掉背景墙）→ 离这个距离最近的点所在的簇 → 簇质心沿视线往后推 body_radius
    （激光只打到人的前表面）。簇宽度不像人时返回 None。
    返回 (xy, 距离) 或 None
    """
    if len(pts) == 0:
        return None
    d = pts - np.asarray(origin)
    ang = np.arctan2(d[:, 1], d[:, 0])
    rng = np.hypot(d[:, 0], d[:, 1])
    inside = (np.abs(wrap(ang - center)) <= half_width) & (labels >= 0)
    if not inside.any():
        return None
    r_in = rng[inside]
    q = np.quantile(r_in, quantile)
    k = np.nonzero(inside)[0][np.argmin(np.abs(r_in - q))]
    p = pts[labels == labels[k]]
    width = float(np.linalg.norm(p.max(0) - p.min(0)))
    if not (min_width <= width <= max_width):
        return None
    c = p.mean(0)
    v = c - np.asarray(origin)
    dist = float(np.hypot(*v))
    if dist < 1e-6:
        return None
    c = c + v / dist * body_radius
    return c, dist + body_radius


def approach_goal(person, robot, distance):
    """在「机器人→人」连线上、离人 distance 处的目标位姿 (x, y, yaw)，车头朝人。
    机器人已经在 distance 以内时不后退，原地转向人。"""
    p, r = np.asarray(person, float), np.asarray(robot, float)
    v = r - p
    d = float(np.hypot(*v))
    if d <= distance or d < 1e-6:
        g = r
    else:
        g = p + v / d * distance
    yaw = math.atan2(p[1] - g[1], p[0] - g[0])
    return float(g[0]), float(g[1]), yaw
