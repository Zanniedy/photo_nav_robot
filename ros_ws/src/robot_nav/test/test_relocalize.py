#!/usr/bin/env python3
"""
relocalize 离线测试：在真实地图上用光线投射合成 scan，扰动初值后检查能否精配回真值

用法: python3 test/test_relocalize.py <map.yaml>
"""
import math
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from relocalize_node import MapField, match, scan_to_points  # noqa: E402


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']), cv2.IMREAD_GRAYSCALE)
    img = np.flipud(img)  # pgm 第一行是地图顶部，OccupancyGrid 第一行是 y 最小
    occ = (255 - img.astype(np.float32)) / 255.0
    data = np.where(occ > meta['occupied_thresh'], 100,
                    np.where(occ < meta['free_thresh'], 0, -1)).astype(np.int16)
    return data, meta['resolution'], meta['origin'][0], meta['origin'][1]


def raycast(grid, res, ox, oy, x, y, th, n=450, rmax=12.0, rng=None):
    h, w = grid.shape
    angles = -math.pi + 2 * math.pi * np.arange(n) / n
    ranges = np.full(n, np.inf, dtype=np.float32)
    steps = np.arange(0.05, rmax, res * 0.5)
    for i, a in enumerate(angles):
        px = x + steps * math.cos(th + a)
        py = y + steps * math.sin(th + a)
        ix = ((px - ox) / res).astype(int)
        iy = ((py - oy) / res).astype(int)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        hit = np.nonzero(ok & (grid[np.clip(iy, 0, h - 1), np.clip(ix, 0, w - 1)] >= 65))[0]
        if hit.size:
            ranges[i] = steps[hit[0]]
    if rng is not None:
        ranges += rng.normal(0, 0.02, n).astype(np.float32)        # 测距噪声
        ranges[rng.random(n) < 0.1] = np.inf                       # 10% 丢点
    return ranges, angles[0], angles[1] - angles[0]


def main():
    data, res, ox, oy = load_map(sys.argv[1])
    h, w = data.shape
    field = MapField(data.ravel(), w, h, res, ox, oy)

    rng = np.random.default_rng(0)
    # 真值取地图中离障碍 0.3~1.5m 的空闲点
    cand = np.argwhere((data == 0) & (field.dist > 0.3) & (field.dist < 1.5))
    fails, n_trials = 0, 20
    for k in range(n_trials):
        iy, ix = cand[rng.integers(len(cand))]
        gx, gy = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
        gth = rng.uniform(-math.pi, math.pi)
        ranges, amin, ainc = raycast(data, res, ox, oy, gx, gy, gth, rng=rng)
        pts = scan_to_points(ranges, amin, ainc, 0.05, 12.0)

        # 初值扰动：位置 0.5~0.9m、朝向 10~25°，模拟手点误差
        r = rng.uniform(0.5, 0.9)
        a = rng.uniform(-math.pi, math.pi)
        x0, y0 = gx + r * math.cos(a), gy + r * math.sin(a)
        th0 = gth + math.radians(rng.uniform(10, 25)) * rng.choice([-1, 1])

        import time
        t = time.time()
        x, y, th, inl = match(field, pts, x0, y0, th0)
        dt = (time.time() - t) * 1000
        exy = math.hypot(x - gx, y - gy)
        eth = abs(math.degrees(math.atan2(math.sin(th - gth), math.cos(th - gth))))
        ok = exy < 0.05 and eth < 1.0
        fails += not ok
        print(f'[{k:2d}] {"OK  " if ok else "FAIL"} err {exy:.3f}m {eth:.2f}°  inlier {inl:.2f}  {dt:.0f}ms')

    print(f'{n_trials - fails}/{n_trials} 通过')
    sys.exit(1 if fails > n_trials // 10 else 0)


if __name__ == '__main__':
    main()
