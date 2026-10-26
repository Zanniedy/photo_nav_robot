"""相机-雷达联合标定（只标相机相对车体的 yaw，顺带粗估 fx）。不依赖 ROS，单元测试见 test/test_extcal.py

做法：画面里只站一个人，在机器人前方不同方位来回走。每一帧
    相机方位 θc：检测框中心像素按当前内参和当前外参算出的、base_link 下的水平方位
    雷达方位 θl：同一时刻激光里这个人那一簇的质心，相对相机位置的方位（也在 base_link 下）
相机 yaw 的修正量 δ = median(θl - θc)，新的 yaw = 当前 yaw + δ。
残差随方位有系统性变化（左边偏正、右边偏负）说明 fx 不对，所以另外不带畸变地拟合
    u - cx = fx · tan(ψ - θl)        （cx 取画面中心，ψ 是相机朝向）
给出一个 fx 的粗估，用来判断内参标定结果靠不靠谱。
"""
import math

import numpy as np

from .geometry import cluster_stats, wrap


def match_lidar(pts, labels, origin, bearing, search=math.radians(20), min_width=0.1, max_width=1.0,
                min_range=0.3, max_range=6.0, ambiguity=math.radians(6)):
    """在相机方位 bearing 附近 ±search 里找人那一簇。

    pts/labels: base_link 下的扫描点和聚类；origin: 相机 xy。
    宽度像人、距离合适的簇里取方位最接近的；第二近的只比它差不到 ambiguity 时算分不清，返回 None。
    返回 (簇质心方位, 距离) 或 None
    """
    cents, widths, _ = cluster_stats(pts, labels)
    if not len(cents):
        return None
    d = cents - np.asarray(origin)
    rng = np.hypot(d[:, 0], d[:, 1])
    ang = np.arctan2(d[:, 1], d[:, 0])
    err = np.abs(wrap(ang - bearing))
    ok = (widths >= min_width) & (widths <= max_width) & (rng >= min_range) & (rng <= max_range) & (err <= search)
    if not ok.any():
        return None
    idx = np.nonzero(ok)[0][np.argsort(err[ok])]
    if len(idx) > 1 and err[idx[1]] - err[idx[0]] < ambiguity:
        return None
    k = idx[0]
    return float(ang[k]), float(rng[k])


def _pinhole_fit(u, theta, cx, center, span=math.radians(20), step=math.radians(0.02)):
    """u - cx = fx·tan(ψ - θ)：网格搜 ψ，fx 闭式最小二乘。返回 (fx, ψ, 像素残差 rms)"""
    best = None
    d = u - cx
    for psi in np.arange(center - span, center + span, step):
        t = np.tan(wrap(psi - theta))
        tt = float(t @ t)
        if tt < 1e-9:
            continue
        fx = float(t @ d) / tt
        if fx <= 0:
            continue
        rms = float(np.sqrt(np.mean((d - fx * t) ** 2)))
        if best is None or rms < best[2]:
            best = (fx, float(psi), rms)
    return best


def solve(samples, current_yaw, width, fx_now=None, min_samples=10, min_spread=math.radians(20)):
    """samples: (N,3) 每行 (u 像素, θc, θl)。返回结果 dict，样本不够时 ok=False 并给原因"""
    s = np.asarray(samples, np.float64).reshape(-1, 3)
    n = len(s)
    if n < min_samples:
        return {'ok': False, 'n': n, 'message': f'样本太少（{n}/{min_samples}）'}
    u, tc, tl = s[:, 0], s[:, 1], s[:, 2]
    d = wrap(tl - tc)
    med = float(np.median(d))
    mad = float(np.median(np.abs(wrap(d - med)))) * 1.4826
    inl = np.abs(wrap(d - med)) <= max(3 * mad, math.radians(1.0))
    delta = med + float(np.mean(wrap(d[inl] - med)))
    res = wrap(d[inl] - delta)
    rel = wrap(tl[inl] - (current_yaw + delta))         # 人相对相机光轴的方位，看覆盖范围
    spread = float(rel.max() - rel.min())
    # 残差对方位的斜率：fx 偏大时边缘的人算出来的方位偏小，斜率为正
    slope = float(np.polyfit(rel, res, 1)[0]) if spread > math.radians(5) else 0.0
    out = {
        'ok': True, 'n': n, 'inliers': int(inl.sum()),
        'delta_deg': math.degrees(delta), 'yaw_deg': math.degrees(current_yaw + delta), 'yaw': current_yaw + delta,
        'current_yaw_deg': math.degrees(current_yaw),
        'rms_deg': math.degrees(float(np.sqrt(np.mean(res ** 2)))),
        'spread_deg': math.degrees(spread), 'slope': slope,
        'fx_pinhole': None, 'pinhole_rms_px': None, 'message': '',
    }
    notes = []
    if spread >= min_spread:
        fit = _pinhole_fit(u[inl], tl[inl], width / 2, current_yaw + delta)
        if fit is not None:
            out['fx_pinhole'], out['pinhole_rms_px'] = round(fit[0], 1), round(fit[2], 1)
            if fx_now and abs(fit[0] / fx_now - 1) > 0.25:
                notes.append(f'针孔粗估 fx≈{fit[0]:.0f}，和当前内参 fx={fx_now:.0f} 差得多，内参可能要重标')
    else:
        notes.append(f'方位覆盖只有 {math.degrees(spread):.0f}°，左右再走开一点才能估 fx')
    if abs(slope) > 0.1:
        notes.append(f'残差随方位变化（斜率 {slope:.2f}），fx 可能不准')
    out['message'] = '；'.join(notes)
    return out


def coverage(rel_bearings, bins_deg=(-40, -30, -20, -10, 0, 10, 20, 30, 40)):
    """相对光轴方位的直方图（度，左正右负），网页上提示哪边还缺样本"""
    if not len(rel_bearings):
        return [0] * (len(bins_deg) - 1)
    h, _ = np.histogram(np.degrees(np.asarray(rel_bearings)), bins=bins_deg)
    return [int(v) for v in h]
