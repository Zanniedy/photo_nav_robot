"""多人跟踪（不依赖 ROS，单元测试见 test/test_tracker.py）

- 每个人一条轨迹，匀速模型卡尔曼滤波，状态 [x, y, vx, vy]（跟踪系，一般是 map）
- 轨迹只由相机检测创建（避免把墙角、桌腿当人）；之后相机和激光都能更新
- 相机远处检测不到人时，靠激光簇继续跟；但人在相机视野内且距离较近、相机却长时间没看到，就判定跟丢
  （防止轨迹漂到旁边的柱子上）
- 锁定目标：开始时选离机器人最近的；目标丢了，再选当前离机器人最近的
- 人脸：相机观测可以带「这一帧看到了脸」，记在 t_face；锁定时可以只选最近 face_within 秒内看到过脸的
"""
import math

import numpy as np


class Track:
    _next_id = 1

    def __init__(self, xy, t, pos_std=0.3):
        self.id = Track._next_id
        Track._next_id += 1
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.diag([pos_std ** 2, pos_std ** 2, 1.0, 1.0])
        self.t = t              # 状态对应的时间
        self.t_cam = t          # 最近一次相机确认
        self.t_upd = t          # 最近一次任意更新
        self.t_face = -math.inf  # 最近一次看到人脸
        self.cam_hits = 1

    @property
    def xy(self):
        return self.x[:2].copy()

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    def predict(self, t, accel_std):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = accel_std ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        self.t = t

    def gate_dist(self, z):
        """到观测的马氏距离（只看位置）"""
        S = self.P[:2, :2] + np.eye(2) * 1e-4
        d = np.asarray(z) - self.x[:2]
        return float(math.sqrt(d @ np.linalg.solve(S, d)))

    def update(self, z, std, t, from_camera):
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1
        R = np.eye(2) * std ** 2
        S = H @ self.P @ H.T + R
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ (np.asarray(z) - H @ self.x)
        self.P = (np.eye(4) - Kg @ H) @ self.P
        self.t_upd = max(self.t_upd, t)
        if from_camera:
            self.t_cam = max(self.t_cam, t)
            self.cam_hits += 1


class Tracker:
    def __init__(self, gate=0.8, lidar_gate=0.5, accel_std=1.5, cam_std=0.15, lidar_std=0.08,
                 lost_timeout=2.0, cam_confirm_timeout=3.0, max_speed=2.5):
        self.gate = gate                    # 相机观测关联半径 m
        self.lidar_gate = lidar_gate        # 激光簇关联半径 m（比相机小，防止吸到旁边的物体）
        self.accel_std = accel_std
        self.cam_std = cam_std
        self.lidar_std = lidar_std
        self.lost_timeout = lost_timeout    # 多久没有任何更新算丢失
        self.cam_confirm_timeout = cam_confirm_timeout
        self.max_speed = max_speed          # 估计速度超过这个就不是人（多半关联错了）
        self.tracks: list[Track] = []
        self.target_id = None

    # ---------------- 更新 ----------------
    def _predict(self, t):
        for tr in self.tracks:
            tr.predict(t, self.accel_std)

    def _associate(self, zs, gate):
        """贪心最近邻：返回 [(track, z_index)]，以及未匹配的观测下标"""
        pairs = []
        for i, tr in enumerate(self.tracks):
            for j, z in enumerate(zs):
                d = float(np.hypot(*(np.asarray(z) - tr.x[:2])))
                if d <= gate:
                    pairs.append((d, i, j))
        pairs.sort()
        used_t, used_z, out = set(), set(), []
        for _, i, j in pairs:
            if i in used_t or j in used_z:
                continue
            used_t.add(i)
            used_z.add(j)
            out.append((self.tracks[i], j))
        return out, [j for j in range(len(zs)) if j not in used_z]

    def update_camera(self, t, zs, faces=None):
        """zs: 相机检测 + 激光测距得到的人位置列表；faces: 和 zs 对应，这个人这一帧有没有看到脸"""
        self._predict(t)
        matched, new = self._associate(zs, self.gate)
        for tr, j in matched:
            tr.update(zs[j], self.cam_std, t, from_camera=True)
            if faces is not None and faces[j]:
                tr.t_face = t
        for j in new:
            tr = Track(zs[j], t)
            if faces is not None and faces[j]:
                tr.t_face = t
            self.tracks.append(tr)

    def update_lidar(self, t, centroids):
        """centroids: 宽度像人的激光簇质心。只更新已有轨迹，不新建"""
        if not self.tracks or len(centroids) == 0:
            return
        self._predict(t)
        matched, _ = self._associate(list(centroids), self.lidar_gate)
        for tr, j in matched:
            tr.update(centroids[j], self.lidar_std, t, from_camera=False)

    def prune(self, t, in_camera_view=None):
        """in_camera_view(xy) -> bool：这个位置相机应该能检测到人（在视野内且不太远）"""
        keep = []
        for tr in self.tracks:
            if t - tr.t_upd > self.lost_timeout:
                continue
            if tr.speed > self.max_speed:
                continue
            if in_camera_view is not None and t - tr.t_cam > self.cam_confirm_timeout and in_camera_view(tr.xy):
                continue
            keep.append(tr)
        self.tracks = keep
        if self.target_id is not None and self.get(self.target_id) is None:
            self.target_id = None

    # ---------------- 目标 ----------------
    def get(self, tid):
        return next((tr for tr in self.tracks if tr.id == tid), None)

    def nearest(self, robot_xy, t, cam_within=1.0, face_within=None):
        """离机器人最近、且 cam_within 秒内被相机确认过的轨迹；face_within 不为 None 时还要这么多秒内看到过脸"""
        cands = [tr for tr in self.tracks if t - tr.t_cam <= cam_within
                 and (face_within is None or t - tr.t_face <= face_within)]
        if not cands:
            return None
        return min(cands, key=lambda tr: float(np.hypot(*(tr.xy - np.asarray(robot_xy)))))

    def target(self, robot_xy, t, face_within=None):
        """当前锁定的目标；没有或丢了就重新锁最近的（face_within 见 nearest）。返回 (track, 是否刚换了目标)"""
        tr = self.get(self.target_id) if self.target_id is not None else None
        if tr is not None:
            return tr, False
        tr = self.nearest(robot_xy, t, face_within=face_within)
        changed = tr is not None
        self.target_id = tr.id if tr is not None else None
        return tr, changed

    def release(self):
        self.target_id = None
