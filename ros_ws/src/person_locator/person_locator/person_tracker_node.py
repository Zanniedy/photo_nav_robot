#!/usr/bin/env python3
"""人员跟踪 + 走到人面前

相机（BPU 检测 person 框）给方位，2D 激光在这个方位的扇形里取距离，得到人在 map 下的位置，
再用卡尔曼滤波跟踪（相机远处检测不到时靠激光簇继续跟）。
人脸（face.py，YuNet）只在人体框上半部分里找；require_face=true 时只锁定最近 face_within 秒内看到过脸的人。

订阅:
    /camera/image_raw/compressed  /camera/camera_info  /scan
发布:
    /person/markers   visualization_msgs/MarkerArray  所有轨迹（目标为绿色）、标记点、导航目标
    /person/target    geometry_msgs/PoseStamped       当前锁定目标的位置
    /person/state     std_msgs/String                 JSON 调试状态（网页用）：轨迹、记住的位置（红点）、目标点、动作状态、外参标定
    /person/arrived   std_msgs/String                 approach 成功到达人面前时发一次 JSON（给语音播报等上层用）:
                      {"target_id", "person": [x, y], "robot": [x, y], "distance", "frame", "stamp"}
服务（网页调试用，std_srvs/Trigger）:
    /person/approach_start   按当前参数（approach_distance、face_front）开始一次 approach
    /person/approach_cancel  取消正在执行的 approach
    /person/extcal_start     相机-雷达外参标定：清空样本开始采集（画面里只站一个人，左右来回走，机器人别动）
    /person/extcal_solve     停止采集并计算相机 yaw（extcal.py），结果在 /person/state 的 extcal 里
    /person/extcal_save      把算出的 yaw 写到 extrinsic_file，robot.launch.py 下次启动时读（要重启 base 才生效）
Action:
    /person/approach  robot_interfaces/action/ApproachPerson
        锁定最近的人 → 导航到离人 distance 米处、车头朝人

参数 face_front:
    false（默认）  目标点在「机器人→人」连线上，导航中人移动超过 replan_dist 就更新目标点
    true           锁定后用前 mark_frames 次相机观测粗匹配出人的位置，把「人面前 distance 米」的位姿
                   记下来，直到这次导航结束都按记下的位姿走，不再随跟踪更新。人的朝向取标记时
                   人→机器人的方向（require_face=true 时锁定前看到过正脸，这个假设基本成立）

    ros2 run person_locator person_tracker --ros-args --params-file config/person_tracker.yaml
    ros2 action send_goal /person/approach robot_interfaces/action/ApproachPerson "{distance: 3.0}" --feedback
"""
import json
import math
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_interfaces.action import ApproachPerson

from . import extcal
from .geometry import (approach_goal, cluster_scan, cluster_stats, measure_in_window, pixel_rays,
                       quat_to_rot, ray_bearings, scan_points, wrap)
from .tracker import Tracker

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        p = self.declare_parameter
        self.model_path = os.path.expanduser(p('model_path', '~/max_robot/models/yolo26s_seg_bpu.bin').value)
        self.conf = p('conf', 0.4).value
        self.track_frame = p('track_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.max_det_rate = p('max_det_rate', 10.0).value
        self.window_shrink = p('window_shrink', 0.6).value      # 激光扇形取框宽的多少（框边上多是背景）
        self.body_radius = p('body_radius', 0.12).value
        self.cluster_jump = p('cluster_jump', 0.15).value
        self.min_width = p('person_min_width', 0.1).value
        self.max_width = p('person_max_width', 1.0).value
        self.cam_confirm_range = p('cam_confirm_range', 5.0).value   # 这个距离内相机应该能检测到人
        p('approach_distance', 3.0)                             # 这两个每次 action 开始时读
        p('search_timeout', 5.0)
        self.replan_dist = p('replan_dist', 0.5).value
        self.mark_frames = p('mark_frames', 3).value
        self.declare_parameter('face_front', False)              # 每次 action 开始时读，可以运行中 param set
        p('require_face', True)                                 # 每次 action 开始时读
        self.face_within = p('face_within', 1.5).value
        face_model = os.path.expanduser(p('face_model', '~/max_robot/models/face_detection_yunet_2023mar.onnx').value)
        face_conf = p('face_conf', 0.6).value
        # 没有对应分辨率的 camera_info 时，按这个水平视场角估一个无畸变针孔内参（0 = 不估，等标定）
        self.fallback_hfov = p('fallback_hfov_deg', 70.0).value
        self.extrinsic_file = os.path.expanduser(p('extrinsic_file', '~/max_robot/calib/camera_extrinsic.yaml').value)
        self.extcal_search = math.radians(p('extcal_search_deg', 20.0).value)
        self.tracker = Tracker(lost_timeout=p('lost_timeout', 2.0).value,
                               cam_confirm_timeout=p('cam_confirm_timeout', 3.0).value)

        self.lock = threading.Lock()
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.info = None
        self.latest_img = None
        self.img_event = threading.Event()
        self.scan = None            # (t, pts_track, labels)
        self.scan_raw = None        # (t, frame_id, pts_laser, labels)：外参标定用，不经过 map
        self.extcal = {'on': False, 'samples': [], 'rel': [], 'skip': {'n_det': 0, 'sync': 0, 'no_match': 0},
                       'result': None, 'saved': ''}
        self.cam_view = None        # (origin_xy, axis_bearing, half_fov)：判断「相机应该看得到」
        # 记住的人位置（红点）：导航目标就是按它算的。动作结束后保留（active=False），方便网页上对比
        self.mark = None            # {'xy': (x,y), 'frames': n, 'face_front': bool, 'active': bool}
        self.nav_goal = None        # 当前导航目标 (x,y,yaw)，显示用
        self.approach = {'state': 'idle', 'target_id': -1, 'distance': -1.0, 'message': ''}
        self.det_boxes = {'t': 0.0, 'boxes': []}
        self.det_stat = {'n': 0, 'ms': 0.0, 'hz': 0.0, 'with_range': 0, 'faces': 0, 'face_ms': 0.0}

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, SENSOR_QOS, callback_group=self.cb)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_image, SENSOR_QOS,
                                 callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR_QOS, callback_group=self.cb)
        self.pub_markers = self.create_publisher(MarkerArray, '/person/markers', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/person/target', 10)
        self.pub_state = self.create_publisher(String, '/person/state', 10)
        self.pub_arrived = self.create_publisher(String, '/person/arrived', 10)
        self.create_timer(0.2, self.publish_vis, callback_group=self.cb)

        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb)
        self.busy = False
        ActionServer(self, ApproachPerson, '/person/approach', self.execute,
                     goal_callback=self.on_goal, cancel_callback=lambda _g: CancelResponse.ACCEPT,
                     callback_group=self.cb)
        self.self_client = ActionClient(self, ApproachPerson, '/person/approach', callback_group=self.cb)
        self.self_handle = None
        self.create_service(Trigger, '/person/approach_start', self.srv_start, callback_group=self.cb)
        self.create_service(Trigger, '/person/approach_cancel', self.srv_cancel, callback_group=self.cb)
        self.create_service(Trigger, '/person/extcal_start', self.srv_extcal_start, callback_group=self.cb)
        self.create_service(Trigger, '/person/extcal_solve', self.srv_extcal_solve, callback_group=self.cb)
        self.create_service(Trigger, '/person/extcal_save', self.srv_extcal_save, callback_group=self.cb)

        from .detector import BpuPersonDetector
        self.detector = BpuPersonDetector(self.model_path, conf=self.conf)
        self.face = None
        if os.path.exists(face_model):
            from .face import FaceDetector
            self.face = FaceDetector(face_model, conf=face_conf)
        else:
            self.get_logger().warn(f'没有人脸模型 {face_model}，不做人脸检测（require_face 会一直找不到人）')
        self.running = True
        self.det_thread = threading.Thread(target=self.det_loop, daemon=True)
        self.det_thread.start()
        self.get_logger().info(f'人员跟踪就绪: 模型 {self.model_path}，跟踪系 {self.track_frame}，'
                               f'face_front={self.get_parameter("face_front").value}')

    # ---------------- TF ----------------
    def lookup(self, source, stamp=None, target=None):
        """source → target（默认 track_frame）的 (R 3x3, t 3)；按时间戳查不到就退回最新的"""
        for when in ([Time.from_msg(stamp)] if stamp is not None else []) + [Time()]:
            try:
                tf = self.tf.lookup_transform(target or self.track_frame, source, when, timeout=Duration(seconds=0.05))
                q, t = tf.transform.rotation, tf.transform.translation
                return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
            except Exception:   # noqa: BLE001  tf2 的几种异常都按查不到处理
                continue
        return None

    def robot_xy(self):
        T = self.lookup(self.base_frame)
        return None if T is None else T[1][:2]

    # ---------------- 传感器 ----------------
    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.latest_img = msg
        self.img_event.set()

    def on_scan(self, msg):
        pts = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        labels = cluster_scan(pts, self.cluster_jump)       # 在雷达系里聚类（扫描顺序是对的）
        self.scan_raw = (stamp_sec(msg.header.stamp), msg.header.frame_id, pts, labels)
        T = self.lookup(msg.header.frame_id, msg.header.stamp)
        if T is None:
            self.get_logger().warn(f'没有 {msg.header.frame_id}→{self.track_frame} 的 TF', throttle_duration_sec=5.0)
            return
        R, t = T
        pts_tr = pts @ R[:2, :2].T + t[:2]
        cents, widths, _ = cluster_stats(pts_tr, labels)
        ok = (widths >= self.min_width) & (widths <= self.max_width)
        now = stamp_sec(msg.header.stamp)
        with self.lock:
            self.scan = (now, pts_tr, labels)
            self.tracker.update_lidar(now, cents[ok])
            self.tracker.prune(now, self.in_camera_view)

    def in_camera_view(self, xy):
        cv = self.cam_view
        if cv is None:
            return False
        origin, axis, half = cv
        d = np.asarray(xy) - origin
        return (math.hypot(*d) < self.cam_confirm_range
                and abs(wrap(math.atan2(d[1], d[0]) - axis)) < half * 0.8)

    def det_loop(self):
        last = 0.0
        while self.running and rclpy.ok():
            if not self.img_event.wait(0.5):
                continue
            self.img_event.clear()
            wait = 1.0 / self.max_det_rate - (time.time() - last)
            if wait > 0:
                time.sleep(wait)
            last = time.time()
            msg = self.latest_img
            try:
                self.process(msg)
            except Exception as e:   # noqa: BLE001  单帧出错不能把线程弄死
                self.get_logger().error(f'检测出错: {e}', throttle_duration_sec=5.0)

    def _fallback_info(self, w, h):
        info = CameraInfo(width=w, height=h)
        fx = w / 2 / math.tan(math.radians(self.fallback_hfov) / 2)
        info.k = [fx, 0.0, w / 2, 0.0, fx, h / 2, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        return info

    def process(self, msg):
        info = self.info
        if info is None and self.fallback_hfov <= 0:
            self.get_logger().warn('没有 /camera/camera_info（相机还没标定？）', throttle_duration_sec=10.0)
            return
        # 1280 宽的 jpeg 直接按 1/2 解码（640x360），letterbox 就不用再缩放
        reduce = info is not None and info.width >= 1280
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                           cv2.IMREAD_REDUCED_COLOR_2 if reduce else cv2.IMREAD_COLOR)
        if bgr is None:
            return
        if info is None:
            info = self._fallback_info(bgr.shape[1], bgr.shape[0])
            self.get_logger().warn(f'没有 camera_info，按水平视场 {self.fallback_hfov:.0f}° 估内参（方位不准，请重新标定）',
                                   throttle_duration_sec=30.0)
        t_det = time.time()
        dets = self.detector(bgr)
        ms = (time.time() - t_det) * 1e3
        t_face = time.time()
        faces = self.face(bgr, dets) if self.face is not None and len(dets) else [None] * len(dets)
        face_ms = (time.time() - t_face) * 1e3
        dets[:, :4] *= info.width / bgr.shape[1]
        self.det_boxes = self._boxes_json(dets, faces, info, bgr.shape)
        # 统计放在 TF 之前：没开导航（没有 map）时网页上也能看到检测在跑
        st = self.det_stat
        dt = time.time() - st.get('t', time.time() - 1)
        st.update(n=len(dets), with_range=0, ms=0.8 * st['ms'] + 0.2 * ms, faces=sum(f is not None for f in faces),
                  face_ms=0.8 * st['face_ms'] + 0.2 * face_ms, hz=0.8 * st['hz'] + 0.2 / max(dt, 1e-3), t=time.time())
        K, D = np.array(info.k).reshape(3, 3), list(info.d)
        if self.extcal['on']:
            self._extcal_sample(msg, dets, K, D, info.width)

        T = self.lookup(msg.header.frame_id, msg.header.stamp)
        if T is None:
            self.get_logger().warn(f'没有 {msg.header.frame_id}→{self.track_frame} 的 TF', throttle_duration_sec=5.0)
            return
        R, t = T
        origin = t[:2]
        axis = ray_bearings(pixel_rays([[K[0, 2], K[1, 2]]], K, D), R)[0]
        half_fov = math.atan(info.width / 2 / K[0, 0])
        now = stamp_sec(msg.header.stamp)

        zs, zf = [], []
        with self.lock:
            self.cam_view = (origin, axis, half_fov)
            scan = self.scan
        if scan is not None and len(dets):
            _, pts, labels = scan
            for (x1, y1, x2, y2, _s), f in zip(dets, faces):
                uc, v, hw = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 2 * self.window_shrink
                b = ray_bearings(pixel_rays([[uc, v], [uc - hw, v], [uc + hw, v]], K, D), R)
                m = measure_in_window(pts, labels, origin, b[0], abs(wrap(b[1] - b[2])) / 2,
                                      self.body_radius, min_width=self.min_width, max_width=self.max_width)
                if m is not None:
                    zs.append(m[0])
                    zf.append(f is not None)
        st['with_range'] = len(zs)
        with self.lock:
            if zs:
                self.tracker.update_camera(now, zs, zf)
            self.tracker.prune(now, self.in_camera_view)

    @staticmethod
    def _boxes_json(dets, faces, info, shape):
        """网页相机预览上画框用：坐标归一化到 0~1"""
        from .face import is_frontal
        W, H, w, h = info.width, info.height, shape[1], shape[0]
        out = []
        for (x1, y1, x2, y2, s), f in zip(dets, faces):
            out.append({'b': [round(float(v), 4) for v in (x1 / W, y1 / H, x2 / W, y2 / H)], 's': round(float(s), 2),
                        'f': None if f is None else [round(float(v), 4) for v in (f[0] / w, f[1] / h, f[2] / w, f[3] / h)],
                        'fs': None if f is None else round(float(f[-1]), 2),
                        'front': None if f is None else is_frontal(f)})
        return {'t': time.time(), 'boxes': out}

    # ---------------- 相机-雷达外参标定 ----------------
    def _extcal_sample(self, msg, dets, K, D, width):
        """一帧只有一个人时，记 (框中心 u, 相机方位, 雷达方位)，都在 base_frame 下"""
        ec = self.extcal
        if len(dets) != 1:
            ec['skip']['n_det'] += 1
            return
        raw = self.scan_raw
        if raw is None or abs(raw[0] - stamp_sec(msg.header.stamp)) > 0.15:
            ec['skip']['sync'] += 1
            return
        T_cam = self.lookup(msg.header.frame_id, target=self.base_frame)
        T_las = self.lookup(raw[1], target=self.base_frame)
        if T_cam is None or T_las is None:
            return
        x1, y1, x2, y2, _s = dets[0]
        uc, v = (x1 + x2) / 2, (y1 + y2) / 2
        b = ray_bearings(pixel_rays([[uc, v], [K[0, 2], K[1, 2]]], K, D), T_cam[0])
        pts = raw[2] @ T_las[0][:2, :2].T + T_las[1][:2]
        m = extcal.match_lidar(pts, raw[3], T_cam[1][:2], b[0], search=self.extcal_search,
                               min_width=self.min_width, max_width=self.max_width)
        if m is None:
            ec['skip']['no_match'] += 1
            return
        if len(ec['samples']) < 1000:
            ec['samples'].append((float(uc), float(b[0]), m[0]))
            ec['rel'].append(wrap(m[0] - b[1]))      # 相对当前光轴的方位，左正右负
            ec['axis'], ec['fx'], ec['width'] = float(b[1]), float(K[0, 0]), float(width)

    def srv_extcal_start(self, _req, res):
        self.extcal.update(on=True, samples=[], rel=[], skip={'n_det': 0, 'sync': 0, 'no_match': 0}, result=None, saved='')
        res.success, res.message = True, '开始采集：画面里只站一个人，在 1~5m 处左右来回慢慢走，机器人别动'
        return res

    def srv_extcal_solve(self, _req, res):
        ec = self.extcal
        ec['on'] = False
        if not ec['samples']:
            res.success, res.message = False, '没有样本'
            return res
        r = extcal.solve(list(ec['samples']), ec['axis'], ec['width'], fx_now=ec['fx'])
        ec['result'] = r
        res.success = r['ok']
        res.message = (f'yaw {r["yaw_deg"]:+.2f}°（修正 {r["delta_deg"]:+.2f}°），残差 {r["rms_deg"]:.2f}°，'
                       f'{r["inliers"]}/{r["n"]} 个样本' + (f'；{r["message"]}' if r['message'] else '')
                       if r['ok'] else r['message'])
        self.get_logger().info(f'外参标定: {res.message}')
        return res

    def srv_extcal_save(self, _req, res):
        r = self.extcal['result']
        if not r or not r['ok']:
            res.success, res.message = False, '还没有有效的标定结果'
            return res
        T = self.lookup('camera_link', target='base_link')
        xyz = [round(float(v), 3) for v in T[1]] if T is not None else None
        os.makedirs(os.path.dirname(self.extrinsic_file), exist_ok=True)
        with open(self.extrinsic_file, 'w', encoding='utf-8') as f:
            f.write('# 相机-雷达联合标定结果（person_tracker /person/extcal_*）。robot.launch.py 启动时读 yaw\n'
                    f'# {datetime.now():%Y-%m-%d %H:%M:%S}，{r["inliers"]}/{r["n"]} 个样本，残差 {r["rms_deg"]:.2f}°，'
                    f'方位覆盖 {r["spread_deg"]:.0f}°，针孔粗估 fx={r["fx_pinhole"]}\n'
                    f'base_link_to_camera_link:\n'
                    f'  yaw: {r["yaw"]:.5f}        # rad（{r["yaw_deg"]:+.2f}°），左正\n'
                    f'  xyz_at_calib: {xyz}   # 标定时 TF 里的平移，只做记录，平移仍以 launch 为准\n')
        self.extcal['saved'] = self.extrinsic_file
        res.success, res.message = True, f'已保存到 {self.extrinsic_file}，重启 base（robot.launch.py）后生效'
        self.get_logger().info(res.message)
        return res

    def _extcal_state(self):
        ec = self.extcal
        return {'on': ec['on'], 'n': len(ec['samples']), 'skip': ec['skip'], 'coverage': extcal.coverage(ec['rel']),
                'result': None if ec['result'] is None else
                {k: (round(v, 3) if isinstance(v, float) else v) for k, v in ec['result'].items()},
                'saved': ec['saved']}

    # ---------------- 走到人面前 ----------------
    def on_goal(self, _req):
        if self.busy:
            self.get_logger().warn('已有 approach 在执行，拒绝新的')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def srv_start(self, _req, res):
        if self.busy:
            res.success, res.message = False, '已有 approach 在执行'
            return res
        done = threading.Event()
        fut = self.self_client.send_goal_async(ApproachPerson.Goal())
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(3.0) or not fut.result().accepted:
            res.success, res.message = False, '发送失败'
            return res
        self.self_handle = fut.result()
        res.success = True
        res.message = (f'开始: 距离 {self.get_parameter("approach_distance").value:.1f}m，'
                       f'face_front={self.get_parameter("face_front").value}')
        return res

    def srv_cancel(self, _req, res):
        if not self.busy or self.self_handle is None:
            res.success, res.message = (False, '没有在执行（命令行发起的请用 Ctrl-C 取消）') if self.busy else (False, '没有在执行')
            return res
        self.self_handle.cancel_goal_async()
        res.success, res.message = True, '已请求取消'
        return res

    def _facing(self, person_xy, robot_xy):
        """人面朝的方向（单位向量）。现在只有人体框，假设被识别时人面朝机器人"""
        v = np.asarray(robot_xy) - np.asarray(person_xy)
        return v / max(float(np.hypot(*v)), 1e-6)

    def _pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.track_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def _send_nav(self, g):
        """发 NavigateToPose，返回 (goal_handle, result_future) 或 None。新 goal 会抢占 nav2 里的旧 goal"""
        done = threading.Event()
        fut = self.nav.send_goal_async(NavigateToPose.Goal(pose=self._pose(*g)))
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0) or not fut.result().accepted:
            return None
        h = fut.result()
        self.nav_goal = g
        return h, h.get_result_async()

    def execute(self, gh):
        try:
            return self._execute(gh)
        finally:
            with self.lock:
                self.tracker.release()
            if self.mark is not None:
                self.mark['active'] = False
            self.nav_goal, self.self_handle, self.busy = None, None, False

    def _publish_arrived(self, tr, person_xy):
        """到达人面前：发 /person/arrived，语音播报等上层订阅这个话题"""
        robot = self.robot_xy()
        p = np.asarray(person_xy, float)
        msg = {'target_id': tr.id, 'person': [round(float(v), 3) for v in p],
               'robot': None if robot is None else [round(float(v), 3) for v in robot],
               'distance': None if robot is None else round(float(np.hypot(*(p - robot))), 3),
               'frame': self.track_frame, 'stamp': time.time()}
        self.pub_arrived.publish(String(data=json.dumps(msg)))

    def _canceled(self, gh, res):
        gh.canceled()
        res.message = '已取消'
        self.approach.update(state='canceled', message='已取消')
        self.get_logger().info('approach 已取消')
        return res

    def _execute(self, gh):
        req = gh.request
        dist = req.distance if req.distance > 0 else self.get_parameter('approach_distance').value
        search_to = req.search_timeout if req.search_timeout > 0 else self.get_parameter('search_timeout').value
        face_front = self.get_parameter('face_front').value
        face_within = self.face_within if self.get_parameter('require_face').value else None
        t0 = time.time()
        res = ApproachPerson.Result()
        fb = ApproachPerson.Feedback()
        self.mark = None
        self.approach = {'state': 'searching', 'target_id': -1, 'distance': -1.0, 'message': ''}

        def finish(ok, text, tr=None, robot=None):
            res.success, res.message = ok, text
            if tr is not None:
                res.person = self._pose(tr.xy[0], tr.xy[1], 0.0)
                if robot is not None:
                    res.distance = float(np.hypot(*(tr.xy - robot)))
            self.approach.update(state='succeeded' if ok else 'failed', message=text,
                                 distance=float(res.distance) if tr is not None and robot is not None else -1.0)
            self.get_logger().info(f'approach 结束: {text}')
            (gh.succeed if ok else gh.abort)()
            return res

        def feedback(state, tr, robot):
            fb.state = state
            fb.target_id = tr.id if tr is not None else -1
            if tr is not None:
                fb.person = self._pose(tr.xy[0], tr.xy[1], 0.0)
                fb.distance = float(np.hypot(*(tr.xy - robot))) if robot is not None else -1.0
            gh.publish_feedback(fb)
            self.approach.update(state=state, target_id=fb.target_id, distance=float(fb.distance) if tr is not None else -1.0,
                                 message='')

        if not self.nav.wait_for_server(timeout_sec=3.0):
            return finish(False, 'nav2 不在线（/navigate_to_pose）')

        # 1. 找人并锁定
        tr = robot = None
        while tr is None:
            if gh.is_cancel_requested:
                return self._canceled(gh, res)
            if time.time() - t0 > search_to:
                return finish(False, f'{search_to:.0f}s 内没看到{"人脸" if face_within else "人"}')
            robot = self.robot_xy()
            if robot is not None:
                with self.lock:
                    tr, _ = self.tracker.target(robot, self.get_clock().now().nanoseconds * 1e-9, face_within)
            feedback('searching', tr, robot)
            time.sleep(0.1)
        self.get_logger().info(f'锁定 #{tr.id}，距离 {np.hypot(*(tr.xy - robot)):.2f}m，face_front={face_front}')

        # 2. face_front：用前 mark_frames 次相机观测粗匹配位置，记下目标位姿
        if face_front:
            xs, last_cam = [], -1.0
            while len(xs) < self.mark_frames:
                if gh.is_cancel_requested:
                    return self._canceled(gh, res)
                if time.time() - t0 > search_to + 3.0:
                    break
                with self.lock:
                    cur = self.tracker.get(tr.id)
                    if cur is not None and cur.t_cam != last_cam:
                        last_cam = cur.t_cam
                        xs.append(cur.xy)
                if cur is None:
                    break
                time.sleep(0.05)
            if not xs:
                return finish(False, '锁定后马上丢失，没能标记位置')
            person = np.mean(xs, 0)
            robot = self.robot_xy()
            g_xy = person + self._facing(person, robot) * dist
            yaw = math.atan2(person[1] - g_xy[1], person[0] - g_xy[0])
            if float(np.hypot(*(person - robot))) <= dist:      # 已经够近了：不后退，原地转向人
                g_xy, yaw = robot, math.atan2(person[1] - robot[1], person[0] - robot[0])
            goal, goal_person = (float(g_xy[0]), float(g_xy[1]), yaw), person
            self.mark = {'xy': (float(person[0]), float(person[1])), 'frames': len(xs), 'face_front': True, 'active': True}
            self.get_logger().info(f'标记人位置 ({person[0]:.2f},{person[1]:.2f})（{len(xs)} 帧），'
                                   f'目标 ({goal[0]:.2f},{goal[1]:.2f})，记住到导航结束')
        else:
            goal_person = tr.xy
            goal = approach_goal(goal_person, robot, dist)
            self.mark = {'xy': (float(goal_person[0]), float(goal_person[1])), 'frames': 1, 'face_front': False,
                         'active': True}

        # 3. 导航，跟踪模式下人移动就更新目标
        sent = self._send_nav(goal)
        if sent is None:
            return finish(False, 'nav2 拒绝了目标点')
        handle, result = sent
        last_send, state = time.time(), 'approaching'
        while True:
            if gh.is_cancel_requested:
                handle.cancel_goal_async()
                return self._canceled(gh, res)
            if req.timeout > 0 and time.time() - t0 > req.timeout:
                handle.cancel_goal_async()
                return finish(False, f'超时 {req.timeout:.0f}s', tr, robot)
            robot = self.robot_xy()
            now = self.get_clock().now().nanoseconds * 1e-9
            with self.lock:
                cur, changed = self.tracker.target(robot, now, face_within) if robot is not None else (None, False)
            if cur is None:
                state = 'lost'          # 继续开往最后的目标点
            else:
                if changed:
                    self.get_logger().info(f'目标丢失，改为跟踪最近的 #{cur.id}')
                tr, state = cur, 'approaching'
                if (not face_front and time.time() - last_send > 1.0
                        and float(np.hypot(*(tr.xy - goal_person))) > self.replan_dist):
                    goal_person = tr.xy
                    goal = approach_goal(goal_person, robot, dist)
                    self.mark.update(xy=(float(goal_person[0]), float(goal_person[1])), frames=self.mark['frames'] + 1)
                    sent = self._send_nav(goal)
                    if sent is not None:
                        handle, result = sent
                        last_send = time.time()
            feedback(state, tr, robot)
            if result.done():
                status = result.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    feedback('arrived', tr, robot)
                    self._publish_arrived(tr, goal_person)
                    note = '（途中目标丢失，停在最后的目标点）' if state == 'lost' else ''
                    return finish(True, f'已到达人前 {dist:.1f}m{note}', tr, robot)
                return finish(False, f'导航失败，状态 {status}', tr, robot)
            time.sleep(0.2)

    # ---------------- 可视化 ----------------
    def publish_vis(self):
        try:
            self._publish_vis()
        except Exception as e:   # noqa: BLE001  显示出错不能把节点带崩
            self.get_logger().error(f'可视化出错: {e}', throttle_duration_sec=5.0)

    def _publish_vis(self):
        with self.lock:
            tracks = [(tr.id, tr.xy, tr.id == self.tracker.target_id) for tr in self.tracker.tracks]
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = self.track_frame
        arr.markers.append(clear)

        def mk(ns, mid, mtype, x, y, rgba, scale, text='', yaw=0.0):
            m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD, text=text)
            m.header.frame_id, m.header.stamp = self.track_frame, stamp
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            return m

        for tid, xy, is_target in tracks:
            rgba = (0.1, 0.9, 0.2, 0.9) if is_target else (0.9, 0.6, 0.1, 0.8)
            arr.markers.append(mk('person', tid, Marker.CYLINDER, xy[0], xy[1], rgba, (0.4, 0.4, 1.7)))
            t = mk('person_id', tid, Marker.TEXT_VIEW_FACING, xy[0], xy[1], (1, 1, 1, 1), (0, 0, 0.3), f'#{tid}')
            t.pose.position.z = 2.0
            arr.markers.append(t)
            if is_target:
                self.pub_target.publish(self._pose(xy[0], xy[1], 0.0))
        mark, goal = self.mark, self.nav_goal
        if mark is not None:
            a = 1.0 if mark['active'] else 0.4
            arr.markers.append(mk('mark', 0, Marker.SPHERE, *mark['xy'], (1.0, 0.1, 0.1, a), (0.3, 0.3, 0.3)))
        if goal is not None:
            arr.markers.append(mk('goal', 0, Marker.ARROW, goal[0], goal[1], (1.0, 0.3, 0.8, 0.9), (0.6, 0.1, 0.1),
                                  yaw=goal[2]))
        self.pub_markers.publish(arr)

        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            trs = [{'id': tr.id, 'x': round(float(tr.x[0]), 3), 'y': round(float(tr.x[1]), 3),
                    'speed': round(tr.speed, 2), 'cam_age': round(now - tr.t_cam, 1),
                    'face_age': round(now - tr.t_face, 1) if tr.t_face > -math.inf else None,
                    'target': tr.id == self.tracker.target_id} for tr in self.tracker.tracks]
        st = {k: (round(v, 1) if isinstance(v, float) else v) for k, v in self.det_stat.items() if k != 't'}
        state = {
            'frame': self.track_frame, 'tracks': trs,
            'mark': None if mark is None else {**mark, 'xy': [round(v, 3) for v in mark['xy']]},
            'goal': None if goal is None else [round(float(v), 3) for v in goal],
            'approach': self.approach, 'busy': self.busy, 'det': st,
            'params': {k: self.get_parameter(k).value for k in ('face_front', 'require_face', 'approach_distance', 'search_timeout')},
            'face_model': self.face is not None,
            'camera_info': self.info is not None,
            'extcal': self._extcal_state(),
            # 检测框（1s 内的才发，节点卡住时网页上不会一直留着旧框）
            'boxes': self.det_boxes['boxes'] if time.time() - self.det_boxes['t'] < 1.0 else [],
        }
        self.pub_state.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def destroy_node(self):
        self.running = False
        self.det_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = PersonTracker()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
