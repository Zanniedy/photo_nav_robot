#!/usr/bin/env python3
"""在线标定节点：订阅相机原图找棋盘，网页上看叠加画面、点按钮采集/标定/保存。

板端没显示器，所以把 calibrate.py 的交互流程搬到 ROS 上，网页经 rosbridge 操作。

订阅  /camera/image_raw/compressed
发布  /calib/preview/compressed   叠加角点和已覆盖区域的预览（1/2 分辨率）
      /calib/status               std_msgs/String，JSON 状态，2Hz
服务  /calib/capture  /calib/calibrate  /calib/save  /calib/reset   std_srvs/Trigger
      /calib/auto                                                   std_srvs/SetBool

标定结果写到 output_file（camera_node 默认从这里读，会自动重载），
采集的原图同时存在 output_file 同目录 images/ 下，可用 calibrate --images 离线重标。
"""
import json
import os
import shutil
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from person_locator.calibrate import (board_pose_signature, calibrate, check_pose_diversity,
                                      find_corners, is_new_pose, write_camera_info)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class CalibNode(Node):
    def __init__(self):
        super().__init__('calib_node')
        board = self.declare_parameter('board', '8x11').value
        self.board = tuple(int(v) for v in board.lower().split('x'))
        self.square = self.declare_parameter('square', 0.025).value
        self.min_frames = self.declare_parameter('min_frames', 20).value
        self.camera_name = self.declare_parameter('camera_name', 'front_camera').value
        self.out = os.path.expanduser(
            self.declare_parameter('output_file', '~/max_robot/calib/camera_info.yaml').value)
        self.img_dir = os.path.join(os.path.dirname(self.out), 'images')

        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_image, SENSOR_QOS)
        self.pub_preview = self.create_publisher(CompressedImage, '/calib/preview/compressed', SENSOR_QOS)
        self.pub_status = self.create_publisher(String, '/calib/status', 10)
        self.create_service(Trigger, '/calib/capture', self.srv_capture)
        self.create_service(Trigger, '/calib/calibrate', self.srv_calibrate)
        self.create_service(Trigger, '/calib/save', self.srv_save)
        self.create_service(Trigger, '/calib/reset', self.srv_reset)
        self.create_service(SetBool, '/calib/auto', self.srv_auto)
        self.create_timer(0.5, self.publish_status)

        self.lock = threading.Lock()
        self.latest = None          # 最新一帧 CompressedImage，处理线程只取最新的
        self.event = threading.Event()
        self.reset_state()
        self.running = True
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'棋盘 {self.board[0]}x{self.board[1]} 内角点，方格 {self.square * 1000:.0f}mm，输出 {self.out}')

    def reset_state(self):
        self.img_pts, self.sigs, self.frames = [], [], []
        self.coverage = None
        self.size = None
        self.auto = False
        self.last_auto = 0.0
        self.prev = None
        self.cur = None             # 当前帧 (gray 原图, corners, frame_jpeg, still, new)
        self.result = None          # 标定结果 dict（未保存）
        self.busy = ''
        self.message = '等待相机图像'
        self.fps = 0.0

    # ---------- 图像处理 ----------
    def on_image(self, msg):
        with self.lock:
            self.latest = msg
        self.event.set()

    def worker(self):
        t_last = time.time()
        while self.running and rclpy.ok():
            if not self.event.wait(timeout=1.0):
                continue
            self.event.clear()
            with self.lock:
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            jpeg = bytes(msg.data)
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            self.process(frame, jpeg)
            now = time.time()
            self.fps = 0.8 * self.fps + 0.2 / max(now - t_last, 1e-3)
            t_last = now

    def process(self, frame, jpeg):
        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_corners(gray, self.board)
        with self.lock:
            if self.size is not None and size != self.size:
                self.message = f'分辨率变了 {self.size} → {size}，已清空采集'
                self.img_pts, self.sigs, self.frames, self.coverage = [], [], [], None
            self.size = size
            if self.coverage is None:
                self.coverage = np.zeros((size[1] // 2, size[0] // 2, 3), np.uint8)
            still = new = False
            if corners is not None:
                # 相邻两帧角点平均位移 < 1.5px 视为静止
                still = self.prev is not None and float(np.mean(np.linalg.norm(corners - self.prev, axis=2))) < 1.5
                new = is_new_pose(board_pose_signature(corners, size), self.sigs)
            self.prev = corners
            self.cur = (corners, jpeg, still, new)
            if self.auto and corners is not None and new and still and time.time() - self.last_auto > 1.0:
                self._take()
            self.draw(frame, corners, still, new)

    def draw(self, frame, corners, still, new):
        if self.pub_preview.get_subscription_count() == 0:
            return
        small = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2), interpolation=cv2.INTER_AREA)
        disp = cv2.addWeighted(small, 1.0, self.coverage, 0.6, 0)
        if corners is not None:
            cv2.drawChessboardCorners(disp, self.board, corners * 0.5, True)
            hint = ('NEW POSE' if new else 'similar') + ('' if still else ' (hold still)')
            cv2.putText(disp, hint, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if new and still else (0, 200, 255), 2)
        mode = 'AUTO' if self.auto else 'MANUAL'
        cv2.putText(disp, f'[{mode}] {len(self.img_pts)}/{self.min_frames}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0) if corners is not None else (0, 0, 255), 2)
        ok, enc = cv2.imencode('.jpg', disp, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.format = 'jpeg'
            m.data = enc.tobytes()
            self.pub_preview.publish(m)

    def _take(self):
        """调用方持有 self.lock"""
        corners, jpeg, still, _ = self.cur
        self.img_pts.append(corners)
        self.sigs.append(board_pose_signature(corners, self.size))
        self.frames.append(jpeg)
        self.last_auto = time.time()
        hull = cv2.convexHull((corners.reshape(-1, 2) * 0.5).astype(np.int32))
        cv2.fillConvexPoly(self.coverage, hull, (80, 0, 0))
        self.message = f'采集第 {len(self.img_pts)} 帧' + ('' if still else '（棋盘在动，可能模糊）')
        self.get_logger().info(self.message)
        if self.auto and len(self.img_pts) >= self.min_frames:
            self.auto = False
            self.message = f'已采够 {len(self.img_pts)} 帧，自动采集停止，可以点「标定」'

    # ---------- 服务 ----------
    def srv_capture(self, _req, res):
        with self.lock:
            if self.cur is None or self.cur[0] is None:
                res.success, res.message = False, '当前画面没检测到棋盘'
            else:
                self._take()
                res.success, res.message = True, self.message
        return res

    def srv_auto(self, req, res):
        with self.lock:
            self.auto = bool(req.data)
            self.message = '自动采集开' if self.auto else '自动采集关'
        res.success, res.message = True, self.message
        return res

    def srv_reset(self, _req, res):
        with self.lock:
            if self.busy:
                res.success, res.message = False, f'{self.busy}中，稍后再试'
                return res
            self.reset_state()
            self.message = '已清空'
        res.success, res.message = True, '已清空'
        return res

    def srv_calibrate(self, _req, res):
        with self.lock:
            if self.busy:
                res.success, res.message = False, f'{self.busy}中'
                return res
            if len(self.img_pts) < 10:
                res.success, res.message = False, f'只有 {len(self.img_pts)} 帧，至少 10 帧（建议 20+）'
                return res
            self.busy = '标定'
            self.message = f'用 {len(self.img_pts)} 帧标定中…'
            pts, size = list(self.img_pts), self.size
        # calibrateCamera 在 ARM 上要几秒，放后台线程，结果经 /calib/status 返回
        threading.Thread(target=self._calibrate, args=(pts, size), daemon=True).start()
        res.success, res.message = True, '开始标定，结果见状态'
        return res

    def _calibrate(self, pts, size):
        try:
            rms, K, D, keep, rvecs, tvecs = calibrate(pts, self.board, self.square, size, cv2.CALIB_FIX_K3)
            diverse, msgs = check_pose_diversity(rvecs, tvecs)
            w, h = size
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            warns = [m for m in msgs if m.startswith('警告')]
            if rms > 1.0:
                warns.append(f'警告: RMS {rms:.3f}px 偏大（一般 < 0.5px），检查方格尺寸、棋盘平整度、运动模糊')
            if abs(cx - w / 2) > 0.1 * w or abs(cy - h / 2) > 0.1 * h:
                warns.append('警告: 主点偏离图像中心超过 10%，通常是缺倾斜和边角姿态')
            result = {
                'rms': float(rms), 'frames': len(keep), 'dropped': len(pts) - len(keep),
                'fx': float(fx), 'fy': float(fy), 'cx': float(cx), 'cy': float(cy),
                'D': [float(v) for v in D.flatten()[:5]],
                'hfov': float(np.degrees(2 * np.arctan(w / (2 * fx)))),
                'vfov': float(np.degrees(2 * np.arctan(h / (2 * fy)))),
                'summary': msgs[0], 'warnings': warns, 'ok': diverse and rms <= 1.0,
                'saved': False,
            }
            with self.lock:
                self.result = result
                self._K, self._D, self._size, self._keep = K, D, size, keep
                self.message = f'标定完成 RMS={rms:.3f}px' + ('' if result['ok'] else '，有警告')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'标定失败: {e}')
            with self.lock:
                self.message = f'标定失败: {e}'
        finally:
            with self.lock:
                self.busy = ''
        self.get_logger().info(self.message)

    def srv_save(self, _req, res):
        with self.lock:
            if self.result is None:
                res.success, res.message = False, '还没有标定结果'
                return res
            K, D, size, keep = self._K, self._D, self._size, self._keep
            frames = [self.frames[i] for i in keep]
            rms = self.result['rms']
        try:
            if os.path.exists(self.out):
                shutil.copy2(self.out, self.out + '.bak')
            # 先写临时文件再 rename，camera_node 不会读到半个文件
            tmp = self.out + '.tmp'
            write_camera_info(tmp, self.camera_name, size, K, D, rms, len(keep))
            os.replace(tmp, self.out)
            shutil.rmtree(self.img_dir, ignore_errors=True)
            os.makedirs(self.img_dir, exist_ok=True)
            for i, jpeg in enumerate(frames):
                with open(os.path.join(self.img_dir, f'calib_{i + 1:03d}.jpg'), 'wb') as f:
                    f.write(jpeg)
        except OSError as e:
            res.success, res.message = False, f'写入失败: {e}'
            return res
        with self.lock:
            self.result['saved'] = True
            self.message = f'已保存 {self.out}（旧文件备份为 .bak），原图 {len(frames)} 张'
        res.success, res.message = True, self.message
        return res

    def publish_status(self):
        with self.lock:
            cur = self.cur
            st = {
                'captured': len(self.img_pts), 'min_frames': self.min_frames, 'auto': self.auto,
                'detected': cur is not None and cur[0] is not None,
                'still': bool(cur and cur[2]), 'new_pose': bool(cur and cur[3]),
                'size': self.size, 'fps': round(self.fps, 1), 'busy': self.busy, 'message': self.message,
                'board': f'{self.board[0]}x{self.board[1]}', 'square_mm': self.square * 1000,
                'output': self.out, 'result': self.result,
            }
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))


def main():
    rclpy.init()
    node = CalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
