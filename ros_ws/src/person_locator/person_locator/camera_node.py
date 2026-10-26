#!/usr/bin/env python3
"""USB 相机节点：MJPG 原始帧直接转发，不解码。

发布:
    /camera/image_raw/compressed   sensor_msgs/CompressedImage  全分辨率 jpeg（相机原始码流）
    /camera/camera_info            sensor_msgs/CameraInfo       标定文件存在时才发，文件更新后自动重载
    /camera/preview/compressed     sensor_msgs/CompressedImage  缩小预览（网页用），有订阅者时才解码
服务:
    /camera/snapshot               std_srvs/Trigger  把最新一帧全分辨率原图存到 capture_dir，message 返回文件名

    ros2 run person_locator camera_node --ros-args --params-file config/camera.yaml
"""
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


def load_camera_info(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    msg = CameraInfo()
    msg.width = int(d['image_width'])
    msg.height = int(d['image_height'])
    msg.distortion_model = d.get('distortion_model', 'plumb_bob')
    msg.k = [float(v) for v in d['camera_matrix']['data']]
    msg.d = [float(v) for v in d['distortion_coefficients']['data']]
    msg.r = [float(v) for v in d['rectification_matrix']['data']]
    msg.p = [float(v) for v in d['projection_matrix']['data']]
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.device = self.declare_parameter('device', '/dev/v4l/by-id/usb-AX_USB2.0_AX_USB2.0-video-index0').value
        self.width = self.declare_parameter('width', 1280).value
        self.height = self.declare_parameter('height', 720).value
        self.fps = self.declare_parameter('fps', 30).value
        # 相机只有 30fps 一档；全分辨率 jpeg 经 rclpy 发 30Hz 要占满一个核（10-26 实测 ~100%），检测只要 4~6Hz。
        # 读帧照常（不读 V4L2 缓冲会积压），只按这个频率发布；0 = 每帧都发
        self.publish_rate = self.declare_parameter('publish_rate', 10.0).value
        self.last_pub = 0.0
        self.frame_id = self.declare_parameter('frame_id', 'camera_optical_frame').value
        self.info_file = os.path.expanduser(
            self.declare_parameter('camera_info_file', '~/max_robot/calib/camera_info.yaml').value)
        # 预览：jpeg 按 1/2 缩小解码（libjpeg DCT 域缩放，比全尺寸解码再 resize 快很多）
        self.preview_fps = self.declare_parameter('preview_fps', 10.0).value
        self.preview_quality = self.declare_parameter('preview_quality', 70).value
        self.capture_dir = os.path.expanduser(
            self.declare_parameter('capture_dir', '~/max_robot/captures').value)

        self.pub_raw = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', SENSOR_QOS)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', SENSOR_QOS)
        self.pub_preview = self.create_publisher(CompressedImage, '/camera/preview/compressed', SENSOR_QOS)
        self.create_service(Trigger, '/camera/snapshot', self.srv_snapshot)
        self.latest = None          # (time, jpeg bytes)，拍照用

        self.info = None
        self.info_mtime = None
        self.last_info_check = 0.0
        self.last_preview = 0.0
        self.cap = None
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # 不解码，read() 拿到的是 jpeg 码流
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'相机 {self.device} -> {os.path.realpath(self.device)}: {w}x{h}')
        if (w, h) != (self.width, self.height):
            self.get_logger().warn(f'请求 {self.width}x{self.height}，实际 {w}x{h}')
        return cap

    def check_info(self):
        """每 2s 看一次标定文件 mtime，标定完不用重启节点"""
        now = time.time()
        if now - self.last_info_check < 2.0:
            return
        self.last_info_check = now
        try:
            mtime = os.path.getmtime(self.info_file)
        except OSError:
            if self.info is not None:
                self.get_logger().warn(f'标定文件消失: {self.info_file}')
            self.info, self.info_mtime = None, None
            return
        if mtime == self.info_mtime:
            return
        try:
            info = load_camera_info(self.info_file)
        except Exception as e:  # noqa: BLE001  文件写到一半之类，下次再读
            self.get_logger().warn(f'读取标定文件失败: {e}')
            return
        if (info.width, info.height) != (self.width, self.height):
            self.get_logger().error(f'标定分辨率 {info.width}x{info.height} 与相机 {self.width}x{self.height} 不符，不发布 camera_info')
            info = None
        else:
            self.get_logger().info(f'已加载标定 {self.info_file}  fx={info.k[0]:.1f} fy={info.k[4]:.1f}')
        self.info, self.info_mtime = info, mtime

    def loop(self):
        fails = 0
        while self.running and rclpy.ok():
            if self.cap is None:
                self.cap = self.open()
                if self.cap is None:
                    self.get_logger().error(f'无法打开 {self.device}，2s 后重试', throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            ok, buf = self.cap.read()
            if not ok or buf is None or buf.size < 4 or buf.ravel()[0] != 0xFF or buf.ravel()[1] != 0xD8:
                fails += 1
                if fails > 30:  # 约 1s 读不到：多半是 USB 掉了，重开
                    self.get_logger().warn('读帧失败，重新打开相机')
                    self.cap.release()
                    self.cap = None
                    fails = 0
                time.sleep(0.03)
                continue
            fails = 0
            now = time.time()
            if self.publish_rate > 0 and now - self.last_pub < 1.0 / self.publish_rate:
                continue
            self.last_pub = now
            self.publish(buf.tobytes())

    def publish(self, jpeg):
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg
        self.pub_raw.publish(msg)
        self.latest = (time.time(), jpeg)

        self.check_info()
        if self.info is not None:
            self.info.header = msg.header
            self.pub_info.publish(self.info)

        now = time.time()
        if self.pub_preview.get_subscription_count() == 0 or now - self.last_preview < 1.0 / self.preview_fps:
            return
        self.last_preview = now
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if img is None:
            return
        ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(self.preview_quality)])
        if ok:
            pm = CompressedImage()
            pm.header = msg.header
            pm.format = 'jpeg'
            pm.data = enc.tobytes()
            self.pub_preview.publish(pm)

    def srv_snapshot(self, _req, res):
        latest = self.latest
        if latest is None or time.time() - latest[0] > 1.0:
            res.success, res.message = False, '相机没有图像'
            return res
        t, jpeg = latest
        name = time.strftime('%Y%m%d_%H%M%S', time.localtime(t)) + f'_{int(t * 1000) % 1000:03d}.jpg'
        try:
            os.makedirs(self.capture_dir, exist_ok=True)
            with open(os.path.join(self.capture_dir, name), 'wb') as f:
                f.write(jpeg)
        except OSError as e:
            res.success, res.message = False, f'保存失败: {e}'
            return res
        res.success, res.message = True, name
        return res

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
