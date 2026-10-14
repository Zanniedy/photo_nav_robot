#!/usr/bin/env python3
"""
relocalize_node — 粗初值 + correlative scan matching 精配，给 AMCL 发准确初始位姿

流程:
  RViz 2D Pose Estimate → /initialpose_rough (粗略点一下即可，误差 ~1m / 30°)
    → 取最新 /scan，在地图距离场上做由粗到细的 (x, y, θ) 穷举搜索
    → 内点率 >= min_inlier_ratio 才发 /initialpose 给 AMCL，否则只告警不发

初值可以在任意 frame（如 RViz Fixed Frame=lio_odom），会经 TF 转到 map。
±yaw_search_range 内匹配失败时，自动退到 360° 全向搜索（应对朝向点反）。

另有服务 ~/relocalize (std_srvs/Trigger)：以当前 /amcl_pose 为初值再精配一次。
精配时机器人应静止。
"""
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


# ── 纯算法部分（无 ROS 依赖，便于离线测试） ─────────────────────────────────────

class MapField:
    """OccupancyGrid → 到最近障碍的距离场（单位 m）"""

    def __init__(self, data, width, height, resolution, origin_x, origin_y, occ_thresh=65):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        free = np.where(grid >= occ_thresh, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5).astype(np.float32) * resolution
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y
        self.w = width
        self.h = height
        # 越界点按"离障碍很远"处理
        self.far = np.float32(10.0)

    def lookup(self, xy):
        """xy: (..., 2) 世界坐标 → 距离 (...)"""
        ix = np.floor((xy[..., 0] - self.ox) / self.res).astype(np.int32)
        iy = np.floor((xy[..., 1] - self.oy) / self.res).astype(np.int32)
        inside = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        d = np.full(ix.shape, self.far, dtype=np.float32)
        d[inside] = self.dist[iy[inside], ix[inside]]
        return d


def _search(field, pts, x0, y0, th0, xy_range, xy_step, th_range, th_step, sigma):
    """在 (x0±xy_range, y0±xy_range, th0±th_range) 网格上打分，返回最优 (x, y, θ, score)"""
    offs = np.arange(-xy_range, xy_range + 1e-9, xy_step, dtype=np.float32)
    dx, dy = np.meshgrid(offs, offs, indexing='ij')
    trans = np.stack([dx.ravel() + x0, dy.ravel() + y0], axis=1)        # (T, 2)
    ths = th0 + np.arange(-th_range, th_range + 1e-9, th_step)
    inv2s2 = -0.5 / (sigma * sigma)

    best = (x0, y0, th0, -1.0)
    for th in ths:
        c, s = math.cos(th), math.sin(th)
        rot = pts @ np.array([[c, s], [-s, c]], dtype=np.float32)       # (N, 2)
        world = trans[:, None, :] + rot[None, :, :]                     # (T, N, 2)
        d = field.lookup(world)
        score = np.exp(d * d * inv2s2).mean(axis=1)                     # (T,)
        i = int(np.argmax(score))
        if score[i] > best[3]:
            best = (float(trans[i, 0]), float(trans[i, 1]), float(th), float(score[i]))
    return best


def match(field, pts, x0, y0, th0, xy_range=1.0, th_range=math.radians(30)):
    """
    由粗到细三层搜索。pts: (N, 2) base 系下的激光点。
    返回 (x, y, θ, inlier_ratio)，inlier = 到最近障碍 < 2 个栅格
    """
    x, y, th, _ = _search(field, pts, x0, y0, th0,
                          xy_range, 0.10, th_range, math.radians(2.0), sigma=0.20)
    x, y, th, _ = _search(field, pts, x, y, th,
                          0.12, 0.03, math.radians(3.0), math.radians(0.5), sigma=0.08)
    x, y, th, _ = _search(field, pts, x, y, th,
                          0.03, 0.01, math.radians(0.6), math.radians(0.15), sigma=0.04)

    c, s = math.cos(th), math.sin(th)
    world = pts @ np.array([[c, s], [-s, c]], dtype=np.float32) + np.array([x, y], dtype=np.float32)
    inlier = float((field.lookup(world) < 2.0 * field.res).mean())
    return x, y, math.atan2(math.sin(th), math.cos(th)), inlier


def scan_to_points(ranges, angle_min, angle_inc, range_min, range_max,
                   tx=0.0, ty=0.0, tyaw=0.0, max_points=360):
    """LaserScan → base 系 (N, 2)，(tx, ty, tyaw) 为 base←laser 外参"""
    r = np.asarray(ranges, dtype=np.float32)
    a = angle_min + angle_inc * np.arange(r.size, dtype=np.float32)
    ok = np.isfinite(r) & (r > max(range_min, 0.05)) & (r < range_max)
    r, a = r[ok], a[ok]
    if r.size > max_points:
        idx = np.linspace(0, r.size - 1, max_points).astype(np.int32)
        r, a = r[idx], a[idx]
    a = a + tyaw
    return np.stack([r * np.cos(a) + tx, r * np.sin(a) + ty], axis=1).astype(np.float32)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ── ROS 节点 ─────────────────────────────────────────────────────────────────

class RelocalizeNode(Node):

    def __init__(self):
        super().__init__('relocalize_node')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('xy_search_range', 1.0)       # m
        self.declare_parameter('yaw_search_range_deg', 30.0)
        self.declare_parameter('min_inlier_ratio', 0.45)
        self.declare_parameter('max_scan_age', 1.0)          # s
        self.declare_parameter('full_yaw_fallback', True)
        self.declare_parameter('pose_topic', '/amcl_pose')   # ~/relocalize 服务用作初值的当前位姿

        self.base_frame = self.get_parameter('base_frame').value
        self.field = None
        self.map_frame = 'map'
        self.scan = None
        self.amcl_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose_rough', self.on_rough, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter('pose_topic').value, self.on_amcl, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_service(Trigger, '~/relocalize', self.on_trigger)

        self.get_logger().info('等待 /map 和 /scan；RViz 2D Pose Estimate 应发到 /initialpose_rough')

    def on_map(self, msg):
        info = msg.info
        self.field = MapField(msg.data, info.width, info.height, info.resolution,
                              info.origin.position.x, info.origin.position.y)
        self.map_frame = msg.header.frame_id or 'map'
        self.get_logger().info(f'地图已加载 {info.width}x{info.height} @ {info.resolution}m')

    def on_scan(self, msg):
        self.scan = msg

    def on_amcl(self, msg):
        self.amcl_pose = msg

    def on_rough(self, msg):
        p = msg.pose.pose
        x, y, th = p.position.x, p.position.y, yaw_of(p.orientation)
        frame = msg.header.frame_id or self.map_frame
        if frame != self.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time(),
                                                     timeout=Duration(seconds=0.5))
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'初值在 {frame}，查不到 {self.map_frame}<-{frame}: {e}')
                return
            t = tf.transform
            tyaw = yaw_of(t.rotation)
            c, s = math.cos(tyaw), math.sin(tyaw)
            x, y = t.translation.x + c * x - s * y, t.translation.y + s * x + c * y
            th += tyaw
            self.get_logger().info(f'初值 frame {frame} → {self.map_frame}')
        self.relocalize(x, y, th)

    def on_trigger(self, _req, resp):
        if self.amcl_pose is None:
            resp.success, resp.message = False, '还没收到当前位姿'
            return resp
        p = self.amcl_pose.pose.pose
        resp.success, resp.message = self.relocalize(p.position.x, p.position.y, yaw_of(p.orientation))
        return resp

    def relocalize(self, x0, y0, th0):
        log = self.get_logger()
        if self.field is None:
            log.warn('还没收到 /map')
            return False, 'no map'
        if self.scan is None:
            log.warn('还没收到 /scan')
            return False, 'no scan'

        scan = self.scan
        age = (self.get_clock().now() - Time.from_msg(scan.header.stamp)).nanoseconds * 1e-9
        if age > self.get_parameter('max_scan_age').value:
            log.warn(f'scan 太旧 ({age:.2f}s)')
            return False, 'stale scan'

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id,
                                                 Time(), timeout=Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001  tf2 异常类型较多，统一兜底
            log.warn(f'查不到 {self.base_frame}<-{scan.header.frame_id}: {e}')
            return False, 'no tf'
        t = tf.transform
        pts = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max,
                             t.translation.x, t.translation.y, yaw_of(t.rotation))
        if len(pts) < 30:
            log.warn(f'有效激光点太少 ({len(pts)})')
            return False, 'too few points'

        t0 = self.get_clock().now()
        x, y, th, inlier = match(self.field, pts, x0, y0, th0,
                                 self.get_parameter('xy_search_range').value,
                                 math.radians(self.get_parameter('yaw_search_range_deg').value))
        min_inlier = self.get_parameter('min_inlier_ratio').value
        if inlier < min_inlier and self.get_parameter('full_yaw_fallback').value:
            log.info(f'±{self.get_parameter("yaw_search_range_deg").value:.0f}° 内内点率 {inlier:.2f}，'
                     f'改 360° 全向搜索…')
            x, y, th, inlier = match(self.field, pts, x0, y0, th0,
                                     self.get_parameter('xy_search_range').value, math.pi)
        cost_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6

        dxy = math.hypot(x - x0, y - y0)
        dth = math.degrees(math.atan2(math.sin(th - th0), math.cos(th - th0)))
        summary = (f'精配 ({x:.2f}, {y:.2f}, {math.degrees(th):.1f}°) '
                   f'修正 {dxy:.2f}m / {dth:+.1f}°  内点率 {inlier:.2f}  耗时 {cost_ms:.0f}ms')

        if inlier < min_inlier:
            log.warn(summary + '  → 低于阈值，不发布，请重新点更准的初值')
            return False, summary

        out = PoseWithCovarianceStamped()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(th / 2.0)
        out.pose.pose.orientation.w = math.cos(th / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(2.0) ** 2
        out.pose.covariance = cov
        self.pub.publish(out)
        log.info(summary + '  → 已发布 /initialpose')
        return True, summary


def main():
    rclpy.init()
    node = RelocalizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
