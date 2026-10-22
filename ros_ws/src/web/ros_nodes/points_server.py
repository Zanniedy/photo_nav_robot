#!/usr/bin/env python3
"""
points_server — 地图命名点位的 ROS 服务，和网页共用同一份存储（backend/annot_store.py）

服务（robot_interfaces/srv）：
  /points/list    ListPoints   列出 / 按子串搜索
  /points/get     GetPoint     按名字或 id 查
  /points/set     SetPoint     按名字新增或修改
  /points/delete  DeletePoint  按名字或 id 删除
  /points/goto    GotoPoint    导航过去：1 个点 NavigateToPose，多个点 FollowWaypoints
                               goal 被 nav2 接受就返回 success，不等到达（到达状态看 action status）

map 字段为空时用当前导航地图（web 后端写在 state_file 的 current_map）。

  ros2 service call /points/goto robot_interfaces/srv/GotoPoint "{names: ['门口']}"
  ros2 service call /points/list robot_interfaces/srv/ListPoints "{query: ''}"
"""
import math, os, sys, threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "backend"))
import annot_store, maps, state   # noqa: E402

import rclpy   # noqa: E402
from rclpy.action import ActionClient   # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup   # noqa: E402
from rclpy.executors import MultiThreadedExecutor   # noqa: E402
from rclpy.node import Node   # noqa: E402
from geometry_msgs.msg import PoseStamped   # noqa: E402
from nav2_msgs.action import FollowWaypoints, NavigateToPose   # noqa: E402
from robot_interfaces.msg import NamedPoint   # noqa: E402
from robot_interfaces.srv import DeletePoint, GetPoint, GotoPoint, ListPoints, SetPoint   # noqa: E402


def to_msg(p: dict) -> NamedPoint:
    return NamedPoint(id=p["id"], name=p["name"], x=float(p["x"]), y=float(p["y"]),
                      yaw=float(p["yaw"]), note=p.get("note", ""))


class PointsServer(Node):
    def __init__(self):
        super().__init__("points_server")
        cb = ReentrantCallbackGroup()
        self.create_service(ListPoints, "/points/list", self.on_list, callback_group=cb)
        self.create_service(GetPoint, "/points/get", self.on_get, callback_group=cb)
        self.create_service(SetPoint, "/points/set", self.on_set, callback_group=cb)
        self.create_service(DeletePoint, "/points/delete", self.on_delete, callback_group=cb)
        self.create_service(GotoPoint, "/points/goto", self.on_goto, callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=cb)
        self.wps = ActionClient(self, FollowWaypoints, "/follow_waypoints", callback_group=cb)
        self.get_logger().info("点位服务就绪: /points/{list,get,set,delete,goto}")

    # map 参数 → 地图 id；空 = 当前导航地图
    def _map(self, name: str) -> tuple[str | None, str]:
        if name:
            m = maps.resolve(name)
            return (m["id"], "") if m else (None, f"地图不存在: {name}")
        cur = state.load().get("current_map")
        return (cur, "") if cur else (None, "没有正在导航的地图，请指定 map")

    def on_list(self, req, res):
        mid, err = self._map(req.map)
        if mid is None:
            res.success, res.message = False, err
            return res
        res.map = mid
        res.points = [to_msg(p) for p in annot_store.list_points(mid, req.query)]
        res.success, res.message = True, f"{len(res.points)} 个点"
        return res

    def on_get(self, req, res):
        mid, err = self._map(req.map)
        p = annot_store.get_point(mid, req.name) if mid else None
        if p is None:
            res.success, res.message = False, err or f"点不存在: {req.name}"
            return res
        res.success, res.point = True, to_msg(p)
        return res

    def on_set(self, req, res):
        mid, err = self._map(req.map)
        if mid is None:
            res.success, res.message = False, err
            return res
        q = req.point
        try:
            p = annot_store.upsert_point(mid, q.name, q.x, q.y, q.yaw, q.note)
        except annot_store.StoreError as e:
            res.success, res.message = False, str(e)
            return res
        res.success, res.message, res.point = True, "已保存", to_msg(p)
        return res

    def on_delete(self, req, res):
        mid, err = self._map(req.map)
        ok = bool(mid) and annot_store.delete_point(mid, req.name)
        res.success = ok
        res.message = "已删除" if ok else (err or f"点不存在: {req.name}")
        return res

    def on_goto(self, req, res):
        mid, err = self._map("")
        if mid is None:
            res.success, res.message = False, "导航未启动"
            return res
        if req.map and (maps.resolve(req.map) or {}).get("id") != mid:
            res.success, res.message = False, "指定的地图不是当前导航地图"
            return res
        if not req.names:
            res.success, res.message = False, "names 为空"
            return res
        pts = []
        for n in req.names:
            p = annot_store.get_point(mid, n)
            if p is None:
                res.success, res.message = False, f"点不存在: {n}"
                return res
            pts.append(p)
        ok, msg = self._send(pts)
        res.success, res.message = ok, msg
        return res

    def _pose(self, p: dict) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(p["x"]), float(p["y"])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(p["yaw"] / 2), math.cos(p["yaw"] / 2)
        return ps

    def _send(self, pts: list[dict]) -> tuple[bool, str]:
        if len(pts) == 1:
            client, goal = self.nav, NavigateToPose.Goal(pose=self._pose(pts[0]))
        else:
            client, goal = self.wps, FollowWaypoints.Goal(poses=[self._pose(p) for p in pts])
        if not client.wait_for_server(timeout_sec=3.0):
            return False, "nav2 action server 不在线"
        done = threading.Event()
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0):
            return False, "发送 goal 超时"
        handle = fut.result()
        if not handle.accepted:
            return False, "nav2 拒绝了 goal"
        names = " → ".join(p["name"] for p in pts)
        self.get_logger().info(f"导航: {names}")
        return True, f"已出发: {names}"


def main():
    rclpy.init()
    node = PointsServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
