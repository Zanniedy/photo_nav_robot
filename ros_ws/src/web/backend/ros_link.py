"""
后端进程内的 rclpy 节点，实现 task_runner.NavPort：
  导航  /navigate_to_pose（nav2_msgs/action/NavigateToPose）
  位姿  TF map → base_footprint
  定位  /localizer/score（Float32），3s 没更新算没有
  人员  /person/state（person_tracker 发的 JSON），1s 没更新算没有
  限速  /speed_limit（nav2_msgs/SpeedLimit，controller_server 订阅；0 = 不限速）
  急停  /robot/estop（Bool，transient local，robot_bridge 订阅）
导航是否启动、当前地图由后端进程管理（process_mgr / stack），构造时以函数传进来。
"""
import json, math, threading, time
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String

SCORE_STALE = 3.0
PERSON_STALE = 1.0
_RESULT_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: "到达",
    GoalStatus.STATUS_ABORTED: "导航失败（nav2 中止）",
    GoalStatus.STATUS_CANCELED: "已取消",
}


def person_distance(state: dict, pose) -> float | None:
    """/person/state → 离机器人最近的人的距离（轨迹要在 map 系）"""
    if pose is None or state.get("frame", "map") != "map":
        return None
    ds = [math.hypot(t["x"] - pose[0], t["y"] - pose[1])
          for t in state.get("tracks") or [] if "x" in t and "y" in t]
    return min(ds) if ds else None


class RosLink:
    def __init__(self, nav_running, mapping, current_map_id):
        self.nav_running = nav_running
        self.mapping = mapping
        self.current_map_id = current_map_id
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("web_task_link")
        self._nav = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self.node)
        self._lock = threading.Lock()
        self._score: tuple = (None, 0.0)       # (值, monotonic 时间)
        self._person: tuple = (None, 0.0)
        self._handle = None                    # 当前 goal handle
        self._seq = 0                          # 每发 / 撤一次目标 +1
        self.node.create_subscription(Float32, "/localizer/score", self._on_score, 10)
        self.node.create_subscription(String, "/person/state", self._on_person, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._estop_pub = self.node.create_publisher(Bool, "/robot/estop", latched)
        self._speed_pub = self.node.create_publisher(SpeedLimit, "/speed_limit", 10)
        self._exec = MultiThreadedExecutor(num_threads=2)
        self._exec.add_node(self.node)
        threading.Thread(target=self._exec.spin, daemon=True).start()

    def shutdown(self) -> None:
        self._exec.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_score(self, msg: Float32) -> None:
        self._score = (float(msg.data), time.monotonic())

    def _on_person(self, msg: String) -> None:
        try:
            self._person = (json.loads(msg.data), time.monotonic())
        except ValueError:
            pass

    def loc_score(self) -> float | None:
        v, t = self._score
        return v if v is not None and time.monotonic() - t < SCORE_STALE else None

    def pose(self):
        try:
            tf = self._tf.lookup_transform("map", "base_footprint", Time())
        except tf2_ros.TransformException:
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        return (tr.x, tr.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def nearest_person(self) -> float | None:
        st, t = self._person
        if st is None or time.monotonic() - t > PERSON_STALE:
            return None
        return person_distance(st, self.pose())

    def send_goal(self, x: float, y: float, yaw: float, on_done) -> None:
        """异步发目标，结果经 on_done(ok, msg) 回调（executor 线程）。被 cancel_goal 撤掉的目标也会回调，
        task_runner 自己丢掉过期回调"""
        if not self._nav.server_is_ready():
            on_done(False, "nav2 action server 不在线")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"            # stamp 留 0 = 用最新 TF
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        with self._lock:
            self._seq += 1
            seq = self._seq
            self._handle = None

        def on_result(fut):
            try:
                st = fut.result().status
            except Exception as e:
                on_done(False, f"取导航结果失败: {e}")
                return
            on_done(st == GoalStatus.STATUS_SUCCEEDED, _RESULT_TEXT.get(st, f"导航结束 status={st}"))

        def on_accept(fut):
            try:
                h = fut.result()
            except Exception as e:
                on_done(False, f"发送目标失败: {e}")
                return
            if not h.accepted:
                on_done(False, "nav2 拒绝了目标")
                return
            with self._lock:
                if seq != self._seq:                 # 等接受的时候已经被撤 / 换掉了
                    h.cancel_goal_async()
                    return
                self._handle = h
            h.get_result_async().add_done_callback(on_result)

        self._nav.send_goal_async(goal).add_done_callback(on_accept)

    def cancel_goal(self) -> None:
        with self._lock:
            self._seq += 1
            h, self._handle = self._handle, None
        if h is not None:
            h.cancel_goal_async()

    def set_speed_limit(self, v: float | None) -> None:
        msg = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = float(v) if v else 0.0
        self._speed_pub.publish(msg)

    def set_estop(self, on: bool) -> None:
        self._estop_pub.publish(Bool(data=bool(on)))
