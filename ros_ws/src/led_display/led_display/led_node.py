#!/usr/bin/env python3
"""LED 软屏 ROS 节点：保持 BLE 长连接，断了自动重连。

订阅:
    /led/text        std_msgs/String  要显示的文字；也可以发 JSON {"text": "...", "color": [r, g, b]}
    /led/gif         std_msgs/String  机器人上 GIF / 图片文件路径（自动缩放成 32x16）
    /led/brightness  std_msgs/UInt8   1..100
    /led/power       std_msgs/Bool
    /led/eyes        std_msgs/Empty   切回待机眼睛动画（ledctl.eyes_gif）
发布:
    /led/state       std_msgs/String  JSON {"connected", "showing", "error"}，有变化时发，transient local

文字/图片上传要 1~3s，期间收到多条只发最新一条。
参数 idle=eyes 时，连上屏后先显示眼睛（断线重连后屏上内容还在，不重发）。

    ros2 run led_display led_node --ros-args --params-file config/led.yaml
"""
import asyncio
import json
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String, UInt8

from led_display.ledctl import ADDRESS, LedPanel, eyes_gif, prepare_gif, text_gif

STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LedNode(Node):
    def __init__(self):
        super().__init__("led_display")
        self.address = self.declare_parameter("address", ADDRESS).value
        self.font_path = self.declare_parameter("font_path", "").value or None
        self.color = list(self.declare_parameter("color", [255, 0, 0]).value)
        self.brightness = self.declare_parameter("brightness", 0).value  # 0 = 连上后不改亮度
        self.reconnect_s = self.declare_parameter("reconnect_interval", 5.0).value
        self.verbose = self.declare_parameter("verbose", False).value
        self.idle = self.declare_parameter("idle", "eyes").value            # eyes / ""（不管）
        self.eyes_color = tuple(self.declare_parameter("eyes_color", [0, 160, 255]).value)

        self.state_pub = self.create_publisher(String, "/led/state", STATE_QOS)
        self.create_subscription(String, "/led/text", self._on_text, 10)
        self.create_subscription(String, "/led/gif", self._on_gif, 10)
        self.create_subscription(UInt8, "/led/brightness", lambda m: self._post("brightness", m.data), 10)
        self.create_subscription(Bool, "/led/power", lambda m: self._post("power", m.data), 10)
        self.create_subscription(Empty, "/led/eyes", lambda _m: self._post("content", ("eyes", "", self.eyes_color)), 10)

        self.state = {"connected": False, "showing": "", "error": ""}
        self._publish_state()

        # 待办只留每种的最新一条：content（文字/图片二选一）、brightness、power
        self._pending = {}
        self._loop = asyncio.new_event_loop()
        self._wake = asyncio.Event()  # 3.10 起 Event 不在创建时绑 loop，只在 loop 线程里用
        threading.Thread(target=self._loop.run_until_complete, args=(self._worker(),), daemon=True).start()

    # ---- ROS 回调（rclpy 线程）------------------------------------------------

    def _on_text(self, msg):
        text, color = msg.data, self.color
        if text.lstrip().startswith("{"):
            try:
                d = json.loads(text)
                text, color = str(d.get("text", "")), d.get("color", color)
            except ValueError:
                pass
        self._post("content", ("text", text, tuple(int(c) for c in color)))

    def _on_gif(self, msg):
        self._post("content", ("gif", msg.data, None))

    def _post(self, key, value):
        def put():
            self._pending[key] = value
            self._wake.set()
        self._loop.call_soon_threadsafe(put)

    def _publish_state(self, **kw):
        self.state.update(kw)
        self.state_pub.publish(String(data=json.dumps(self.state, ensure_ascii=False)))

    # ---- BLE（asyncio 线程）-------------------------------------------------

    async def _worker(self):
        led = LedPanel(self.address, verbose=self.verbose, log=self.get_logger().info)
        while rclpy.ok():
            if not led.connected:
                try:
                    await led.connect()
                    self._publish_state(connected=True, error="")
                    self.get_logger().info(f"LED 已连接 {self.address}")
                    if self.brightness:
                        self._pending.setdefault("brightness", self.brightness)
                    if self.idle == "eyes" and not self.state["showing"]:
                        self._pending.setdefault("content", ("eyes", "", self.eyes_color))
                except Exception as e:
                    self._publish_state(connected=False, error=f"连接失败: {e}")
                    self.get_logger().warn(f"LED 连接失败，{self.reconnect_s}s 后重试: {e}", throttle_duration_sec=60)
                    self._drop_stale_link()
                    await asyncio.sleep(self.reconnect_s)
                    continue

            if not self._pending:
                self._wake.clear()
                try:
                    await asyncio.wait_for(self._wake.wait(), 1.0)
                except asyncio.TimeoutError:
                    pass  # 定期回来看连接是否还在
                continue

            key = next(iter(self._pending))
            value = self._pending.pop(key)
            try:
                await self._apply(led, key, value)
            except Exception as e:
                self.get_logger().error(f"LED {key} 失败: {e}")
                self._publish_state(connected=led.connected, error=f"{key}: {e}")
                if not led.connected:
                    self._pending.setdefault(key, value)  # 断线导致的，重连后再发一次

    def _drop_stale_link(self):
        """上一个 led_node 被强杀时 BlueZ 里还留着到屏的连接：屏被占着不广播，新连接报 not found。
        只有本节点连这块屏，连不上时 BlueZ 显示 Connected 就是残留的，断掉（10-26 碰到过）"""
        try:
            info = subprocess.run(["bluetoothctl", "info", self.address], capture_output=True, text=True, timeout=5).stdout
            if "Connected: yes" in info:
                subprocess.run(["bluetoothctl", "disconnect", self.address], capture_output=True, timeout=10)
                self.get_logger().info(f"断开了 BlueZ 里残留的连接 {self.address}")
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f"检查残留连接失败: {e}", throttle_duration_sec=60)

    async def _apply(self, led, key, value):
        if key == "brightness":
            await led.brightness(value)
        elif key == "power":
            await led.power(value)
        elif key == "content":
            kind, data, color = value
            if kind == "eyes":
                gif = eyes_gif(color)
            else:
                gif = text_gif(data, color, self.font_path) if kind == "text" else prepare_gif(data)
            await led.send_gif(gif)
            self._publish_state(showing=f"{kind}:{data}" if data else kind, error="")


def main():
    rclpy.init()
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
