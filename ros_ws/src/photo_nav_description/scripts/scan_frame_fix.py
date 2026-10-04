#!/usr/bin/env python3
"""Republish LaserScan with corrected frame_id for slam_toolbox compatibility."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameFix(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')
        self.declare_parameter('target_frame', 'lidar_link')
        self._frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self._sub = self.create_subscription(LaserScan, '/scan_raw', self._cb, 10)

    def _cb(self, msg: LaserScan):
        msg.header.frame_id = self._frame
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameFix()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
