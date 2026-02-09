#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import math
import time
import yaml
import os
import numpy as np

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan

# 官方 Python SDK（建議用它避免 proto/版本不合造成伺服端中止 RPC）
import kachaka_api


class KachakaLaserApiNode(Node):
    def __init__(self):
        super().__init__('kachaka_laser_api_node')

        # === CALLBACK GROUPS ===
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.io_callback_group = ReentrantCallbackGroup()

        # --- PARAMETERS（同你模板風格）---
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('use_cursor', True)
        self.declare_parameter('drop_if_no_new', True)
        self.declare_parameter('reconnect_every_fail', 3)   # 每 N 次失敗就重連一次
        self.declare_parameter('log_every_n', 30)           # 每 N 次 publish log 一次

        self.robot_ip = self.get_parameter('kachaka_ip').value
        self.topic_name = self.get_parameter('topic_name').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id_override = str(self.get_parameter('frame_id_override').value)
        self.use_cursor = bool(self.get_parameter('use_cursor').value)
        self.drop_if_no_new = bool(self.get_parameter('drop_if_no_new').value)
        self.reconnect_every_fail = int(self.get_parameter('reconnect_every_fail').value)
        self.log_every_n = int(self.get_parameter('log_every_n').value)

        if self.rate_hz <= 0.0:
            self.rate_hz = 5.0

        # --- PUBLISHER ---
        self.scan_pub = self.create_publisher(LaserScan, self.topic_name, 10)

        # --- STATE ---
        self._client = None
        self._in_flight = False
        self._fail_count = 0
        self._pub_count = 0
        self._last_cursor = None
        self._last_ok_time = time.time()

        # --- CONNECT ---
        self.get_logger().info(f"Connecting to REAL KACHAKA at {self.robot_ip} ...")
        self._connect_client()

        # --- TIMER ---
        self.create_timer(1.0 / self.rate_hz, self.timer_callback, callback_group=self.timer_callback_group)

        self.get_logger().info("✅ Kachaka Laser API Node ready (kachaka_api client).")

    def _connect_client(self):
        """(Re)create KachakaApiClient."""
        try:
            self._client = kachaka_api.KachakaApiClient(self.robot_ip)
            self._fail_count = 0
            self.get_logger().info("✅ KachakaApiClient connected.")
        except Exception as e:
            self._client = None
            self.get_logger().error(f"❌ Failed to create KachakaApiClient: {e}")

    def _extract_scan_obj(self, obj):
        """
        有些版本可能回傳 wrapper response（裡面有 .scan），有些版本直接回 RosLaserScan。
        這邊做兼容。
        """
        if obj is None:
            return None
        return getattr(obj, 'scan', obj)

    def _extract_cursor(self, scan_obj):
        md = getattr(scan_obj, 'metadata', None)
        if md is None:
            return None
        return getattr(md, 'cursor', None)

    def _extract_stamp_and_frame(self, scan_obj):
        """
        盡量使用 scan 內帶的 header.stamp/frame_id；取不到就用 ROS now / fallback frame。
        """
        now = self.get_clock().now().to_msg()

        frame_id = ''
        stamp = now

        hdr = getattr(scan_obj, 'header', None)
        if hdr is not None:
            # frame_id
            frame_id = getattr(hdr, 'frame_id', '') or ''

            # stamp (可能有 sec/nanosec 或 sec/nsec)
            st = getattr(hdr, 'stamp', None)
            if st is not None and hasattr(st, 'sec'):
                sec = int(getattr(st, 'sec', 0))
                nsec = getattr(st, 'nanosec', None)
                if nsec is None:
                    nsec = getattr(st, 'nsec', 0)
                nsec = int(nsec)
                stamp.sec = sec
                stamp.nanosec = nsec

        # override frame_id if requested
        if self.frame_id_override:
            frame_id = self.frame_id_override
        if not frame_id:
            frame_id = 'laser'

        return stamp, frame_id

    def timer_callback(self):
        if self._in_flight:
            return
        if self._client is None:
            self._connect_client()
            return

        self._in_flight = True
        try:
            # 1) 取 LiDAR（官方 SDK）
            raw = self._client.get_ros_laser_scan()
            scan = self._extract_scan_obj(raw)
            if scan is None:
                raise RuntimeError("get_ros_laser_scan() returned None")

            # 2) cursor 去重（如果有）
            cursor = self._extract_cursor(scan)
            if self.use_cursor and cursor is not None:
                if self._last_cursor == cursor:
                    if self.drop_if_no_new:
                        return
                else:
                    self._last_cursor = cursor

            # 3) 組 ROS LaserScan
            msg = LaserScan()
            stamp, frame_id = self._extract_stamp_and_frame(scan)
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id

            msg.angle_min = float(getattr(scan, 'angle_min', 0.0))
            msg.angle_max = float(getattr(scan, 'angle_max', 0.0))
            msg.angle_increment = float(getattr(scan, 'angle_increment', 0.0))
            msg.time_increment = float(getattr(scan, 'time_increment', 0.0))
            msg.scan_time = float(getattr(scan, 'scan_time', 0.0))
            msg.range_min = float(getattr(scan, 'range_min', 0.0))
            msg.range_max = float(getattr(scan, 'range_max', 0.0))

            ranges = getattr(scan, 'ranges', [])
            intensities = getattr(scan, 'intensities', [])

            # protobuf repeated -> list
            msg.ranges = [float(x) for x in list(ranges)]
            msg.intensities = [float(x) for x in list(intensities)]

            self.scan_pub.publish(msg)
            self._pub_count += 1
            self._last_ok_time = time.time()

            if self.log_every_n > 0 and (self._pub_count % self.log_every_n == 0):
                valid = [r for r in msg.ranges if math.isfinite(r)]
                vmin = min(valid) if valid else float('nan')
                self.get_logger().info(
                    f"📡 published {self._pub_count} scans | n={len(msg.ranges)} | min_range={vmin:.3f} | frame_id={frame_id}"
                )

            # reset fail count after success
            self._fail_count = 0

        except Exception as e:
            self._fail_count += 1
            self.get_logger().warn(f"⚠️ get_ros_laser_scan() failed (fail_count={self._fail_count}): {e}")

            # 遇到伺服端中止/連線怪異時，重建 client 最有效
            if self.reconnect_every_fail > 0 and (self._fail_count % self.reconnect_every_fail == 0):
                self.get_logger().warn("🔁 Reconnecting KachakaApiClient ...")
                self._connect_client()

        finally:
            self._in_flight = False


def main(args=None):
    rclpy.init(args=args)
    node = KachakaLaserApiNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()