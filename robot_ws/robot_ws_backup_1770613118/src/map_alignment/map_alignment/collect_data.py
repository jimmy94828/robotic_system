#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import threading
from pathlib import Path
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

# ---- UI: Matplotlib 鍵盤事件 ----
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion -> yaw (Z axis)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CaptureRGBDAndLidarPose(Node):
    def __init__(self):
        super().__init__("capture_rgbd_and_lidar_pose")

        # ---------------- Params ----------------
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")

        self.declare_parameter("fixed_frame", "map")             # map / odom
        self.declare_parameter("lidar_frame", "base_footprint")  # base_scan / laser_link / lidar_frame...
        self.declare_parameter("output_dir", str(Path.home() / "capture_logs"))
        self.declare_parameter("sync_tolerance_ms", 50.0)        # 深度與彩色允許的時間差（ms）
        self.declare_parameter("depth_cache_size", 200)          # 暫存最近幾張深度

        self.color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value
        self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self.output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)

        self.sync_tol_ns = int(self.get_parameter("sync_tolerance_ms").value * 1e6)
        self.depth_cache_size = int(self.get_parameter("depth_cache_size").value)

        # ---------------- Output dirs ----------------
        (self.output_dir / "rgb").mkdir(parents=True, exist_ok=True)
        (self.output_dir / "depth").mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "captures.csv"
        self._init_csv_if_needed()

        # ---------------- ROS IO ----------------
        self.bridge = CvBridge()
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self._lock = threading.Lock()

        self.latest_color_bgr = None
        self.latest_color_stamp = None  # builtin_interfaces/Time
        self.latest_color_frame = None

        # cache: deque of (stamp_ns, depth_array, depth_encoding)
        self.depth_cache = deque(maxlen=self.depth_cache_size)

        # ---------------- Matplotlib UI ----------------
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Press 'C' to capture RGB+D+pose | Press 'Q' to quit")
        self.im_artist = None
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self.timer = self.create_timer(0.05, self.refresh_ui)

        self.get_logger().info("Node started.")
        self.get_logger().info(f"Color topic : {self.color_topic}")
        self.get_logger().info(f"Depth topic : {self.depth_topic}")
        self.get_logger().info(f"Fixed frame : {self.fixed_frame}")
        self.get_logger().info(f"LiDAR frame : {self.lidar_frame}")
        self.get_logger().info(f"Output dir  : {self.output_dir}")
        self.get_logger().info(f"Sync tol    : {self.sync_tol_ns/1e6:.1f} ms")

    def _init_csv_if_needed(self):
        if not self.csv_path.exists():
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow([
                    "rgb_file",
                    "depth_file",
                    "stamp_sec", "stamp_nanosec",
                    "depth_encoding",
                    "fixed_frame", "lidar_frame",
                    "x", "y", "yaw_rad",
                    "qx", "qy", "qz", "qw"
                ])

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def on_color(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge color convert failed: {e}")
            return

        with self._lock:
            self.latest_color_bgr = bgr
            self.latest_color_stamp = msg.header.stamp
            self.latest_color_frame = msg.header.frame_id

    def on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge depth convert failed: {e}")
            return

        stamp_ns = self._stamp_to_ns(msg.header.stamp)

        if depth is None:
            return
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        with self._lock:
            self.depth_cache.append((stamp_ns, depth, msg.encoding))

    def refresh_ui(self):
        with self._lock:
            if self.latest_color_bgr is None:
                plt.pause(0.001)
                return
            rgb = self.latest_color_bgr[:, :, ::-1].copy()

        if self.im_artist is None:
            self.im_artist = self.ax.imshow(rgb)
            self.ax.axis("off")
        else:
            self.im_artist.set_data(rgb)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def on_key(self, event):
        if event.key is None:
            return
        k = event.key.lower()
        if k == "c":
            self.capture()
        elif k == "q":
            self.get_logger().info("Quit requested. Shutting down...")
            rclpy.shutdown()

    def _find_depth_nearest(self, target_ns: int):
        """Return (depth, encoding) nearest to target_ns within tolerance; else None."""
        with self._lock:
            if not self.depth_cache:
                return None

            best = None
            best_dt = None
            for ts, depth, enc in self.depth_cache:
                dt = abs(ts - target_ns)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best = (depth, enc)

        if best is None or best_dt is None or best_dt > self.sync_tol_ns:
            return None
        return best

    def _save_depth(self, depth: np.ndarray, base: str):
        """
        Save depth:
        - uint16 -> PNG (保留原值)
        - float32/float64 -> NPY (保留原值)
        return relative path string.
        """
        if depth.dtype == np.uint16:
            rel = Path("depth") / f"{base}.png"
            path = self.output_dir / rel
            ok = cv2.imwrite(str(path), depth)
            if not ok:
                raise RuntimeError("cv2.imwrite failed for depth PNG")
            return str(rel)

        rel = Path("depth") / f"{base}.npy"
        path = self.output_dir / rel
        np.save(str(path), depth.astype(np.float32, copy=False))
        return str(rel)

    def capture(self):
        # 取最新 RGB 與時間戳
        with self._lock:
            if self.latest_color_bgr is None or self.latest_color_stamp is None:
                self.get_logger().warn("No color image yet; cannot capture.")
                return
            color_bgr = self.latest_color_bgr.copy()
            stamp = self.latest_color_stamp

        sec = int(stamp.sec)
        nsec = int(stamp.nanosec)
        base = f"{sec}_{nsec:09d}"
        stamp_ns = sec * 1_000_000_000 + nsec

        # ✅ 1) 先查 TF（pose），失敗就直接不存任何檔案
        t = rclpy.time.Time(seconds=sec, nanoseconds=nsec)
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.lidar_frame,
                t,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed at {sec}.{nsec:09d}: {e}. "
                f"Skip saving this capture."
            )
            return

        tx = tfm.transform.translation.x
        ty = tfm.transform.translation.y
        qx = tfm.transform.rotation.x
        qy = tfm.transform.rotation.y
        qz = tfm.transform.rotation.z
        qw = tfm.transform.rotation.w
        yaw = quat_to_yaw(qx, qy, qz, qw)

        # ✅ 2) TF 成功後才存 RGB / Depth
        rgb_rel = Path("rgb") / f"{base}.png"
        rgb_path = self.output_dir / rgb_rel

        depth_rel_str = ""
        depth_encoding = ""

        # 讓後續如果存檔/寫 CSV 出錯時可清理
        saved_rgb = False
        saved_depth_path_abs = None

        try:
            # 存 RGB（OpenCV 慣用：直接寫 BGR）
            ok = cv2.imwrite(str(rgb_path), color_bgr)
            if not ok:
                raise RuntimeError("cv2.imwrite failed for RGB PNG")
            saved_rgb = True

            # depth 可選（找不到就只存 RGB + pose）
            depth_pack = self._find_depth_nearest(stamp_ns)
            if depth_pack is None:
                self.get_logger().warn(
                    f"Depth not found within {self.sync_tol_ns/1e6:.1f} ms of RGB stamp; "
                    f"save RGB+pose only."
                )
            else:
                depth, depth_encoding = depth_pack
                depth_rel_str = self._save_depth(depth, base)
                saved_depth_path_abs = self.output_dir / depth_rel_str

            # 寫 CSV
            with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow([
                    str(rgb_rel),
                    depth_rel_str,
                    sec, nsec,
                    depth_encoding,
                    self.fixed_frame, self.lidar_frame,
                    f"{tx:.6f}", f"{ty:.6f}", f"{yaw:.6f}",
                    f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}", f"{qw:.6f}",
                ])

        except Exception as e:
            # ❗發生任何錯誤，盡量把這次 capture 已存的檔案清掉，避免殘留半套資料
            self.get_logger().error(f"Capture failed, cleanup files. Reason: {e}")

            try:
                if saved_rgb and rgb_path.exists():
                    rgb_path.unlink()
            except Exception:
                pass

            try:
                if saved_depth_path_abs is not None and saved_depth_path_abs.exists():
                    saved_depth_path_abs.unlink()
            except Exception:
                pass

            return

        self.get_logger().info(
            f"Captured RGB={rgb_rel}, D={depth_rel_str or 'NONE'} | "
            f"pose {self.fixed_frame}->{self.lidar_frame}: x={tx:.3f}, y={ty:.3f}, yaw={yaw:.3f}"
        )


def main():
    rclpy.init()
    node = CaptureRGBDAndLidarPose()
    plt.show(block=False)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()