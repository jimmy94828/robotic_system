#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from laser_geometry import LaserProjection


class ScanToCloud(Node):
    def __init__(self):
        super().__init__("scan_to_cloud")

        # ---- params ----
        self.declare_parameter("scan_topic", "/kachaka/lidar/scan")
        self.declare_parameter("cloud_topic", "/scan_cloud")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("tf_timeout_sec", 0.2)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.tf_timeout = self.get_parameter("tf_timeout_sec").get_parameter_value().double_value

        # ---- tf ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- laser projection ----
        self.projector = LaserProjection()

        self.pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        self.get_logger().info(
            f"Listening {scan_topic} -> publishing {cloud_topic}, target_frame={self.target_frame}"
        )

    def cb_scan(self, scan: LaserScan):
        # 1) LaserScan -> PointCloud2 in scan frame
        cloud = self.projector.projectLaser(scan)  # output frame = scan.header.frame_id :contentReference[oaicite:4]{index=4}

        # 2) TF: scan frame -> target_frame (use scan timestamp)
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud.header.frame_id,
                scan.header.stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # 3) transform cloud
        cloud_tf = do_transform_cloud(cloud, tfm)  # :contentReference[oaicite:5]{index=5}
        cloud_tf.header.stamp = scan.header.stamp
        cloud_tf.header.frame_id = self.target_frame
        self.pub.publish(cloud_tf)

        # 4) debug: z range (判斷是否「斜」)
        zmin, zmax = self._z_range(cloud_tf)
        if zmin is not None:
            self.get_logger().info(f"z-range in {self.target_frame}: [{zmin:.3f}, {zmax:.3f}] (m)")

    def _z_range(self, cloud: PointCloud2):
        zmin, zmax = None, None
        # read_points yields (x,y,z,...) depending on fields; we only request x,y,z
        for x, y, z in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            if zmin is None:
                zmin = zmax = float(z)
            else:
                zmin = min(zmin, float(z))
                zmax = max(zmax, float(z))
        return zmin, zmax


def main():
    rclpy.init()
    node = ScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()