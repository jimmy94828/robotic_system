#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


def quat_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero.")
    return q / n


def quat_to_rotmat(q_xyzw):
    qx, qy, qz, qw = quat_normalize(q_xyzw)
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=float)


def rotmat_to_quat(R):
    R = np.array(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return quat_normalize([qx, qy, qz, qw]).tolist()


def invert_T(T):
    T = np.array(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def tfmsg_to_matrix(tf_msg: TransformStamped):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quat_to_rotmat([q.x, q.y, q.z, q.w])
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def matrix_to_tfmsg(T_parent_child, parent, child, stamp):
    T_parent_child = np.array(T_parent_child, dtype=float)
    R = T_parent_child[:3, :3]
    t = T_parent_child[:3, 3]
    qx, qy, qz, qw = rotmat_to_quat(R)

    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(t[0])
    msg.transform.translation.y = float(t[1])
    msg.transform.translation.z = float(t[2])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def load_extrinsic_yaml(path):
    """
    回傳 4x4 外參矩陣 T_raw
    - 支援你這份格式：T_lidar_to_camera.translation_xyz + quaternion_xyzw
    - 也保留舊格式：T_parent_child / translation + quaternion_xyzw
    注意：這裡「只負責讀檔」不負責判斷方向，要不要反轉交給外面參數。
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # ====== 你的 YAML 格式 ======
    if "T_lidar_to_camera" in data:
        blk = data["T_lidar_to_camera"]

        # translation
        if "translation_xyz" in blk:
            t = np.array(blk["translation_xyz"], dtype=float).reshape(3)
        elif "translation" in blk:
            t = np.array(blk["translation"], dtype=float).reshape(3)
        else:
            raise ValueError("T_lidar_to_camera must contain translation_xyz (or translation).")

        # rotation
        if "quaternion_xyzw" in blk:
            q = blk["quaternion_xyzw"]
            R = quat_to_rotmat(q)
        elif "rotation_xyzw" in blk:
            q = blk["rotation_xyzw"]
            R = quat_to_rotmat(q)
        elif "rotation_matrix" in blk:
            R = np.array(blk["rotation_matrix"], dtype=float)
            if R.shape != (3, 3):
                raise ValueError("rotation_matrix must be 3x3.")
        else:
            raise ValueError("T_lidar_to_camera must contain quaternion_xyzw (or rotation_matrix).")

        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    # ====== 舊格式：4x4 ======
    if "T_parent_child" in data:
        T = np.array(data["T_parent_child"], dtype=float)
        if T.shape != (4, 4):
            raise ValueError("T_parent_child must be 4x4.")
        return T

    # ====== 舊格式：translation + quaternion ======
    if "translation" in data and ("rotation_xyzw" in data or "quaternion_xyzw" in data):
        t = np.array(data["translation"], dtype=float).reshape(3)
        q = data.get("rotation_xyzw", data.get("quaternion_xyzw"))
        R = quat_to_rotmat(q)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    raise ValueError("Unsupported YAML format.")

class Bridge(Node):
    def __init__(self):
        super().__init__("lidar_camera_link_bridge")

        self.declare_parameter("extrinsic_yaml", "/home/acm118/calibration_test/calib_out_stage3/lidar_to_camera.yaml")
        self.declare_parameter("lidar_frame", "laser_frame")  # 改成你 /scan 的 frame_id
        self.declare_parameter("camera_link", "camera_link")
        self.declare_parameter("camera_optical", "camera_color_optical_frame")

        yaml_path = self.get_parameter("extrinsic_yaml").value
        if not yaml_path:
            raise RuntimeError("Set -p extrinsic_yaml:=/path/to/extrinsic.yaml")

        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.camera_link = self.get_parameter("camera_link").value
        self.camera_optical = self.get_parameter("camera_optical").value

        # 你的外參：lidar <- camera_optical
        self.T_L_Copt = invert_T(load_extrinsic_yaml(yaml_path))

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        self.published = False
        self.timer = self.create_timer(0.5, self.try_publish)

    def try_publish(self):
        if self.published:
            return

        # 查：camera_link <- camera_optical
        try:
            tf_Clink_Copt = self.buffer.lookup_transform(
                self.camera_link, self.camera_optical, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().info(f"Waiting TF {self.camera_link} <- {self.camera_optical}: {e}")
            return

        T_Clink_Copt = tfmsg_to_matrix(tf_Clink_Copt)

        # 合成：T_L_Clink = T_L_Copt * inv(T_Clink_Copt)
        T_L_Clink = self.T_L_Copt @ invert_T(T_Clink_Copt)

        stamp = self.get_clock().now().to_msg()
        msg = matrix_to_tfmsg(T_L_Clink, self.lidar_frame, self.camera_link, stamp)
        self.broadcaster.sendTransform(msg)

        self.published = True
        self.get_logger().info(f"Published static TF: {self.lidar_frame} -> {self.camera_link}")
        self.get_logger().info("Now you can query base_footprint <-> camera_color_optical_frame directly.")


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()