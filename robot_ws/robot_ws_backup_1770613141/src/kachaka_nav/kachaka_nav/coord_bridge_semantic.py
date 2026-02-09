#!/usr/bin/env python3
"""
座標連接，把kachaka的位置轉換到3d地圖
顯示kachaka的位置和移軌跡
"""
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
import numpy as np

class CoordBridgeNode(Node):
    def __init__(self):
        super().__init__('coord_bridge_node')
        
        # 初始化路徑
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        # === kachaka to semantic ===
        #  auto_align.py 獲得
        self.declare_parameter('tx', 0.0)  # X 方向平移
        self.declare_parameter('ty', 0.0)  # Y 方向平移  
        self.declare_parameter('rot', 0.0) # 旋轉角度（弧度）
        self.declare_parameter('sx', 1.0)  # X 方向縮放（通常 1.0 或 -1.0）
        self.declare_parameter('sy', 1.0)  # Y 方向縮放（通常 1.0 或 -1.0）
        self.declare_parameter('use_z_swap', True)  # 是否交換 Y 和 Z 軸（3D->2D）
        self.declare_parameter('robot_height', -1.0)  # 機器人高度（Z 值），可調整
        
        self.tx = self.get_parameter('tx').value
        self.ty = self.get_parameter('ty').value
        self.rot = self.get_parameter('rot').value
        self.sx = self.get_parameter('sx').value
        self.sy = self.get_parameter('sy').value
        self.use_z_swap = self.get_parameter('use_z_swap').value
        self.robot_height = self.get_parameter('robot_height').value
        
        self.get_logger().info(f"Bridge Transform: tx={self.tx:.2f}, ty={self.ty:.2f}, rot={math.degrees(self.rot):.1f}°")
        
        # === 發布器===
        self.pub_transformed = self.create_publisher(PoseStamped, '/robot_pose_in_semantic_map', 10)
        self.pub_path = self.create_publisher(Path, '/robot_path_semantic', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # === Kachaka 原始位置訂閱器（最後創建）===
        self.sub_kachaka = self.create_subscription(
            PoseStamped,
            '/kachaka_pose',
            self.kachaka_callback,
            10
        )
        
        self.get_logger().info("✅ Coordinate Bridge Ready!")
        
    def transform_kachaka_to_semantic(self, kx, ky, kyaw):
        """
        將 Kachaka 坐標系的位置變換到語義地圖坐標系
        auto_align.py 中的變換邏輯
        """
        # 1. 處理 Z 軸交換（如果語義地圖使用 X-Z 平面）
        if self.use_z_swap:
            x_raw = kx
            y_raw = ky  # 在 Kachaka 中是 Y，在語義地圖中對應 Z
        else:
            x_raw = kx
            y_raw = ky
        
        # 2. 翻轉（如果需要）
        x_flipped = x_raw * self.sx
        y_flipped = y_raw * self.sy
        
        # 3. 旋轉
        c = math.cos(self.rot)
        s = math.sin(self.rot)
        x_rot = x_flipped * c - y_flipped * s
        y_rot = x_flipped * s + y_flipped * c
        
        # 4. 平移
        x_final = x_rot + self.tx
        y_final = y_rot + self.ty
        
        # 5. 旋轉角度
        yaw_final = kyaw + self.rot
        
        return x_final, y_final, yaw_final
    
    def kachaka_callback(self, msg):
        """接收 Kachaka 位置並發布變換後的位置"""
        try:
            # 提取 Kachaka 位置
            kx = msg.pose.position.x
            ky = msg.pose.position.y
            
            # 從四元數提取 yaw
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            kyaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # 進行坐標變換
            sx, sy, syaw = self.transform_kachaka_to_semantic(kx, ky, kyaw)
            
            current_time = self.get_clock().now().to_msg()
            
            #   發布變換後的位置
            transformed_msg = PoseStamped()
            transformed_msg.header.stamp = current_time
            transformed_msg.header.frame_id = "map"  # 使用與點雲相同的 frame
            transformed_msg.pose.position.x = sx
            transformed_msg.pose.position.y = sy
            transformed_msg.pose.position.z = self.robot_height  # 机器人高度（可调整）
            
            # 轉換角度為四元數
            quat = euler2quat(0, 0, syaw)
            transformed_msg.pose.orientation.w = quat[0]
            transformed_msg.pose.orientation.x = quat[1]
            transformed_msg.pose.orientation.y = quat[2]
            transformed_msg.pose.orientation.z = quat[3]
            
            # 發布變換後的位置
            self.pub_transformed.publish(transformed_msg)
            
            # 廣播 TF（map -> robot_base_link）
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "robot_base_link"
            t.transform.translation.x = sx
            t.transform.translation.y = sy
            t.transform.translation.z = self.robot_height
            t.transform.rotation.w = quat[0]
            t.transform.rotation.x = quat[1]
            t.transform.rotation.y = quat[2]
            t.transform.rotation.z = quat[3]
            self.tf_broadcaster.sendTransform(t)
            
            # 添加到路徑 - 创建新的PoseStamped对象副本
            path_pose = PoseStamped()
            path_pose.header.stamp = current_time
            path_pose.header.frame_id = "map"
            path_pose.pose.position.x = sx
            path_pose.pose.position.y = sy
            path_pose.pose.position.z = self.robot_height
            path_pose.pose.orientation.w = quat[0]
            path_pose.pose.orientation.x = quat[1]
            path_pose.pose.orientation.y = quat[2]
            path_pose.pose.orientation.z = quat[3]
            
            self.path_msg.header.stamp = current_time
            self.path_msg.poses.append(path_pose)
            if len(self.path_msg.poses) > 8000:
                self.path_msg.poses.pop(0)
            self.pub_path.publish(self.path_msg)
            
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")

def main():
    rclpy.init()
    node = CoordBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
