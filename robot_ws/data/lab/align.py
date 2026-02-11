#!/usr/bin/env python3
"""
calibration tool for live alignment between Kachaka coordinates and semantic map coordinates
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys

class LiveAlignmentNode(Node):
    def __init__(self):
        super().__init__('live_alignment')
        
        # 記錄對應點
        self.kachaka_points = []
        self.semantic_points = []
        
        # 訂閱 Kachaka 位置
        self.sub_kachaka = self.create_subscription(
            PoseStamped,
            '/kachaka_pose',
            self.kachaka_callback,
            10
        )
        
        self.latest_kachaka = None
        self.get_logger().info("Live Alignment Tool Started")
        self.get_logger().info("=" * 60)
        self.print_instructions()
        
    def print_instructions(self):
        print("\n使用方法:")
        print("1. 讓機器人移動到一個已知位置")
        print("2. 在 RViz 中找到對應的語義地圖位置")
        print("3. 在命令行中輸入語義地圖座標")
        print("4. 重複步驟 1-3，收集至少 3 個對應點")
        print("5. 程式會計算變換參數\n")
        
    def kachaka_callback(self, msg):
        self.latest_kachaka = (msg.pose.position.x, msg.pose.position.y)
        
    def add_correspondence(self, semantic_x, semantic_y):
        """添加一對對應點"""
        if self.latest_kachaka is None:
            print("❌ 還沒有收到 Kachaka 位置數據")
            return False
        
        kx, ky = self.latest_kachaka
        self.kachaka_points.append([kx, ky])
        self.semantic_points.append([semantic_x, semantic_y])
        
        print(f"\n✅ 已添加對應點 #{len(self.kachaka_points)}:")
        print(f"  Kachaka:  ({kx:.3f}, {ky:.3f})")
        print(f"  Semantic: ({semantic_x:.3f}, {semantic_y:.3f})")
        
        if len(self.kachaka_points) >= 2:
            self.estimate_transform()
        
        return True
    
    def estimate_transform(self):
        """估算變換參數（簡化版：僅平移和旋轉）"""
        if len(self.kachaka_points) < 2:
            print("需要至少 2 個對應點")
            return
        
        kachaka_arr = np.array(self.kachaka_points)
        semantic_arr = np.array(self.semantic_points)
        
        # 計算質心
        k_center = kachaka_arr.mean(axis=0)
        s_center = semantic_arr.mean(axis=0)
        
        # 去中心化
        k_centered = kachaka_arr - k_center
        s_centered = semantic_arr - s_center
        
        # 使用 SVD 計算旋轉
        H = k_centered.T @ s_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 處理反射情況
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 提取旋轉角度
        rot_angle = math.atan2(R[1, 0], R[0, 0])
        
        # 計算平移
        t = s_center - R @ k_center
        
        print("\n" + "=" * 60)
        print("估算的變換參數:")
        print("=" * 60)
        print(f"tx: {t[0]:.3f}")
        print(f"ty: {t[1]:.3f}")
        print(f"rot: {rot_angle:.6f}  # {math.degrees(rot_angle):.2f}°")
        print(f"sx: 1.0")
        print(f"sy: 1.0")
        
        # 驗證誤差
        print(f"\n驗證誤差 (使用 {len(self.kachaka_points)} 個點):")
        errors = []
        for i, (kp, sp) in enumerate(zip(kachaka_arr, semantic_arr)):
            # 應用變換
            transformed = R @ kp + t
            error = np.linalg.norm(transformed - sp)
            errors.append(error)
            print(f"  點 {i+1}: {error:.3f} m")
        
        print(f"平均誤差: {np.mean(errors):.3f} m")
        
        print("\n使用這些參數啟動 coord_bridge:")
        print(f"ros2 run kachaka_nav coord_bridge --ros-args \\")
        print(f"  -p tx:={t[0]:.3f} \\")
        print(f"  -p ty:={t[1]:.3f} \\")
        print(f"  -p rot:={rot_angle:.6f} \\")
        print(f"  -p sx:=1.0 \\")
        print(f"  -p sy:=1.0")
        print("=" * 60 + "\n")

def main():
    rclpy.init()
    node = LiveAlignmentNode()
    
    print("\n請確保以下節點正在運行:")
    print("1. ros2 run kachaka_nav simple_pose_test  (或連接真實機器人)")
    print("2. ros2 run object_query object_query_server")
    print("3. rviz2 (顯示語義地圖)\n")
    
    # 創建一個線程來處理 ROS 回調
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    print("輸入 'help' 查看幫助，'quit' 退出\n")
    
    try:
        while rclpy.ok():
            try:
                cmd = input(">>> ").strip()
                
                if cmd == 'quit' or cmd == 'exit':
                    break
                elif cmd == 'help':
                    node.print_instructions()
                elif cmd == 'status':
                    if node.latest_kachaka:
                        kx, ky = node.latest_kachaka
                        print(f"當前 Kachaka 位置: ({kx:.3f}, {ky:.3f})")
                        print(f"已記錄 {len(node.kachaka_points)} 個對應點")
                    else:
                        print("還沒有收到 Kachaka 位置")
                elif cmd.startswith('add '):
                    # 格式: add x y
                    parts = cmd.split()
                    if len(parts) == 3:
                        try:
                            sx = float(parts[1])
                            sy = float(parts[2])
                            node.add_correspondence(sx, sy)
                        except ValueError:
                            print("❌ 坐標格式錯誤，使用: add <x> <y>")
                    else:
                        print("❌ 用法: add <x> <y>")
                        print("   示例: add 1.5 2.3")
                else:
                    print("未知命令。可用命令:")
                    print("  add <x> <y>  - 添加對應點（語義地圖坐標）")
                    print("  status       - 顯示當前狀態")
                    print("  help         - 顯示幫助")
                    print("  quit         - 退出")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\n使用 'quit' 退出")
                continue
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
