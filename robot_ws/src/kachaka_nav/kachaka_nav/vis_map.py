import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import yaml
import sys
import os
import math
import numpy as np

class RobotVisualizer(Node):
    def __init__(self, yaml_path):
        super().__init__('robot_visualizer')
        
        # 1. Load Map
        self.map_data = self.load_yaml(yaml_path)
        self.img = cv2.imread(self.map_data['image_path'])
        
        if self.img is None:
            self.get_logger().error("Could not load map image!")
            sys.exit(1)
            
        self.resolution = self.map_data['resolution']
        self.height, self.width = self.img.shape[:2]
        
        # 2. Subscribe to the Node's new topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/user_pose',  # The topic we added to ModularNavNode
            self.pose_callback,
            10
        )
        
        self.current_x_px = 0
        self.current_y_px = 0
        self.got_pose = False
        
        self.get_logger().info("Visualizer Started. Waiting for robot pose...")

    def load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        map_dir = os.path.dirname(os.path.abspath(yaml_path))
        if os.path.isabs(data['image']):
            img_path = data['image']
        else:
            img_path = os.path.join(map_dir, data['image'])
        return {'image_path': img_path, 'resolution': data['resolution']}

    def pose_callback(self, msg):
        # 1. Get Meters from ROS Topic
        mx = msg.pose.position.x
        my = msg.pose.position.y
        
        # 2. Convert to Pixels (Standard ROS Map Logic)
        # Pixel X = Meters / Res
        # Pixel Y = Height - (Meters / Res)
        
        px = int(mx / self.resolution)
        py = int(self.height - (my / self.resolution))
        
        self.current_x_px = px
        self.current_y_px = py
        self.got_pose = True

    def spin_gui(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Create a clean copy to draw on
            display_img = self.img.copy()
            
            if self.got_pose:
                # Draw Robot (Red Circle)
                cv2.circle(display_img, 
                           (self.current_x_px, self.current_y_px), 
                           8, (0, 0, 255), -1)
                
                # Draw Text
                text = f"Pos: {self.current_x_px}, {self.current_y_px}"
                cv2.putText(display_img, text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(display_img, "Waiting for Robot...", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)

            cv2.imshow("Real-Time Robot Tracker", display_img)
            
            if cv2.waitKey(1) & 0xFF == 27: # ESC to quit
                break
        
        cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_robot.py <my_map.yaml>")
        return
    
    rclpy.init()
    vis = RobotVisualizer(sys.argv[1])
    vis.spin_gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()