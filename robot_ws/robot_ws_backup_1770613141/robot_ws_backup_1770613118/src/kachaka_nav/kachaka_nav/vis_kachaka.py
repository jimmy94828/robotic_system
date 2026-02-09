import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import yaml
import sys
import os
import math

class KachakaVisualizer(Node):
    def __init__(self, yaml_path):
        super().__init__('kachaka_visualizer')
        
        # 1. Load Map
        self.map_data = self.load_yaml(yaml_path)
        self.img = cv2.imread(self.map_data['image_path'])
        
        if self.img is None:
            self.get_logger().error("Could not load Kachaka map image!")
            sys.exit(1)
            
        self.resolution = self.map_data['resolution']
        self.origin = self.map_data['origin'] 
        self.height, self.width = self.img.shape[:2]
        
        # 2. Subscribe to ROBOT Position (Blue)
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/kachaka_pose',
            self.robot_callback,
            10
        )
        
        # 3. Subscribe to COMMANDED GOAL (Green)
        # Your DecisionMaker node publishes this topic immediately when it decides to move!
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Robot State
        self.robot_px = (0, 0)
        self.got_robot = False
        
        # Goal State
        self.goal_px = (0, 0)
        self.got_goal = False
        
        self.get_logger().info("Visualizer Ready: Blue=Robot, Green=Goal")

    def load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        map_dir = os.path.dirname(os.path.abspath(yaml_path))
        if os.path.isabs(data['image']):
            img_path = data['image']
        else:
            img_path = os.path.join(map_dir, data['image'])
        return {
            'image_path': img_path, 
            'resolution': data['resolution'],
            'origin': data['origin']
        }

    def world_to_pixel(self, wx, wy):
        ox = self.origin[0]
        oy = self.origin[1]
        
        px = int((wx - ox) / self.resolution)
        # Flip Y for image coordinates
        py = int(self.height - ((wy - oy) / self.resolution))
        return (px, py)

    def robot_callback(self, msg):
        self.robot_px = self.world_to_pixel(msg.pose.position.x, msg.pose.position.y)
        self.got_robot = True

    def goal_callback(self, msg):
        # This updates INSTANTLY when you send a command
        self.goal_px = self.world_to_pixel(msg.pose.position.x, msg.pose.position.y)
        self.got_goal = True
        self.get_logger().info("Received new Goal Command!")

    def spin_gui(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Start with fresh map
            display_img = self.img.copy()
            
            # 1. Draw GOAL (Green 'X' or Star)
            if self.got_goal:
                gx, gy = self.goal_px
                # Draw lines to form an X
                cv2.line(display_img, (gx-5, gy-5), (gx+5, gy+5), (0, 200, 0), 2)
                cv2.line(display_img, (gx+5, gy-5), (gx-5, gy+5), (0, 200, 0), 2)
                cv2.putText(display_img, "GOAL", (gx+10, gy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

            # 2. Draw ROBOT (Blue Circle)
            if self.got_robot:
                rx, ry = self.robot_px
                cv2.circle(display_img, (rx, ry), 8, (255, 0, 0), -1)
                
                # Optional: Draw line from Robot to Goal
                if self.got_goal:
                    cv2.line(display_img, (rx, ry), self.goal_px, (0, 255, 255), 1)

            # 3. Text Overlay
            if not self.got_robot:
                cv2.putText(display_img, "Waiting for /kachaka_pose...", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("Kachaka Monitor", display_img)
            
            if cv2.waitKey(10) & 0xFF == 27:
                break
        
        cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_kachaka.py <kachaka_master.yaml>")
        return
    
    rclpy.init()
    vis = KachakaVisualizer(sys.argv[1])
    vis.spin_gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()