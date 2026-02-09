import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import yaml
import sys
import os
import math

class NudgeAndScale(Node):
    def __init__(self, yaml_path):
        super().__init__('nudge_and_scale_tool')
        
        # 1. Load Map
        self.yaml_path = yaml_path
        self.map_data = self.load_yaml(yaml_path)
        self.img = cv2.imread(self.map_data['image_path'])
        
        if self.img is None:
            self.get_logger().error("Could not load map image!")
            sys.exit(1)
            
        # Initial Values from YAML
        self.base_resolution = self.map_data['resolution']
        self.base_origin = self.map_data['origin'] # [x, y, yaw]
        self.height, self.width = self.img.shape[:2]
        
        # 2. Correction Factors
        self.nudge_x = 0.0
        self.nudge_y = 0.0
        self.scale_factor = 1.0 # 1.0 = No change
        
        # 3. Subscribe to Pose
        self.create_subscription(PoseStamped, '/user_pose', self.pose_callback, 10)
        
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.got_pose = False
        
        print("\n" + "="*50)
        print("      NUDGE & SCALE TOOL")
        print("="*50)
        print(" [Arrows] : MOVE map (Nudge X/Y)")
        print(" [Z / X]  : ZOOM map scale (Resolution)")
        print("------------------------------------------")
        print(" HOW TO TUNE:")
        print(" 1. Drive robot 1 meter forward.")
        print(" 2. If dot moved TOO FAR -> Press Z (Increase Res)")
        print(" 3. If dot moved TOO LITTLE -> Press X (Decrease Res)")
        print(" 4. Use Arrows to keep dot inside the hallway.")
        print("------------------------------------------")
        print(" [Enter]  : SAVE & PRINT NEW YAML")
        print("="*50)

    def load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        map_dir = os.path.dirname(os.path.abspath(yaml_path))
        if os.path.isabs(data['image']):
            img_path = data['image']
        else:
            img_path = os.path.join(map_dir, data['image'])
        return {'image_path': img_path, 'resolution': data['resolution'], 'origin': data['origin']}

    def pose_callback(self, msg):
        self.raw_x = msg.pose.position.x
        self.raw_y = msg.pose.position.y
        self.got_pose = True

    def spin_gui(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            display_img = self.img.copy()
            
            # Current active resolution
            curr_res = self.base_resolution * self.scale_factor

            if self.got_pose:
                # 1. Apply Nudge (Meters)
                corrected_x = self.raw_x + self.nudge_x
                corrected_y = self.raw_y + self.nudge_y
                
                # 2. Convert to Pixels using NEW SCALE
                # Note: We must clamp values to avoid crashing if dot goes off screen
                try:
                    px = int(corrected_x / curr_res)
                    py = int(self.height - (corrected_y / curr_res))
                    
                    # Draw Red Dot
                    if 0 <= px < self.width and 0 <= py < self.height:
                        cv2.circle(display_img, (px, py), 6, (0, 0, 255), -1)
                        cv2.circle(display_img, (px, py), 10, (0, 0, 255), 2)
                    else:
                        # Draw warning if out of bounds
                        cv2.putText(display_img, "OUT OF BOUNDS!", (10, 150), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

                    # Info Text
                    cv2.putText(display_img, f"Res: {curr_res:.5f}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
                    cv2.putText(display_img, f"Nudge X: {self.nudge_x:.2f}", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,0), 2)
                    cv2.putText(display_img, f"Nudge Y: {self.nudge_y:.2f}", (10, 85), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,0), 2)

                except ValueError:
                    pass
            else:
                cv2.putText(display_img, "Waiting for Robot...", (50, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            cv2.imshow("Nudge & Scale", display_img)
            key = cv2.waitKey(10)
            
            # Controls
            move_step = 0.2
            scale_step = 0.01

            if key == 13: # Enter
                self.print_new_yaml(curr_res)
                break
            elif key == 27: # ESC
                break
            elif key == 82 or key == ord('w'): self.nudge_y += move_step # Up
            elif key == 84 or key == ord('s'): self.nudge_y -= move_step # Down
            elif key == 81 or key == ord('a'): self.nudge_x -= move_step # Left
            elif key == 83 or key == ord('d'): self.nudge_x += move_step # Right
            elif key == ord('z'): self.scale_factor += scale_step # Increase Res (Zoom Out)
            elif key == ord('x'): self.scale_factor -= scale_step # Decrease Res (Zoom In)

        cv2.destroyAllWindows()

    def print_new_yaml(self, final_res):
        # Calculate new origin based on nudge
        new_origin_x = self.base_origin[0] - self.nudge_x
        new_origin_y = self.base_origin[1] - self.nudge_y
        yaw = self.base_origin[2]

        print("\n" + "="*50)
        print("✅  SCALE & ALIGNMENT FIXED!")
        print("   COPY THIS TO YOUR YAML FILE:")
        print("="*50)
        print(f"resolution: {final_res:.6f}")
        print(f"origin: [{new_origin_x:.4f}, {new_origin_y:.4f}, {yaw:.4f}]")
        print("="*50)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 nudge_and_scale.py <my_map.yaml>")
        return
    rclpy.init()
    node = NudgeAndScale(sys.argv[1])
    node.spin_gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()