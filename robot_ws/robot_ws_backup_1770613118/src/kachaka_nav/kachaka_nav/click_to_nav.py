import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kachaka_interfaces.action import Navigate
import sys
import os

class MapClicker(Node):
    def __init__(self, yaml_path):
        super().__init__('map_clicker_gui')
        self._action_client = ActionClient(self, Navigate, '/navigate_to_pose')
        
        # Load Map Metadata
        self.map_data = self.load_yaml(yaml_path)
        self.img = cv2.imread(self.map_data['image_path'])
        
        if self.img is None:
            self.get_logger().error("Could not load map image!")
            sys.exit(1)
            
        self.resolution = self.map_data['resolution']
        self.height, self.width = self.img.shape[:2]
        
        # Visual setup
        self.window_name = "Click to Send Command"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.get_logger().info("Map Clicker Ready. Click anywhere on the image to move the robot.")
        self.get_logger().info("Press 'ESC' to quit.")

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
            'resolution': data['resolution']
            # We ignore 'origin' here because the modular_nav_node handles the 
            # World Offset. We just need to send local Image Coordinates (Meters).
        }

    def pixel_to_local_meters(self, u, v):
        """
        Converts pixel (u, v) to meters relative to the map's bottom-left corner.
        u: column (x)
        v: row (y)
        """
        # ROS Standard: Origin is Bottom-Left
        # Image Standard: Origin is Top-Left
        
        x_meters = u * self.resolution
        y_meters = (self.height - v) * self.resolution
        
        return x_meters, y_meters

    def send_goal(self, x, y):
        self.get_logger().info(f"Waiting for Nav Node...")
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Modular Nav Node not found! Is it running?")
            return

        goal_msg = Navigate.Goal()
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)
        
        self.get_logger().info(f"Sending Goal -> X: {x:.2f}, Y: {y:.2f}")
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            return
        self.get_logger().info("Goal accepted! Robot is moving...")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 1. Draw a circle where user clicked
            display_img = self.img.copy()
            cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow(self.window_name, display_img)
            
            # 2. Convert to meters
            mx, my = self.pixel_to_local_meters(x, y)
            
            # 3. Send to ROS
            self.send_goal(mx, my)

    def spin_gui(self):
        while rclpy.ok():
            cv2.imshow(self.window_name, self.img)
            key = cv2.waitKey(100)
            if key == 27: # ESC
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
        cv2.destroyAllWindows()

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 click_to_nav.py <my_map.yaml>")
        return

    rclpy.init(args=args)
    clicker = MapClicker(sys.argv[1])
    clicker.spin_gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()