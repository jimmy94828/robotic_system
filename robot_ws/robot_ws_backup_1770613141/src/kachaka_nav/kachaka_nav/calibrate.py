import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import sys
import termios
import tty
import math

class MapCalibrator(Node):
    def __init__(self):
        super().__init__('map_frame_calibrator')
        # Use dynamic broadcaster for smooth movement
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/calibration_marker', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        
        print("\n=== 3D to 2D Calibrator (Visual) ===")
        print("1. Look for the GREEN BOX in RViz.")
        print("2. If the Box moves, the tool is working.")
        print("3. If your Object Markers don't move with the Box,")
        print("   check frame_id='semantic_map_origin' in your server code.")
        print("-" * 40)
        print("  W/S : Move X +/-")
        print("  A/D : Move Y +/-")
        print("  Q/E : Rotate Yaw +/-")
        print("  SPACE: Print Final YAML Values")
        print("-" * 40)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_updates(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Publish Transform (The "Skeleton" that moves the map)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "semantic_map_origin"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Visual Marker (Green Box)
        # We attach this to "semantic_map_origin" so it moves with the frame
        m = Marker()
        m.header.frame_id = "semantic_map_origin"
        m.header.stamp = now
        m.id = 9999
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8
        m.pose.orientation.w = 1.0
        
        self.marker_pub.publish(m)

    def run(self):
        while rclpy.ok():
            # Publish constantly so RViz doesn't flicker
            self.publish_updates()
            
            # Non-blocking check for input would be better, 
            # but for simplicity we assume one press = one update loop
            # To fix 'nothing showing up', we publish BEFORE waiting for key
            
            key = self.get_key()
            
            step = 0.1
            rot_step = 0.05
            
            if key == 'w': self.x += step
            elif key == 's': self.x -= step
            elif key == 'a': self.y += step
            elif key == 'd': self.y -= step
            elif key == 'q': self.yaw += rot_step
            elif key == 'e': self.yaw -= rot_step
            elif key == ' ':
                print(f"\n✅ FINAL OFFSET:")
                print(f"origin: [{self.x:.3f}, {self.y:.3f}, {self.yaw:.3f}]")
                break
            elif key == '\x03': break
            
            print(f"\rX: {self.x:.2f} Y: {self.y:.2f} Yaw: {self.yaw:.2f}   ", end="")

def main():
    rclpy.init()
    node = MapCalibrator()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()