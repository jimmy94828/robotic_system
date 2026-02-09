import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from kachaka_interfaces.action import Navigate
from kachaka_nav.robot_driver import KachakaRealDriver, RosSimDriver

class ModularNavNode(Node):
    def __init__(self):
        super().__init__('modular_nav_node')
        self.declare_parameter('use_sim', False)
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        
        self.use_sim = self.get_parameter('use_sim').value
        robot_ip = self.get_parameter('kachaka_ip').value

        if self.use_sim:
            self.driver = RosSimDriver(self)
        else:
            self.get_logger().info(f"Connecting to {robot_ip}...")
            self.driver = KachakaRealDriver(robot_ip)

        self._action_server = ActionServer(
            self, Navigate, '/navigate_to_pose', 
            self.execute_callback, 
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Ready (Native Kachaka Navigation).")

    def execute_callback(self, goal_handle):
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        
        self.get_logger().info(f"Using Kachaka Native Nav to: ({target_x:.2f}, {target_y:.2f})")

        # 1. TRIGGER MOVEMENT (Native)
        # We assume 0.0 rotation (yaw) for now
        self.driver.move_native(target_x, target_y, yaw=0.0)

        # 2. MONITOR PROGRESS
        # Since move_to_pose returns immediately, we loop to check if we arrived.
        timeout_start = time.time()
        success = False

        while rclpy.ok():
            # A. Check Timeout (e.g. 60 seconds max)
            if time.time() - timeout_start > 60.0:
                self.get_logger().error("Native Nav Timed Out!")
                self.driver.stop()
                break

            # B. Check Distance to Goal
            curr_x, curr_y, _ = self.driver.get_pose()
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.sqrt(dx**2 + dy**2)

            # C. Monitor Command State (Optional but good)
            # If the robot decides it's "Done", it stops. 
            # But checking distance is the most reliable generic way.
            
            if dist < 0.15: # 15cm tolerance
                self.get_logger().info("Target Reached!")
                success = True
                break
            
            # Sleep to avoid spamming CPU
            time.sleep(0.5)

        if success:
            goal_handle.succeed()
            return Navigate.Result(success=True)
        else:
            goal_handle.abort()
            return Navigate.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = ModularNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()