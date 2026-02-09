import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from kachaka_interfaces.action import Navigate
from kachaka_nav import rrt_lib

# IMPORT OUR NEW DRIVER FILE
from kachaka_nav.robot_driver import KachakaRealDriver, RosSimDriver

class ModularNavNode(Node):
    def __init__(self):
        super().__init__('modular_nav_node')

        # 1. Declare Parameters
        self.declare_parameter('use_sim', False)  # FALSE = Real Robot, TRUE = Simulation
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        self.declare_parameter('map_path', '')
        self.declare_parameter('meta_path', '')
        self.declare_parameter('points_path', '')

        # 2. Read Parameters
        use_sim = self.get_parameter('use_sim').value
        robot_ip = self.get_parameter('kachaka_ip').value
        
        self.map_path = self.get_parameter('map_path').value
        self.meta_path = self.get_parameter('meta_path').value
        self.points_path = self.get_parameter('points_path').value

        # 3. SELECT THE DRIVER
        if use_sim:
            self.get_logger().info("STARTING IN SIMULATION MODE (Isaac/TurtleBot)")
            self.driver = RosSimDriver(self)
        else:
            self.get_logger().info(f"STARTING IN REAL MODE (Kachaka IP: {robot_ip})")
            # Check environment variable first, else use param
            target_ip = os.getenv('KACHAKA_CLIENT_TARGET', robot_ip)
            self.driver = KachakaRealDriver(target_ip)

        # 4. Create Action Server
        self._action_server = ActionServer(
            self, Navigate, 'navigate_to_pose', 
            self.execute_callback, 
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Modular Navigation Node is Ready.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received Goal Request...')
        
        # GET POSITION (Works for both Sim and Real!)
        start_x, start_y, _ = self.driver.get_pose()
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        # PLAN PATH (RRT)
        self.get_logger().info(f'Planning from ({start_x:.2f}, {start_y:.2f}) to ({target_x}, {target_y})...')
        path = rrt_lib.rrt_planning(
            (start_x, start_y), 
            (target_x, target_y), 
            self.map_path, self.meta_path, self.points_path
        )

        if not path:
            self.get_logger().error("RRT failed to find a path.")
            goal_handle.abort()
            return Navigate.Result(success=False)

        # EXECUTE PATH
        self.get_logger().info(f"Path found with {len(path)} waypoints. Executing...")
        
        for i, point in enumerate(path):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.driver.stop()
                return Navigate.Result(success=False)

            # Drive to this waypoint
            self.drive_to_point(point[0], point[1])
            
            # Send Feedback
            feedback_msg = Navigate.Feedback()
            feedback_msg.distance_remaining = float(len(path) - i) # Approx
            goal_handle.publish_feedback(feedback_msg)

        self.driver.stop()
        goal_handle.succeed()
        return Navigate.Result(success=True)

    def drive_to_point(self, tx, ty):
        # Pure Pursuit Logic
        while True:
            # 1. Get current state (Abstracted)
            curr_x, curr_y, curr_theta = self.driver.get_pose()
            
            dx = tx - curr_x
            dy = ty - curr_y
            distance = math.hypot(dx, dy)

            # Threshold to switch to next point (15cm)
            if distance < 0.15:
                break
            
            # 2. Calculate Control
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - curr_theta
            
            # Normalize angle (-pi to pi)
            while angle_error > math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi

            # Simple P-Controller
            linear_vel = min(0.4 * distance, 0.5)  # Cap speed at 0.5 m/s
            angular_vel = 1.2 * angle_error        # Turn speed

            # 3. Send Command (Abstracted)
            self.driver.set_velocity(linear_vel, angular_vel)
            
            time.sleep(0.05) # 20Hz Loop

def main(args=None):
    rclpy.init(args=args)
    node = ModularNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()