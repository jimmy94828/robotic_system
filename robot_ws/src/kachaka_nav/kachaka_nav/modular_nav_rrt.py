import rclpy
import math
import time
import os
import yaml # Only need to read yaml
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from kachaka_interfaces.action import Navigate
from kachaka_nav.robot_driver import KachakaRealDriver, RosSimDriver
# from kachaka_nav import rrt_lib 
from kachaka_nav import rrt_lib_debug as rrt_lib
class ModularNavNode(Node):
    def __init__(self):
        super().__init__('modular_nav_node_rrt')
        self.declare_parameter('use_sim', False)
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        self.declare_parameter('use_native_map', True)
        
        self.use_sim = self.get_parameter('use_sim').value
        robot_ip = self.get_parameter('kachaka_ip').value
        self.use_native_map = self.get_parameter('use_native_map').value

        if self.use_sim:
            self.driver = RosSimDriver(self)
        else:
            self.get_logger().info(f"Mode: Real Robot. Connecting to {robot_ip}...")
            self.driver = KachakaRealDriver(robot_ip)

        self._action_server = ActionServer(
            self, Navigate, '/navigate_to_pose', 
            self.execute_callback, 
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Ready (Optimized RRT).")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Goal Received.')

        # 1. MAP HANDLING (Direct Read)
        tmp_dir = os.path.join(os.path.expanduser('~'), 'robot_ws', 'data', 'tmp')
        os.makedirs(tmp_dir, exist_ok=True)
        map_png = os.path.join(tmp_dir, 'kachaka_native.png')
        map_yaml = os.path.join(tmp_dir, 'kachaka_native.yaml')

        if self.use_native_map and not self.use_sim:
            dl_img, dl_yaml = self.driver.export_native_map()
            if dl_img: 
                map_png = dl_img
                map_yaml = dl_yaml

        # 2. READ YAML METADATA (No conversion needed)
        try:
            with open(map_yaml, 'r') as f:
                map_data = yaml.safe_load(f)
            resolution = map_data['resolution']
            origin = map_data['origin'] # [x, y, z]
        except Exception as e:
            self.get_logger().error(f"Map YAML read failed: {e}")
            goal_handle.abort()
            return Navigate.Result(success=False)

        # 3. PLANNING
        start_x, start_y, _ = self.driver.get_pose()
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        self.get_logger().info(f"start point: {start_x, start_y}")
        self.get_logger().info(f"start point: {target_x, target_y}")
        self.get_logger().info("Running RRT...")
        
        # PASS RAW DATA TO LIB
        path_points = rrt_lib.rrt_planning(
            start=(start_x, start_y), 
            target=(target_x, target_y), 
            map_path=map_png,
            resolution=resolution,
            origin=origin
        )

        if not path_points:
            self.get_logger().error("RRT failed/No path.")
            goal_handle.abort()
            return Navigate.Result(success=False)

        # 4. EXECUTION
        self.get_logger().info(f"Path found: {len(path_points)} waypoints.")
        for i, (wp_x, wp_y) in enumerate(path_points):
            self.get_logger().info(f" >> Waypoint {i}: ({wp_x:.2f}, {wp_y:.2f})")
            if not self.move_straight_to_goal(wp_x, wp_y):
                self.driver.stop()
                goal_handle.abort()
                return Navigate.Result(success=False)

        self.driver.stop()
        goal_handle.succeed()
        return Navigate.Result(success=True)

    def move_straight_to_goal(self, target_x, target_y):
        # ... (Keep your existing controller logic here) ...
        # (Copy-paste the move_straight_to_goal function from previous response)
        dist_tolerance = 0.15
        angle_tolerance = 0.10
        timeout_start = time.time()
        
        while rclpy.ok():
            if time.time() - timeout_start > 20.0: return False
            curr_x, curr_y, curr_theta = self.driver.get_pose()
            dx = target_x - curr_x; dy = target_y - curr_y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < dist_tolerance: return True
            
            desired_theta = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(desired_theta - curr_theta), math.cos(desired_theta - curr_theta))
            
            cmd_lin = 0.0; cmd_ang = 0.0
            if abs(angle_error) > angle_tolerance:
                cmd_ang = 0.6 if angle_error > 0 else -0.6
            else:
                cmd_lin = 0.25; cmd_ang = angle_error * 0.8
            
            self.driver.set_velocity(cmd_lin, cmd_ang)
            time.sleep(0.05)
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ModularNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()