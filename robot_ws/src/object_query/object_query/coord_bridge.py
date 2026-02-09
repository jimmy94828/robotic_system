import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from object_query_interfaces.srv import ObjectQuery
import math

class BridgeClient(Node):
    def __init__(self):
        super().__init__('coord_bridge_client')
        
        # === CALIBRATION (From your Manual Align Tool) ===
        self.tx = 4.40         # Translation X
        self.ty = 2.4        # Translation Y
        self.yaw = -0.611       # Rotation (radians)
        self.scale_x = 1.0     # 1.0 or -1.0
        self.scale_y = -1.0     # 1.0 or -1.0
        # =================================================

        # 1. Trigger (You send "chair" here)
        self.create_subscription(String, '/find_object', self.command_callback, 10)

        # 2. Service Client (Asks Object Node)
        self.cli = self.create_client(ObjectQuery, '/object_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Object Query Service...')

        # 3. Publisher (To Nav2)
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.get_logger().info("Bridge Ready. Publish object name to '/find_object' to start.")

    def command_callback(self, msg):
        obj_name = msg.data
        self.get_logger().info(f"Received command: Go to '{obj_name}'")
        self.send_request(obj_name)

    def send_request(self, name):
        req = ObjectQuery.Request()
        req.name = name
        
        # Call service asynchronously
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.found:
                # Get 3D Coordinates from Service Response
                x_3d = response.position.x
                y_3d = response.position.y # Assuming Y is depth/up in your output
                # NOTE: If your output uses Z as forward/up, switch to response.position.z
                
                self.process_and_publish(x_3d, y_3d)
            else:
                self.get_logger().warn(f"Object not found: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def process_and_publish(self, x_in, y_in):
        # 1. Flip
        x_flipped = x_in * self.scale_x
        y_flipped = y_in * self.scale_y

        # 2. Rotate
        x_rot = x_flipped * math.cos(self.yaw) - y_flipped * math.sin(self.yaw)
        y_rot = x_flipped * math.sin(self.yaw) + y_flipped * math.cos(self.yaw)
        
        # 3. Translate
        x_final = x_rot + self.tx
        y_final = y_rot + self.ty
        
        # 4. Publish Goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x_final
        goal.pose.position.y = y_final
        goal.pose.orientation.w = 1.0 # Facing forward (neutral)
        
        self.nav_pub.publish(goal)
        self.get_logger().info(f"🚀 Sent Nav Goal: ({x_final:.2f}, {y_final:.2f})")

def main():
    rclpy.init()
    node = BridgeClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()