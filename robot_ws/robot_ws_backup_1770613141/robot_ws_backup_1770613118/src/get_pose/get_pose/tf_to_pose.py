#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

"""Usage: Convert TF transforms to PoseStamped messages."""

class TfToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 20.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f'Publishing PoseStamped: {self.target_frame} -> {self.base_frame} on /pose'
        )

    def on_timer(self):
        # 用 ROS time（bag play + use_sim_time 時會跟著 /clock 走）
        now = rclpy.time.Time()
        try:
            ts = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                now
            )
        except TransformException as e:
            self.get_logger().debug(f'No transform yet: {e}')
            return

        msg = PoseStamped()
        msg.header.stamp = ts.header.stamp
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = ts.transform.translation.x
        msg.pose.position.y = ts.transform.translation.y
        msg.pose.position.z = ts.transform.translation.z
        msg.pose.orientation = ts.transform.rotation

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TfToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()