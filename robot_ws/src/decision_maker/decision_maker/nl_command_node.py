import sys
import select
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class NLCommandNode(Node):
    """
    Simplified NLCommandNode
    ------------------------
    - Sends natural language text to /manual_command
    - Displays feedback from /task_status
    - Allows typing commands via terminal
    """

    def __init__(self):
        super().__init__("nl_command_node")

        # Publishers / Subscribers
        self.pub_cmd = self.create_publisher(String, "/manual_command", 10)
        self.sub_feedback = self.create_subscription(String, "/task_status", self.on_feedback, 10)

        # Timers
        self._stdin_timer = self.create_timer(0.1, self._poll_stdin)

        self.get_logger().info("💬 NLCommandNode ready. Type natural commands (e.g. 'bring me apple').")

    # =========================================================
    # Send NL Command
    # =========================================================
    def send_command(self, text: str):
        msg = String()
        msg.data = text.strip()
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"📤 Sent command: '{text.strip()}'")

    # =========================================================
    # Feedback Listener
    # =========================================================
    def on_feedback(self, msg: String):
        feedback = msg.data.strip()
        if feedback.startswith("done"):
            self.get_logger().info(f"✅ Task completed: {feedback}")
        elif feedback.startswith("failed"):
            self.get_logger().warn(f"❌ Task failed: {feedback}")
        elif feedback.startswith("cancel"):
            self.get_logger().warn("🛑 Task cancelled.")
        else:
            self.get_logger().info(f"📬 Feedback: {feedback}")

    # =========================================================
    # Terminal Input
    # =========================================================
    def _poll_stdin(self):
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not r:
                return
            line = sys.stdin.readline().strip()
            if line:
                self.send_command(line)
        except Exception as e:
            self.get_logger().warn(f"stdin read error: {e}")


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()
    node = NLCommandNode()
    try:
        exe = MultiThreadedExecutor()
        exe.add_node(node)
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
