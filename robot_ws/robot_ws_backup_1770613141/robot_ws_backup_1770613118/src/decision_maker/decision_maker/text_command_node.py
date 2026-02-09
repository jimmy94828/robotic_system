import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextCommandNode(Node):
    """
    Node nhập lệnh bằng text từ terminal rồi publish lên 1 topic.
    Ví dụ lệnh:
      - goto:1.2,3.4,1.57
      - grasp:object_42
      - place:bin_A
    """
    def __init__(self):
        super().__init__('text_command_node')
        topic = self.declare_parameter('topic', '/manual_command').get_parameter_value().string_value
        self.pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f'TextCommandNode started. Publishing to {topic}')
        self.get_logger().info('Nhập lệnh rồi nhấn Enter (Ctrl+C để thoát).')

        # dùng timer để không block executor
        self.timer = self.create_timer(0.05, self._poll_stdin)

        # buffer đọc từng dòng
        self._buffer = ''

    def _poll_stdin(self):
        # đọc không chặn: nếu có gì trong stdin buffer thì gửi
        # đơn giản: dùng sys.stdin.readline() khi có sẵn input (tty)
        # cách nhẹ nhàng: nếu input có sẵn (interactive), readline sẽ block,
        # nên ta chỉ gọi khi có dữ liệu pending. Với TTY, ta dùng try/except.
        try:
            import select
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist:
                line = sys.stdin.readline()
                if not line:
                    return
                line = line.strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.pub.publish(msg)
                    self.get_logger().info(f'Đã gửi: "{line}"')
        except Exception as e:
            self.get_logger().warn(f'stdin read err: {e}')

def main():
    rclpy.init()
    node = TextCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
