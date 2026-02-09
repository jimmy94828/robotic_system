import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
import asyncio


class MockNavServer(Node):
    def __init__(self):
        super().__init__('mock_nav_server')
        # Allow re-entrant callbacks
        self.cb_group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,  # <-- quan trọng
        )
        self.get_logger().info('Mock NAV server ready at /navigate_to_pose')

    def goal_cb(self, goal_request):
        self.get_logger().info('NAV goal received')
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        self.get_logger().info('Server received cancel request')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        feedback = NavigateToPose.Feedback()
        for i in range(2000):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('NAV goal canceled on server')
                return NavigateToPose.Result()
            # simulate progress
            feedback.current_pose = goal_handle.request.pose
            goal_handle.publish_feedback(feedback)
            time.sleep(0.005)
        goal_handle.succeed()
        result = NavigateToPose.Result()
        self.get_logger().info('NAV goal succeeded')
        return result

def main():
    rclpy.init()
    node = MockNavServer()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
