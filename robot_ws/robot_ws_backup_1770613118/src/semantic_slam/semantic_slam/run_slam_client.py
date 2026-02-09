import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from semantic_slam_interfaces.action import RunSlam

class Client(Node):
    def __init__(self):
        super().__init__('run_slam_client')
        self.cli = ActionClient(self, RunSlam, 'run_slam')

    def send(self, output_uri='file:///tmp/gs_map', save_on_stop=True):
        goal = RunSlam.Goal()
        goal.output_uri = output_uri
        goal.save_on_stop = save_on_stop

        self.cli.wait_for_server()
        self.get_logger().info('Sending goal...')
        future = self.cli.send_goal_async(goal, feedback_callback=self.fb)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        result_future = handle.get_result_async()
        self.get_logger().info('Waiting for result...')
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(f'Result: success={res.success}, map_uri={res.map_uri}, num_splats={res.num_splats}')

    def fb(self, feedback_msg):
        self.get_logger().info(f'feedback num_splats={feedback_msg.feedback.num_splats}')

def main():
    rclpy.init()
    node = Client()
    node.send(output_uri='/mnt/HDD1/phudh/home_robot/subproject1/tmp/slam_map', save_on_stop=True)
    node.destroy_node()
    rclpy.shutdown()
