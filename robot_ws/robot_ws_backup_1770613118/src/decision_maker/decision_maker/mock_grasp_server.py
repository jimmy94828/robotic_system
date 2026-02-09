import time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from decision_maker_interfaces.action import GraspPlace


class MockGraspServer(Node):
    def __init__(self):
        super().__init__('mock_grasp_place_server')
        self.server = ActionServer(
            self,
            GraspPlace,
            'grasp_place',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.get_logger().info('Mock Grasp/Place server ready at /grasp_place')

    def goal_cb(self, goal_request: GraspPlace.Goal):
        self.get_logger().info(
            f'GRASP goal: obj={goal_request.object_id} -> bin={goal_request.target_bin}')
        # bạn có thể filter object_id ở đây
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        self.get_logger().info('GRASP cancel requested')
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        fb = GraspPlace.Feedback()
        phases = ['approach', 'align', 'close_gripper', 'lift', 'move', 'open_gripper', 'done']
        for i, ph in enumerate(phases):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('GRASP goal canceled')
                return GraspPlace.Result(success=False, message='canceled')
            fb.state = ph
            fb.progress = (i+1)/len(phases)
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)
        goal_handle.succeed()
        return GraspPlace.Result(success=True, message='ok')

def main():
    rclpy.init()
    node = MockGraspServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
