import threading
import queue
import time
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.task import Future

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from decision_maker_interfaces.action import GraspPlace
from object_query_interfaces.srv import ObjectQuery

from .command_types import Command
from .scenario_library import SCENARIO_REGISTRY
from .nl_planner import WorldModel


class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__('decision_making_node_isaacsim')

        # ====== Core setup ======
        self.cmd_queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self.world = WorldModel()

        # ====== ROS entities ======
        self.sub_manual = self.create_subscription(String, '/manual_command', self.on_text_event, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_command', self.on_cancel_event, 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)

        # ====== Action clients ======
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.grasp_client = ActionClient(self, GraspPlace, '/grasp_place')

        # ====== Service client (object query) ======
        self.obj_client = self.create_client(ObjectQuery, 'object_query')

        # ====== Threads ======
        self.sensor_worker = threading.Thread(target=self.sensor_processing_loop, daemon=True)
        self.exec_worker = threading.Thread(target=self.command_executor_loop, daemon=True)
        self.sensor_worker.start()
        self.exec_worker.start()

        self.get_logger().info("🧭 DecisionMakingNode ready with object query integration.")

    # =============================================================
    # SENSOR LOOP
    # =============================================================
    def sensor_processing_loop(self):
        rate = self.create_rate(10)
        while not self._shutdown.is_set():
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()

    # =============================================================
    # MANUAL COMMAND HANDLER
    # =============================================================
    def on_text_event(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f"🗣 Received command: '{text}'")

        try:
            scenario_fn = None
            for key, fn in SCENARIO_REGISTRY.items():
                if text.startswith(key):
                    scenario_fn = fn
                    break

            if not scenario_fn:
                raise ValueError(f"No matching scenario for '{text}'")

            parts = text.split()
            if "give me" in text or "bring me" in text:
                item = parts[-1]
                primitives = scenario_fn(self.world, item)
            else:
                primitives = scenario_fn(self.world)

            self.enqueue_command(text, primitives)

        except Exception as e:
            self.get_logger().error(f"❌ Failed to interpret command: {e}")

    def enqueue_command(self, name: str, primitives: List[str]):
        try:
            batch = {"name": name, "actions": primitives, "timestamp": time.time()}
            self.cmd_queue.put(batch, timeout=0.2)
            self.get_logger().info(f"📦 Enqueued '{name}' → {primitives}")
        except queue.Full:
            self.get_logger().warn("⚠️ Command queue full. Dropping command.")

    # =============================================================
    # COMMAND EXECUTION LOOP
    # =============================================================
    def command_executor_loop(self):
        while not self._shutdown.is_set():
            try:
                batch = self.cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            name, actions = batch["name"], batch["actions"]
            self.get_logger().info(f"🚀 Executing batch: {name}")

            success = self._execute_batch(name, actions)
            if success:
                self.get_logger().info(f"✅ Finished batch: {name}")
            else:
                self.get_logger().warn(f"❌ Batch '{name}' failed or timed out.")

            self.cmd_queue.task_done()

    # =============================================================
    # BATCH EXECUTION
    # =============================================================
    def _execute_batch(self, name: str, actions: List[str], timeout_sec: float = 150.0) -> bool:
        start_time = time.time()
        for act in actions:
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"⏰ Timeout: '{name}' exceeded {timeout_sec}s, cancelling.")
                self._send_cancel()
                return False

            act = act.strip().lower()
            if act.startswith('goto:'):
                self.get_logger().info(f"📍 Executing {act}")
                if not self._execute_nav(act):
                    return False
            elif act.startswith('grasp:'):
                self.get_logger().info(f"✋ Executing {act}")
                if not self._execute_grasp(act):
                    return False
            elif act.startswith('place:'):
                self.get_logger().info(f"📦 Executing {act}")
                if not self._execute_place(act):
                    return False
            else:
                self.get_logger().warn(f"⚠️ Unknown action: {act}")
        return True

    # =============================================================
    # OBJECT QUERY WRAPPER
    # =============================================================
    def _query_object_position(self, object_name: str, timeout_sec: float = 150.0) -> Optional[tuple]:
        """Request 3D position of object_name from ObjectQuery service."""
        if not self.obj_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ ObjectQuery service not available.")
            return None

        req = ObjectQuery.Request()
        req.name = object_name
        future = self.obj_client.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().warn(f"⏰ Object query timeout after {timeout_sec}s.")
                return None

        if future.cancelled():
            self.get_logger().warn("⚠️ Object query was cancelled.")
            return None

        result = future.result()
        if result is None:
            self.get_logger().error("⚠️ No result from ObjectQuery service.")
            return None

        if result.found:
            pos = result.position
            self.get_logger().info(f"✅ Object '{object_name}' found at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            return (pos.x, pos.y, pos.z)
        else:
            self.get_logger().warn(f"⚠️ Object '{object_name}' not found.")
            return None

    # =============================================================
    # NAV / GRASP / PLACE
    # =============================================================
    def _execute_nav(self, cmd: str) -> bool:
        try:
            _, payload = cmd.split(':', 1)
            x, y, th = [float(v) for v in payload.split(',')]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(th / 2.0)
            pose.pose.orientation.w = math.cos(th / 2.0)

            if not self.nav_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error("❌ Nav2 server not available.")
                return False

            goal = NavigateToPose.Goal()
            goal.pose = pose
            fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("NAV goal rejected.")
                return False

            res_future = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res_future, timeout_sec=150.0)
            if not res_future.done():
                self.get_logger().warn("⏰ NAV timeout.")
                self._send_cancel()
                return False

            self.get_logger().info("✅ NAV success.")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ NAV error: {e}")
            return False

    def _execute_grasp(self, cmd: str) -> bool:
        try:
            obj = cmd.split(':', 1)[1]
            pos = self._query_object_position(obj)
            if pos:
                self.get_logger().info(f"📍 Using object position: {pos}")
            else:
                self.get_logger().warn(f"⚠️ Skipping grasp — position unavailable for '{obj}'.")
                return False

            if not self.grasp_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error("❌ GraspPlace server not available.")
                return False

            goal = GraspPlace.Goal()
            goal.object_id = obj
            goal.target_bin = ''
            fut = self.grasp_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().warn("GRASP rejected.")
                return False

            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future, timeout_sec=10.0)
            if not res_future.done():
                self.get_logger().warn("⏰ GRASP timeout.")
                self._send_cancel()
                return False

            result = res_future.result().result
            if not result.success:
                self.get_logger().warn(f"❌ GRASP failed: {result.message}")
                return False

            self.get_logger().info(f"✅ GRASP success: {result.message}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ GRASP error: {e}")
            return False

    def _execute_place(self, cmd: str) -> bool:
        self.get_logger().info(f"🧺 PLACE simulated for {cmd}")
        return True

    # =============================================================
    # FEEDBACK / CANCEL / UTILITIES
    # =============================================================
    def _on_nav_feedback(self, fb): ...
    def _send_cancel(self):
        msg = String()
        msg.data = "cancel"
        self.status_pub.publish(msg)
        self.get_logger().warn("🛑 Cancel broadcast sent.")

    def on_cancel_event(self, msg: String):
        if msg.data.strip().lower() == "cancel":
            self._send_cancel()

    def create_rate(self, hz: float):
        period = 1.0 / hz
        class _Rate:
            def __init__(self, p): self.p = p
            def sleep(self): time.sleep(self.p)
        return _Rate(period)

    def destroy_node(self):
        self._shutdown.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = DecisionMakingNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
