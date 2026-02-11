import threading
import queue
import time
import math
from typing import List, Optional
import yaml
import numpy as np
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from kachaka_interfaces.action import Navigate
from decision_maker_interfaces.action import GraspPlace
from object_query_interfaces.srv import ObjectQuery

# Assuming these exist in your package
from .command_types import Command
from .scenario_library import SCENARIO_REGISTRY
from .nl_planner import WorldModel

class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__('decision_making_node')

        # ====== Core setup ======
        self.cmd_queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self.world = WorldModel()

        # ====== 🛠️ MAP CALIBRATION CONFIG (3D->2D YAML) 🛠️ ======
        # Load 3D->2D transform parameters from YAML
        self.declare_parameter('map3d_to_map2d_yaml', 'data/Util/alignment.yaml')
        yaml_path = self.get_parameter('map3d_to_map2d_yaml').get_parameter_value().string_value
        
        self.map2d_params = None
        if yaml_path:
            # Support both absolute and relative paths
            if not os.path.isabs(yaml_path):
                # Try relative to workspace or current directory
                yaml_path = os.path.abspath(yaml_path)
            
            if os.path.exists(yaml_path):
                try:
                    self.map2d_params = load_map3d_to_map2d(yaml_path)
                    self.get_logger().info(f'✅ Loaded 3D->2D transform from: {yaml_path}')
                except Exception as e:
                    self.get_logger().error(f'❌ Failed loading 3D->2D yaml: {e}')
                    import traceback
                    traceback.print_exc()
            else:
                self.get_logger().error(f'❌ YAML file not found: {yaml_path}')
        else:
            self.get_logger().warn('⚠️ No map3d_to_map2d_yaml specified, 3D->2D transform disabled')
        # ==========================================================

        # ====== ROS entities ======
        self.sub_manual = self.create_subscription(String, '/manual_command', self.on_text_event, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_command', self.on_cancel_event, 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)

        # ====== Action clients ======
        self.nav_client = ActionClient(self, Navigate, '/Navigate_to_pose')         #change to similar name but not the same name with nav2's topic
        self.grasp_client = ActionClient(self, GraspPlace, '/grasp_place')

        # ====== Service client (object query) ======
        self.obj_client = self.create_client(ObjectQuery, '/object_query')

        # ====== Threads ======
        self.sensor_worker = threading.Thread(target=self.sensor_processing_loop, daemon=True)
        self.exec_worker = threading.Thread(target=self.command_executor_loop, daemon=True)
        self.sensor_worker.start()
        self.exec_worker.start()

        self.get_logger().info("🧭 DecisionMakingNode ready with 3D->2D Map Calibration.")

    # =============================================================
    # SENSOR LOOP (Removed spin_once to avoid conflict with MultiThreadedExecutor)
    # =============================================================
    def sensor_processing_loop(self):
        """Background thread for sensor data processing.
        Note: Actual spinning is handled by MultiThreadedExecutor in main().
        """
        rate = self.create_rate(10)
        while not self._shutdown.is_set():
            # Process sensor data here if needed
            # DO NOT call rclpy.spin_once() - causes race condition with executor
            rate.sleep()

    # =============================================================
    # MANUAL COMMAND HANDLER
    # =============================================================
    def on_text_event(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f"🗣 Received command: '{text}'")

        try:
            scenario_fn = None
            matched_key = None

            for key, fn in SCENARIO_REGISTRY.items():
                if text.startswith(key):
                    scenario_fn = fn
                    matched_key = key
                    break

            if not scenario_fn:
                raise ValueError(f"No matching scenario for '{text}'")

            argument_str = text[len(matched_key):].strip()

            if argument_str:
                primitives = scenario_fn(self.world, argument_str)
            else:
                try:
                    primitives = scenario_fn(self.world)
                except TypeError:
                    raise ValueError(f"Command '{matched_key}' requires a target.")

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

    def _execute_batch(self, name: str, actions: List[str], timeout_sec: float = 300.0) -> bool:
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
    # 📐 MAP TRANSFORM HELPER (3D->2D with plane_fit + sim2)
    # =============================================================
    def _apply_transform(self, x_in: float, y_in: float, z_in: float = 0.0) -> tuple:
        """Converts 3D Map Coordinates -> 2D Navigation Coordinates using YAML params"""
        if self.map2d_params is None:
            self.get_logger().warn('⚠️ No 3D->2D params loaded, returning raw (x,y)')
            return (x_in, y_in)
        
        mu, e1, e2, s, R, t = self.map2d_params
        xy = map3d_point_to_map2d_xy((x_in, y_in, z_in), mu, e1, e2, s, R, t)
        return (float(xy[0]), float(xy[1]))

    # =============================================================
    # OBJECT QUERY WRAPPER
    # =============================================================
    def _query_object_position(self, object_name: str, timeout_sec: float = 30.0) -> Optional[tuple]:
        """Request 3D position AND transform it to 2D Nav Frame."""
        if not self.obj_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("❌ ObjectQuery service not available.")
            return None

        req = ObjectQuery.Request()
        req.name = object_name
        future = self.obj_client.call_async(req)

        # Wait for result (MultiThreadedExecutor handles spinning)
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().warn(f"⏰ Object query timeout for '{object_name}'.")
                return None
            time.sleep(0.05)  # Small sleep to avoid busy waiting

        result = future.result()
        if result is None or not result.found:
            self.get_logger().warn(f"⚠️ Object '{object_name}' not found.")
            return None

        # --- TRANSFORM LOGIC ---
        # Get raw coordinates (x, y, z) from the 3D map
        raw_x = result.position.x
        raw_y = result.position.y
        raw_z = result.position.z
        
        # Apply the 3D->2D calibration (plane_fit + sim2)
        nav_x, nav_y = self._apply_transform(raw_x, raw_y, raw_z)
        
        self.get_logger().info(f"✅ Found '{object_name}'")
        self.get_logger().info(f"   Raw 3D: ({raw_x:.2f}, {raw_y:.2f})")
        self.get_logger().info(f"   Nav 2D: ({nav_x:.2f}, {nav_y:.2f})")
        
        return (nav_x, nav_y, 0.0)

    # =============================================================
    # NAV / GRASP / PLACE
    # =============================================================
    def _execute_nav(self, cmd: str) -> bool:
        """
        Handles 'goto:x,y,th'. 
        CRITICAL CHANGE: Now applies calibration transform to these coordinates too.
        """
        try:
            _, payload = cmd.split(':', 1)
            payload = payload.strip()
            
            x, y, th = 0.0, 0.0, 0.0

            # CASE A: Planner sent coordinates (e.g., "goto:-0.27, 1.76, -0.97")
            if ',' in payload:
                parts = [float(v) for v in payload.split(',')]
                raw_x, raw_y = parts[0], parts[1]
                if len(parts) > 2: 
                    # We typically don't transform theta unless the map is rotated 90/180 deg
                    # For now, we pass theta through, or you can add self.yaw to it if needed.
                    th = parts[2] 

                # === APPLY TRANSFORM HERE ===
                # The planner sends RAW 3D coordinates. We must convert to NAV coordinates.
                x, y = self._apply_transform(raw_x, raw_y)
                
                self.get_logger().info(f"🔄 Transformed: ({raw_x:.2f}, {raw_y:.2f}) -> ({x:.2f}, {y:.2f})")
            
            # CASE B: Object Name "goto:chair" (Fallback if planner didn't resolve it)
            else:
                self.get_logger().info(f"🔍 Looking up coordinates for '{payload}'...")
                pos = self._query_object_position(payload) # This method already transforms
                if not pos:
                    return False
                x, y, _ = pos
                th = 0.0 

            # --- SEND TO NAV2 ---
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav2 server not available.")
                return False

            goal = Navigate.Goal()
            goal.target_x = float(x)
            goal.target_y = float(y)
            # goal.target_theta = float(th) 

            self.get_logger().info(f"🚀 Sending Nav Goal: ({x:.2f}, {y:.2f})")
            
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
            obj = cmd.split(':', 1)[1].strip()
            
            # Check position before grasping (optional but good for debugging)
            pos = self._query_object_position(obj)
            
            if not pos:
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


# =============================================================
# 📐 3D -> 2D Map Transform Helpers
# =============================================================

def load_map3d_to_map2d(yaml_path: str):
    """Load 3D->2D transform parameters from YAML (plane_fit + sim2)."""
    d = yaml.safe_load(open(yaml_path, "r"))

    mu = np.array([d["plane_fit"]["mu"]["x"],
                   d["plane_fit"]["mu"]["y"],
                   d["plane_fit"]["mu"]["z"]], dtype=float)
    e1 = np.array(d["plane_fit"]["basis_e1"], dtype=float)
    e2 = np.array(d["plane_fit"]["basis_e2"], dtype=float)

    s = float(d["sim2"]["s"])
    R = np.array(d["sim2"]["R"], dtype=float)  # 2x2
    t = np.array([d["sim2"]["t"]["x"], d["sim2"]["t"]["y"]], dtype=float)

    return mu, e1, e2, s, R, t


def map3d_point_to_map2d_xy(p_xyz, mu, e1, e2, s, R, t):
    """Project 3D point onto 2D plane, then apply similarity transform."""
    p = np.array(p_xyz, dtype=float)
    d = p - mu
    uv = np.array([d @ e1, d @ e2], dtype=float)
    xy = s * (R @ uv) + t
    return xy  # (x, y)