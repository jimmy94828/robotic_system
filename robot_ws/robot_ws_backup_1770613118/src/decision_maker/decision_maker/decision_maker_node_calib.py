import threading
import queue
import time
import math
from typing import List, Optional

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
        super().__init__('decision_making_node_calib')

        # ====== Core setup ======
        self.cmd_queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self.world = WorldModel()

        # ====== 🛠️ MAP CALIBRATION CONFIG 🛠️ ======
        # Your specific values are applied here (legacy affine)
        self.tx = 4.4          # Translation X
        self.ty = 0.1           # Translation Y
        self.yaw = 0.593       # Rotation (radians)
        self.scale_x = 1.0      # Scale X
        self.scale_y = -1.0     # Scale Y (Flip Y axis)

        # Optional: provide a calibration matrix (flat list) via ROS parameter
        # 'lidar_to_map_matrix' (len=9 for 3x3 or len=16 for 4x4). If present,
        # we will use its inverse to map 3D map points back to 2D lidar coords.
        self.declare_parameter('lidar_to_map_matrix', [])
        # Optional: path to YAML file that contains T_lidar_to_camera
        self.declare_parameter('lidar_to_map_yaml', '')
        self._set_lidar_to_map_matrix_from_param()

        # If user provided a YAML path param, try to load it; otherwise attempt
        # to load from known calibration output path in workspace.
        try:
            yaml_path = self.get_parameter('lidar_to_map_yaml').value
            if yaml_path:
                self._load_lidar_to_map_from_yaml(yaml_path)
            else:
                # Fallback: check multiple likely locations for the YAML file
                import os as _os
                candidates = []
                # workspace-relative (cwd)
                candidates.append(_os.path.join(_os.getcwd(), 'data', 'calibration_test', 'calib_out_stage3', 'lidar_to_camera.yaml'))
                # relative to this package's path (robot_ws/src/decision_maker/... -> go up to robot_ws)
                base = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), '..', '..', '..'))
                candidates.append(_os.path.join(base, 'data', 'calibration_test', 'calib_out_stage3', 'lidar_to_camera.yaml'))
                # check each candidate
                for p in candidates:
                    if _os.path.exists(p):
                        self._load_lidar_to_map_from_yaml(p)
                        break
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to auto-load lidar YAML: {e}")
        # ==========================================

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
    # 📐 MAP TRANSFORM HELPER
    # =============================================================
    # NOTE: You may provide a calibration matrix (2D LIDAR -> 3D map).
    # If provided, we will invert it and use it to map 3D map points back
    # to 2D LIDAR frame coordinates. Supported shapes: 3x3 (2D affine) or
    # 4x4 (homogeneous 3D transform). The parameter name is
    # 'lidar_to_map_matrix' as a flat list (length 9 or 16).

    def _apply_transform(self, x_in: float, y_in: float) -> tuple:
        """Legacy: simple scale/rotate/translate mapping for 2D points.
        Kept for backwards compatibility when no calibration matrix is given.
        Converts 2D Map Coordinates -> 2D Navigation Coordinates
        """
        # 1. Flip (Scale)
        x_flipped = x_in * self.scale_x
        y_flipped = y_in * self.scale_y

        # 2. Rotate
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        x_rot = x_flipped * c - y_flipped * s
        y_rot = x_flipped * s + y_flipped * c

        # 3. Translate
        x_final = x_rot + self.tx
        y_final = y_rot + self.ty

        return x_final, y_final

    def _set_lidar_to_map_matrix_from_param(self):
        """Reads declared parameter 'lidar_to_map_matrix' (flat list) and
        stores it as numpy array in self.lidar2map_matrix if present and valid."""
        try:
            import numpy as _np
        except Exception:
            self.get_logger().warn("⚠️ numpy not available, calibration matrix support disabled.")
            self.lidar2map_matrix = None
            return

        try:
            vals = self.get_parameter('lidar_to_map_matrix').value
            if not vals:
                self.lidar2map_matrix = None
                return

            arr = _np.array(vals, dtype=float)
            if arr.size == 9:
                arr = arr.reshape((3, 3))
            elif arr.size == 16:
                arr = arr.reshape((4, 4))
            else:
                self.get_logger().warn("⚠️ 'lidar_to_map_matrix' must be length 9 or 16. Ignoring.")
                self.lidar2map_matrix = None
                return

            self.lidar2map_matrix = arr
            self.get_logger().info(f"✅ Loaded lidar_to_map_matrix shape={arr.shape}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to read 'lidar_to_map_matrix' parameter: {e}")
            self.lidar2map_matrix = None

    def set_lidar_to_map_matrix(self, mat: object):
        """Programmatic setter: accepts numpy array or flat list (len 9 or 16).
        Use this to inject calibration at runtime."""
        try:
            import numpy as _np
        except Exception:
            self.get_logger().error("❌ numpy required to use calibration matrix")
            return False

        arr = _np.array(mat, dtype=float)
        if arr.size == 9:
            arr = arr.reshape((3, 3))
        elif arr.size == 16:
            arr = arr.reshape((4, 4))
        else:
            self.get_logger().error("❌ Matrix must be 3x3 (len=9) or 4x4 (len=16).")
            return False

        self.lidar2map_matrix = arr
        self.get_logger().info(f"✅ Set lidar_to_map_matrix shape={arr.shape}")
        return True

    def _load_lidar_to_map_from_yaml(self, path: str) -> bool:
        """Loads a YAML file with 'T_lidar_to_camera' block and constructs a 4x4
        homogeneous matrix [R t; 0 1], then sets it as the lidar_to_map matrix.
        Returns True on success."""
        try:
            import yaml as _yaml
            import numpy as _np
            import os as _os
        except Exception:
            self.get_logger().error("❌ numpy/yaml required to load calibration YAML")
            return False

        if not _os.path.exists(path):
            self.get_logger().warn(f"⚠️ Calibration YAML not found at {path}")
            return False

        try:
            with open(path, 'r') as fh:
                doc = _yaml.safe_load(fh)

            block = doc.get('T_lidar_to_camera', None)
            if not block:
                self.get_logger().warn("⚠️ 'T_lidar_to_camera' not found in YAML.")
                return False

            R_list = block.get('rotation_matrix', None)
            t_list = block.get('translation_xyz', None)
            if R_list is None or t_list is None:
                self.get_logger().warn("⚠️ YAML missing rotation_matrix or translation_xyz.")
                return False

            R = _np.array(R_list, dtype=float)
            # rotation_matrix was stored as nested list rows
            R = R.reshape((3, 3)) if R.size == 9 else R
            t = _np.array(t_list, dtype=float).reshape((3,))

            T = _np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = t

            self.set_lidar_to_map_matrix(T.flatten())
            self.get_logger().info(f"✅ Loaded T_lidar_to_camera from {path} and set as 4x4 matrix")
            return True
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to parse calibration YAML: {e}")
            return False

    def _map3d_to_lidar2d_with_matrix(self, x: float, y: float, z: float) -> Optional[tuple]:
        """If calibration matrix exists, compute inverse and map (x,y,z) in map frame
        to (u,v) in lidar 2D coordinates. Returns (u, v) or None on failure."""
        try:
            import numpy as _np
        except Exception:
            self.get_logger().warn("⚠️ numpy not available, cannot apply calibration matrix.")
            return None

        if not hasattr(self, 'lidar2map_matrix') or self.lidar2map_matrix is None:
            return None

        M = self.lidar2map_matrix
        # 3x3 case: treat lidar 2D homogeneous (u,v,1) -> map (x,y,1). Need inverse of top-left 2x2 + translation.
        try:
            if M.shape == (3, 3):
                # We have: [x_map; y_map; 1] = M @ [u; v; 1]
                # So [u; v; 1] = M_inv @ [x_map; y_map; 1]
                p_map = _np.array([x, y, 1.0], dtype=float)
                Minv = _np.linalg.inv(M)
                p_lidar = Minv @ p_map
                u, v = float(p_lidar[0]), float(p_lidar[1])
                return (u, v)

            elif M.shape == (4, 4):
                # We have: [x_map; y_map; z_map; 1] = M @ [u; v; 0; 1]
                # Solve for [u; v; 0; 1] approximately by inverting M and applying to [x,y,z,1]
                p_map = _np.array([x, y, z, 1.0], dtype=float)
                Minv = _np.linalg.inv(M)
                p_lidar_h = Minv @ p_map
                # Expect lidar z ~ 0 for 2D lidar (or close). We take u,v.
                u, v = float(p_lidar_h[0] / p_lidar_h[3]), float(p_lidar_h[1] / p_lidar_h[3])
                return (u, v)

            else:
                self.get_logger().warn("⚠️ Unsupported calibration matrix shape.")
                return None
        except Exception as e:
            self.get_logger().warn(f"⚠️ Calibration matrix inversion failed: {e}")
            return None

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

        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().warn(f"⏰ Object query timeout.")
                return None

        result = future.result()
        if result is None or not result.found:
            self.get_logger().warn(f"⚠️ Object '{object_name}' not found.")
            return None

        # --- TRANSFORM LOGIC ---
        # Get raw coordinates (x, y, z) from the 3D map (z may be 0)
        raw_x = result.position.x
        raw_y = result.position.y
        raw_z = getattr(result.position, 'z', 0.0)

        # --- NAV coordinates (legacy / required by nav stack) ---
        nav_x, nav_y = self._apply_transform(raw_x, raw_y)

        # --- LIDAR 2D coordinates via calibration matrix (if provided) ---
        lidar_uv = None
        if hasattr(self, 'lidar2map_matrix') and self.lidar2map_matrix is not None:
            lidar_uv = self._map3d_to_lidar2d_with_matrix(raw_x, raw_y, raw_z)
            if lidar_uv:
                self.get_logger().info(f"   LIDAR 2D (from calib): ({lidar_uv[0]:.3f}, {lidar_uv[1]:.3f})")
            else:
                self.get_logger().warn("⚠️ Calibration matrix present but mapping failed.")

        self.get_logger().info(f"✅ Found '{object_name}'")
        self.get_logger().info(f"   Raw 3D: ({raw_x:.2f}, {raw_y:.2f}, {raw_z:.2f})")
        self.get_logger().info(f"   Nav 2D: ({nav_x:.2f}, {nav_y:.2f})")

        # Return legacy nav triplet for compatibility; append lidar coords when available
        if lidar_uv:
            return (nav_x, nav_y, 0.0, lidar_uv[0], lidar_uv[1])
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
                if hasattr(self, 'lidar2map_matrix') and self.lidar2map_matrix is not None:
                    mapped = self._map3d_to_lidar2d_with_matrix(raw_x, raw_y, 0.0)
                    if mapped:
                        x, y = mapped
                        self.get_logger().info(f"🔄 Transformed via calib matrix: ({raw_x:.2f}, {raw_y:.2f}) -> ({x:.2f}, {y:.2f})")
                    else:
                        self.get_logger().warn("⚠️ Calibration matrix present but mapping failed; using legacy transform.")
                        x, y = self._apply_transform(raw_x, raw_y)
                else:
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