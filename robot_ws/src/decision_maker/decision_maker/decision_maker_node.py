import threading
import queue
import time
import math
import json
from typing import List, Optional
import yaml
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from kachaka_interfaces.action import Navigate
from mm_interface.action import TaskCommand
from object_query_interfaces.srv import ObjectQuery

# Assuming these exist in your package
from .command_types import Command
from .scenario_library import SCENARIO_REGISTRY
from .nl_planner import WorldModel
from .occupancy_grid import (
    GoalSearchConfig,
    OccupancyGrid,
    select_goal_around_point,
)


class MapVisualizer:
    """Thread-safe OpenCV visualizer with a dedicated GUI loop thread.
    Keeps an internal display buffer which other threads update via methods.
    The GUI thread performs cv2.imshow/cv2.waitKey to ensure proper display.
    """
    def __init__(self, map_yaml_path: str, logger, rotate_clockwise_90: bool = True):
        import threading as _th
        self.logger = logger
        self.rotate_clockwise_90 = bool(rotate_clockwise_90)
        if not os.path.exists(map_yaml_path):
            raise FileNotFoundError(map_yaml_path)

        with open(map_yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.resolution = float(cfg.get('resolution', 0.015))
        self.origin = cfg.get('origin', [0.0, 0.0, 0.0])

        map_dir = os.path.dirname(map_yaml_path)
        image_file = cfg.get('image')
        if image_file is None:
            raise ValueError('map yaml missing image field')

        image_path = os.path.join(map_dir, image_file)
        if not os.path.exists(image_path):
            raise FileNotFoundError(image_path)

        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError(f'failed to load map image {image_path}')

        # Normalize image to 3-channel BGR if necessary
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 4:
            # drop alpha by converting to BGR
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        self.source_h, self.source_w = img.shape[:2]
        if self.rotate_clockwise_90:
            self.map_image = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        else:
            self.map_image = img
        self.h, self.w = self.map_image.shape[:2]
        self.window_name = 'map_visualizer'

        # thread-safe buffer and controls
        self._lock = _th.Lock()
        self._buffer = self.map_image.copy()
        self._stop = _th.Event()

        # start GUI thread
        self._thread = _th.Thread(target=self._gui_loop, daemon=True)
        self._thread.start()
        rotation_msg = 'clockwise_90' if self.rotate_clockwise_90 else 'none'
        self.logger.info(
            f'📍 MapVisualizer initialized: {self.w}x{self.h}, '
            f'res={self.resolution:.3f}m/px, rotation={rotation_msg}'
        )

    def world_to_pixel(self, x: float, y: float) -> tuple:
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        py = self.source_h - py
        if self.rotate_clockwise_90:
            px, py = self.source_h - 1 - py, px
        return px, py

    def reset(self):
        with self._lock:
            self._buffer = self.map_image.copy()

    def draw_marker(self, x: float, y: float, label: str = '', color=(0, 255, 255), radius: int = 10):
        px, py = self.world_to_pixel(x, y)
        with self._lock:
            if 0 <= px < self.w and 0 <= py < self.h:
                cv2.circle(self._buffer, (px, py), radius, color, -1)
                cv2.circle(self._buffer, (px, py), radius + 2, (255, 255, 255), 2)
                if label:
                    cv2.putText(self._buffer, label, (px + 12, py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(self._buffer, label, (px + 12, py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                self.logger.info(f"Drew '{label}' at world({x:.2f},{y:.2f}) -> pixel({px},{py})")
                return True
            else:
                self.logger.warn(f"Marker out of bounds: world({x:.2f},{y:.2f}) -> pixel({px},{py})")
                return False

    def _gui_loop(self):
        # create window in this thread
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        while not self._stop.is_set():
            with self._lock:
                frame = self._buffer.copy()
            cv2.imshow(self.window_name, frame)
            # small wait to process GUI events
            key = cv2.waitKey(50)
            if key == 27:  # ESC to close
                self._stop.set()
                break
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def stop(self):
        try:
            self._stop.set()
            if hasattr(self, '_thread'):
                self._thread.join(timeout=1.0)
        except Exception:
            pass


class DecisionMakingNode(Node):
    def __init__(self, node_name: str = 'decision_making_node'):
        super().__init__(node_name)

        # ====== Core setup ======
        self.cmd_queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self.world = WorldModel(node=self)

        # ====== 🛠️ MAP CALIBRATION CONFIG (3D->2D YAML) 🛠️ ======
        # Load 3D->2D transform parameters from YAML
        self.declare_parameter('map3d_to_map2d_yaml', 'data/Util/alignment.yaml')       # 
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

        # ====== Semantic instance bounding boxes ======
        self.declare_parameter(
            'instance_semantic_table_path',
            'data/lab/demo/robot_deploy_robot_run_instance_semantic_table_fine.json',
        )
        self.declare_parameter('nav_bbox_clearance', 0.10)
        self.declare_parameter('nav_bbox_match_tolerance', 0.05)
        instance_table_path = self.get_parameter(
            'instance_semantic_table_path'
        ).get_parameter_value().string_value
        self._nav_bbox_clearance = max(
            0.0,
            self.get_parameter('nav_bbox_clearance').get_parameter_value().double_value,
        )
        self._nav_bbox_match_tolerance = max(
            0.0,
            self.get_parameter('nav_bbox_match_tolerance').get_parameter_value().double_value,
        )
        self._semantic_instances = []
        if instance_table_path:
            if not os.path.isabs(instance_table_path):
                instance_table_path = os.path.abspath(instance_table_path)
            self._load_semantic_instances(instance_table_path)

        # ====== Occupancy-map-aware goal selection ======
        # Occupancy map built from the aligned 3D point cloud
        # (tools/build_occupancy_map.py); goals are picked from its free space
        # with a goal-to-obstacle distance inside [min, max] clearance.
        self.declare_parameter('occupancy_map_yaml', 'data/lab/demo/occupancy_from_pcd.yaml')
        self.declare_parameter('nav_min_obstacle_clearance', 0.30)
        self.declare_parameter('nav_max_obstacle_clearance', 0.45)
        self.declare_parameter('nav_max_standoff', 1.0)
        occupancy_map_yaml = self.get_parameter(
            'occupancy_map_yaml'
        ).get_parameter_value().string_value
        self._occupancy_grid = None
        self._goal_search_cfg = None
        if occupancy_map_yaml:
            if not os.path.isabs(occupancy_map_yaml):
                occupancy_map_yaml = os.path.abspath(occupancy_map_yaml)
            if os.path.exists(occupancy_map_yaml):
                try:
                    self._occupancy_grid = OccupancyGrid.from_yaml(occupancy_map_yaml)
                    self.get_logger().info(
                        f'✅ Loaded occupancy map for goal selection: {occupancy_map_yaml} '
                        f'({self._occupancy_grid.width}x{self._occupancy_grid.height})'
                    )
                except Exception as e:
                    self._occupancy_grid = None
                    self.get_logger().error(
                        f'❌ Failed loading occupancy map {occupancy_map_yaml}: {e}; '
                        'falling back to bbox-only goal selection.'
                    )
            else:
                self.get_logger().warn(
                    f'⚠️ Occupancy map not found: {occupancy_map_yaml}; '
                    'falling back to bbox-only goal selection.'
                )
        if self._occupancy_grid is not None:
            min_clearance = max(
                0.0,
                self.get_parameter(
                    'nav_min_obstacle_clearance'
                ).get_parameter_value().double_value,
            )
            self._goal_search_cfg = GoalSearchConfig(
                min_clearance=min_clearance,
                max_clearance=max(
                    min_clearance,
                    self.get_parameter(
                        'nav_max_obstacle_clearance'
                    ).get_parameter_value().double_value,
                ),
                max_standoff=max(
                    0.1,
                    self.get_parameter(
                        'nav_max_standoff'
                    ).get_parameter_value().double_value,
                ),
            )

        # ====== Map visualizer (direct OpenCV display) ======
        # Optional parameter to point to map yaml (PNG must be next to it).
        # Keep the visualizer available, but allow headless deployments to skip
        # creating an OpenCV/Qt window entirely.
        try:
            self.declare_parameter('enable_map_visualizer', True)
            self.declare_parameter('map_yaml', 'data/lab/kachaka_native.yaml')
            self.declare_parameter('map_rotate_clockwise_90', True)
            self.declare_parameter('map_nav_goal_marker_radius', 6)
            enable_map_visualizer = self.get_parameter(
                'enable_map_visualizer'
            ).get_parameter_value().bool_value
            map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
            rotate_map = self.get_parameter('map_rotate_clockwise_90').get_parameter_value().bool_value
            self._nav_goal_marker_radius = max(
                1,
                self.get_parameter('map_nav_goal_marker_radius').get_parameter_value().integer_value,
            )
            if map_yaml and not os.path.isabs(map_yaml):
                map_yaml = os.path.abspath(map_yaml)
            if enable_map_visualizer and map_yaml and os.path.exists(map_yaml):
                try:
                    self.visualizer = MapVisualizer(
                        map_yaml,
                        self.get_logger(),
                        rotate_clockwise_90=rotate_map,
                    )
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to init MapVisualizer: {e}")
                    self.visualizer = None
            else:
                self.visualizer = None
        except Exception:
            self.visualizer = None
            self._nav_goal_marker_radius = 6
        
        # ====== Grasp approach threshold ======
        self.declare_parameter('grasp_approach_dist', 0.3)   # set the arm to reach the object within 0.3 meters
        self._grasp_threshold = self.get_parameter('grasp_approach_dist').get_parameter_value().double_value
        self._nav_distance_remaining = float('inf')          # update by nav feedback to know how far we are from the target, used for grasp approach logic

        # ====== Navigation heading / grasp approach ======
        # /user_pose is published by modular_nav and shares the frame accepted by
        # /Navigate_to_pose.  It lets us choose a goal heading that faces the target.
        self.declare_parameter('robot_pose_topic', '/user_pose')
        self.declare_parameter('nav_approach_standoff_distance', 0.65)
        self.declare_parameter('nav_complete_grasp_goal', True)
        self._robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value
        self._nav_approach_standoff_distance = max(
            0.0,
            self.get_parameter('nav_approach_standoff_distance').get_parameter_value().double_value,
        )
        self._nav_complete_grasp_goal = self.get_parameter(
            'nav_complete_grasp_goal'
        ).get_parameter_value().bool_value
        self._robot_pose_lock = threading.Lock()
        self._robot_xy: Optional[tuple] = None

        # ====== ROS entities ======
        self.sub_manual = self.create_subscription(String, '/manual_command', self.on_text_event, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_command', self.on_cancel_event, 10)
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            self._robot_pose_topic,
            self._on_robot_pose,
            10,
        )
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.preview_goal_pub = self.create_publisher(String, '/semantic_preview/current_goal', 10)

        # ====== Action clients ======
        self.nav_client = ActionClient(self, Navigate, '/Navigate_to_pose')         #change to similar name but not the same name with nav2's topic
        self.task_client = ActionClient(self, TaskCommand, '/task_command')

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
    def _on_robot_pose(self, msg: PoseStamped) -> None:
        """Keep the latest robot position in the navigation action's input frame."""
        with self._robot_pose_lock:
            self._robot_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

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

            # Run scenario planning in a background thread so that any
            # blocking service calls (ObjectQuery) do not block the
            # rclpy executor thread and prevent service responses.
            def _plan_and_enqueue():
                try:
                    if argument_str:
                        primitives = scenario_fn(self.world, argument_str)
                    else:
                        try:
                            primitives = scenario_fn(self.world)
                        except TypeError:
                            raise ValueError(f"Command '{matched_key}' requires a target.")
                    self.enqueue_command(text, primitives)
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to plan command '{text}': {e}")

            t = threading.Thread(target=_plan_and_enqueue, daemon=True)
            t.start()

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
            self._reset_preview_path(reason=f"batch_start:{name}")
            self._clear_preview_goal(reason=f"batch_start:{name}")

            success = self._execute_batch(name, actions)
            if success:
                self.get_logger().info(f"✅ Finished batch: {name}")
                self._clear_preview_goal(reason="batch_complete")
            else:
                self.get_logger().warn(f"❌ Batch '{name}' failed or timed out.")
                self._clear_preview_goal(reason="batch_failed")

            self.cmd_queue.task_done()

    def _execute_batch(self, name: str, actions: List[str], timeout_sec: float = 900.0) -> bool:
        start_time = time.time()
        for i, act in enumerate(actions):
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"⏰ Timeout: '{name}' exceeded {timeout_sec}s, cancelling.")
                self._send_cancel()
                return False

            act = act.strip().lower()
            if act.startswith('goto:'):
                self.get_logger().info(f"📍 Executing {act}")
                next_act = actions[i + 1].strip().lower() if i + 1 < len(actions) else ''
                for_grasp = next_act.startswith('grasp:')
                if not self._execute_nav(act, for_grasp=for_grasp):
                    return False
            elif act.startswith('grasp:'):
                self.get_logger().info(f"✋ Executing {act}")
                if not self._execute_grasp(act):
                    return False
            elif act.startswith('place:'):
                self.get_logger().info(f"📦 Executing {act}")
                if not self._execute_place(act):
                    return False
            elif act.startswith('handover:'):
                self.get_logger().info(f"🤝 Executing {act}")
                if not self._execute_handover(act):
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
        
        mu, e1, e2, normal_n, s, R, t = self.map2d_params
        xy = map3d_point_to_map2d_xy((x_in, y_in, z_in), mu, e1, e2, s, R, t)
        return (float(xy[0]), float(xy[1]))

    def _apply_transform_3d(self, x_in: float, y_in: float, z_in: float = 0.0) -> tuple:
        """Converts raw 3D semantic-map coordinates into Kachaka map-frame (x, y, z)."""
        if self.map2d_params is None:
            self.get_logger().warn('⚠️ No 3D->2D params loaded, returning raw (x,y,z)')
            return (x_in, y_in, z_in)

        mu, e1, e2, normal_n, s, R, t = self.map2d_params
        x_map, y_map, z_map = map3d_point_to_map_frame((x_in, y_in, z_in), mu, e1, e2, normal_n, s, R, t)
        return (float(x_map), float(y_map), float(z_map))

    def _load_semantic_instances(self, instance_table_path: str) -> None:
        """Load centers and bounding boxes used to build object-specific nav goals."""
        if not os.path.exists(instance_table_path):
            self.get_logger().warn(
                f'⚠️ Instance semantic table not found: {instance_table_path}; '
                'bbox-aware navigation disabled.'
            )
            return

        try:
            with open(instance_table_path, 'r') as f:
                data = json.load(f)

            instances = data.get('instances', [])
            if isinstance(instances, dict):
                instance_iter = instances.items()
            elif isinstance(instances, list):
                instance_iter = (
                    (str(inst.get('inst_id', i)), inst)
                    for i, inst in enumerate(instances)
                    if isinstance(inst, dict)
                )
            else:
                raise ValueError(
                    f'unsupported "instances" type: {type(instances).__name__}'
                )

            records = []
            for fallback_id, inst in instance_iter:
                if not isinstance(inst, dict) or inst.get('filtered', False):
                    continue

                center = inst.get('center') or inst.get('centroid')
                bbox_min = inst.get('bbox_min')
                bbox_max = inst.get('bbox_max')
                try:
                    center_xyz = tuple(float(v) for v in center[:3])
                    bbox_min_xyz = tuple(float(v) for v in bbox_min[:3])
                    bbox_max_xyz = tuple(float(v) for v in bbox_max[:3])
                except (TypeError, ValueError):
                    continue

                values = np.asarray(
                    center_xyz + bbox_min_xyz + bbox_max_xyz,
                    dtype=float,
                )
                if not np.all(np.isfinite(values)):
                    continue

                records.append(
                    {
                        'instance_id': str(inst.get('inst_id', fallback_id)),
                        'name': str(
                            inst.get('semantic_name')
                            or inst.get('class_name')
                            or inst.get('category_name')
                            or inst.get('label')
                            or 'object'
                        ).strip().lower(),
                        'center': center_xyz,
                        'bbox_min': bbox_min_xyz,
                        'bbox_max': bbox_max_xyz,
                    }
                )

            self._semantic_instances = records
            self.get_logger().info(
                f'✅ Loaded {len(records)} semantic instance bounding boxes from: '
                f'{instance_table_path}'
            )
        except Exception as e:
            self._semantic_instances = []
            self.get_logger().error(
                f'❌ Failed loading semantic instance bounding boxes: {e}'
            )

    def _find_semantic_instance(self, raw_target_xyz: tuple) -> Optional[dict]:
        """Match a planner/query center back to its semantic-table instance."""
        if not self._semantic_instances:
            return None

        target = np.asarray(raw_target_xyz, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return None

        best_record = None
        best_distance = float('inf')
        for record in self._semantic_instances:
            distance = float(
                np.linalg.norm(target - np.asarray(record['center'], dtype=float))
            )
            if distance < best_distance:
                best_record = record
                best_distance = distance

        if best_distance > self._nav_bbox_match_tolerance:
            return None
        return best_record

    def _bbox_to_nav_footprint(
        self,
        bbox_min: tuple,
        bbox_max: tuple,
    ) -> Optional[tuple]:
        """Project a 3D AABB into an oriented rectangular 2D map footprint.

        The eight AABB corners are projected into the fitted ground-plane basis.
        Their 2D extents define the rectangle, which is then transformed by the
        calibrated Sim(2) into the navigation frame.
        """
        bbox_min_np = np.asarray(bbox_min, dtype=float)
        bbox_max_np = np.asarray(bbox_max, dtype=float)
        if (
            bbox_min_np.shape != (3,)
            or bbox_max_np.shape != (3,)
            or not np.all(np.isfinite(bbox_min_np))
            or not np.all(np.isfinite(bbox_max_np))
        ):
            return None

        low = np.minimum(bbox_min_np, bbox_max_np)
        high = np.maximum(bbox_min_np, bbox_max_np)
        corners_3d = np.asarray(
            [
                [x, y, z]
                for x in (low[0], high[0])
                for y in (low[1], high[1])
                for z in (low[2], high[2])
            ],
            dtype=float,
        )

        if self.map2d_params is None:
            projected = corners_3d[:, :2]
            projected_min = np.min(projected, axis=0)
            projected_max = np.max(projected, axis=0)
            footprint = np.asarray(
                [
                    [projected_min[0], projected_min[1]],
                    [projected_max[0], projected_min[1]],
                    [projected_max[0], projected_max[1]],
                    [projected_min[0], projected_max[1]],
                ],
                dtype=float,
            )
        else:
            mu, e1, e2, _normal_n, s, rotation, translation = self.map2d_params
            offsets = corners_3d - mu
            plane_points = np.column_stack((offsets @ e1, offsets @ e2))
            plane_min = np.min(plane_points, axis=0)
            plane_max = np.max(plane_points, axis=0)
            plane_rectangle = np.asarray(
                [
                    [plane_min[0], plane_min[1]],
                    [plane_max[0], plane_min[1]],
                    [plane_max[0], plane_max[1]],
                    [plane_min[0], plane_max[1]],
                ],
                dtype=float,
            )
            footprint = s * (plane_rectangle @ rotation.T) + translation

        edge_0 = float(np.linalg.norm(footprint[1] - footprint[0]))
        edge_1 = float(np.linalg.norm(footprint[3] - footprint[0]))
        if edge_0 <= 1e-4 or edge_1 <= 1e-4:
            return None
        return tuple((float(point[0]), float(point[1])) for point in footprint)

    def _semantic_footprint_for_target(
        self,
        raw_target_xyz: tuple,
    ) -> tuple:
        """Return the matched semantic record and its navigation-frame footprint."""
        record = self._find_semantic_instance(raw_target_xyz)
        if record is None:
            return None, None
        footprint = self._bbox_to_nav_footprint(
            record['bbox_min'],
            record['bbox_max'],
        )
        if footprint is None:
            return None, None
        return record, footprint

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize a yaw angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _get_robot_xy(self) -> Optional[tuple]:
        with self._robot_pose_lock:
            return self._robot_xy

    @staticmethod
    def _make_bbox_clearance_goal(
        target_x: float,
        target_y: float,
        robot_xy: tuple,
        footprint: tuple,
        clearance: float,
    ) -> tuple:
        """Place a goal outside the nearest rectangle edge and face its center."""
        points = np.asarray(footprint, dtype=float)
        if points.shape != (4, 2):
            raise ValueError('bbox footprint must contain four 2D corners')

        rectangle_center = np.mean(points, axis=0)
        edge_u = points[1] - points[0]
        edge_v = points[3] - points[0]
        half_u = float(np.linalg.norm(edge_u)) / 2.0
        half_v = float(np.linalg.norm(edge_v)) / 2.0
        if half_u <= 1e-4 or half_v <= 1e-4:
            raise ValueError('bbox footprint is degenerate')

        unit_u = edge_u / (2.0 * half_u)
        unit_v = edge_v / (2.0 * half_v)
        robot = np.asarray(robot_xy, dtype=float)
        relative_robot = robot - rectangle_center
        robot_u = float(relative_robot @ unit_u)
        robot_v = float(relative_robot @ unit_v)

        clamped_u = float(np.clip(robot_u, -half_u, half_u))
        clamped_v = float(np.clip(robot_v, -half_v, half_v))
        outside = abs(robot_u) > half_u or abs(robot_v) > half_v

        if outside:
            boundary = (
                rectangle_center
                + clamped_u * unit_u
                + clamped_v * unit_v
            )
            outward = robot - boundary
            outward_norm = float(np.linalg.norm(outward))
        else:
            distance_u = half_u - abs(robot_u)
            distance_v = half_v - abs(robot_v)
            if distance_u <= distance_v:
                sign_u = 1.0 if robot_u >= 0.0 else -1.0
                boundary = (
                    rectangle_center
                    + sign_u * half_u * unit_u
                    + robot_v * unit_v
                )
                outward = sign_u * unit_u
            else:
                sign_v = 1.0 if robot_v >= 0.0 else -1.0
                boundary = (
                    rectangle_center
                    + robot_u * unit_u
                    + sign_v * half_v * unit_v
                )
                outward = sign_v * unit_v
            outward_norm = 1.0

        if outward_norm <= 1e-6:
            raise ValueError('cannot determine bbox outward direction')
        outward = outward / outward_norm
        goal = boundary + max(0.0, float(clearance)) * outward
        goal_theta = math.atan2(target_y - goal[1], target_x - goal[0])
        return (
            float(goal[0]),
            float(goal[1]),
            float(goal_theta),
            (float(boundary[0]), float(boundary[1])),
        )

    def _make_navigation_goal(
        self,
        target_x: float,
        target_y: float,
        requested_theta: Optional[float],
        bbox_footprint: Optional[tuple] = None,
    ) -> tuple:
        """Return goal position, heading, whether stand-off was used, and its mode.

        The goal is chosen from the occupancy map's free space on rings around
        the projected target point: candidates must keep a goal-to-obstacle
        distance inside ``[nav_min_obstacle_clearance,
        nav_max_obstacle_clearance]`` and directly face the target surface.
        The heading always points at the projected target.  Semantic bboxes
        are intentionally NOT used (unreliable extent/orientation);
        ``bbox_footprint`` is accepted for compatibility but ignored.
        Without a map, the legacy fixed center stand-off remains (``fixed``).
        """
        goal_theta = (
            self._normalize_angle(requested_theta)
            if requested_theta is not None
            else None
        )

        robot_xy = self._get_robot_xy()

        if self._occupancy_grid is not None:
            selection = select_goal_around_point(
                self._occupancy_grid,
                (target_x, target_y),
                robot_xy,
                self._goal_search_cfg,
            )
            if selection is not None:
                goal_x, goal_y = selection['x'], selection['y']
                goal_theta = math.atan2(target_y - goal_y, target_x - goal_x)
                frontal = selection.get('frontal_hit')
                self.get_logger().info(
                    "🗺️ Free-space goal (occ_ring): (%.2f, %.2f), "
                    "obstacle clearance %.2f m, ring radius %.2f m, "
                    "facing surface at %s (%d candidates evaluated)"
                    % (
                        goal_x,
                        goal_y,
                        selection['clearance'],
                        selection['standoff'],
                        f"{frontal:.2f} m" if frontal is not None else 'n/a',
                        selection['candidates_checked'],
                    )
                )
                return goal_x, goal_y, goal_theta, True, 'occ_ring', None

            self.get_logger().warn(
                '⚠️ No free-space goal candidate satisfied the occupancy '
                'constraints; falling back to the fixed stand-off.'
            )

        if goal_theta is None and robot_xy is not None:
            dx = target_x - robot_xy[0]
            dy = target_y - robot_xy[1]
            if math.hypot(dx, dy) > 1e-4:
                goal_theta = math.atan2(dy, dx)

        goal_x, goal_y = target_x, target_y
        standoff_applied = (
            goal_theta is not None
            and self._nav_approach_standoff_distance > 0.0
        )
        if standoff_applied:
            goal_x -= self._nav_approach_standoff_distance * math.cos(goal_theta)
            goal_y -= self._nav_approach_standoff_distance * math.sin(goal_theta)

        mode = 'fixed' if standoff_applied else 'none'
        return goal_x, goal_y, goal_theta, standoff_applied, mode, None

    def _publish_preview_goal(
        self,
        *,
        event: str,
        label: str,
        action: str,
        x: float,
        y: float,
        z: float,
        for_grasp: bool = False,
    ) -> None:
        payload = {
            "event": event,
            "label": label,
            "action": action,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "for_grasp": bool(for_grasp),
            "stamp_sec": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.preview_goal_pub.publish(msg)

    def _clear_preview_goal(self, reason: str = "") -> None:
        payload = {
            "event": "clear",
            "reason": reason,
            "stamp_sec": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.preview_goal_pub.publish(msg)

    def _reset_preview_path(self, reason: str = "") -> None:
        payload = {
            "event": "reset_path",
            "reason": reason,
            "stamp_sec": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.preview_goal_pub.publish(msg)

    # =============================================================
    # OBJECT QUERY WRAPPER
    # =============================================================
    def _query_object_position(self, object_name: str, timeout_sec: float = 30.0) -> Optional[tuple]:
        """Return an object's navigation position followed by its raw 3D center."""
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
        nav_x, nav_y, nav_z = self._apply_transform_3d(raw_x, raw_y, raw_z)
        
        self.get_logger().info(f"✅ Found '{object_name}'")
        self.get_logger().info(f"   Raw 3D: ({raw_x:.2f}, {raw_y:.2f})")
        self.get_logger().info(f"   Nav 2D: ({nav_x:.2f}, {nav_y:.2f}, {nav_z:.2f})")
        
        # Visualize on desktop map window (direct OpenCV display)
        try:
            if hasattr(self, 'visualizer') and self.visualizer:
                # reset to base map and draw marker
                self.visualizer.reset()
                self.visualizer.draw_marker(
                    nav_x,
                    nav_y,
                    label=object_name,
                    color=(0, 255, 255),
                    radius=self._nav_goal_marker_radius,
                )
                # show briefly (non-blocking)
                # self.visualizer.show(wait_ms=1)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Visualization error: {e}")

        return (nav_x, nav_y, nav_z, raw_x, raw_y, raw_z)

    # =============================================================
    # NAV / GRASP / PLACE
    # =============================================================
    def _execute_nav(self, cmd: str, for_grasp: bool = False) -> bool:
        """
        Handles 'goto:x,y,z' or 'goto:x,y,z,yaw'.
        The first three coordinates are transformed from the semantic 3D map.
        Bbox-aware goals always face the object's center. An optional yaw in the
        navigation-map frame only overrides the heading for the fixed stand-off
        fallback used when no bbox is available.
        """
        # Reset stale distance so a previous nav's final reading cannot
        # trigger the threshold check at the start of this nav.
        self._nav_distance_remaining = float('inf')
        try:
            _, payload = cmd.split(':', 1)
            payload = payload.strip()
            
            x, y = 0.0, 0.0
            requested_theta: Optional[float] = None
            target_z = 0.0
            target_label = payload if payload else "nav_target"
            raw_target_xyz = None

            # CASE A: Planner sent semantic coordinates
            # (e.g., "goto:-0.27,1.76,-0.97" or "goto:-0.27,1.76,-0.97,1.57").
            if ',' in payload:
                parts = [float(v) for v in payload.split(',')]
                if len(parts) not in (3, 4):
                    raise ValueError(
                        "goto coordinate payload must be x,y,z or x,y,z,yaw"
                    )
                raw_x, raw_y, raw_z = parts[0], parts[1], parts[2]
                if len(parts) == 4:
                    requested_theta = parts[3]
                raw_target_xyz = (raw_x, raw_y, raw_z)

                # The planner sends raw 3D coordinates. Convert them to navigation coordinates.
                x, y, target_z = self._apply_transform_3d(raw_x, raw_y, raw_z)
                target_label = "nav_target"
                
                self.get_logger().info(f"🔄 Transformed: ({raw_x:.2f}, {raw_y:.2f}, {raw_z:.2f}) -> ({x:.2f}, {y:.2f})")
            
            # CASE B: Object Name "goto:chair" (Fallback if planner didn't resolve it)
            else:
                self.get_logger().info(f"🔍 Looking up coordinates for '{payload}'...")
                pos = self._query_object_position(payload) # This method already transforms
                if not pos:
                    self._clear_preview_goal(reason=f"query_failed:{payload}")
                    return False
                x, y, target_z, raw_x, raw_y, raw_z = pos
                raw_target_xyz = (raw_x, raw_y, raw_z)
                target_label = payload

            bbox_record = None
            bbox_footprint = None
            if raw_target_xyz is not None:
                bbox_record, bbox_footprint = self._semantic_footprint_for_target(
                    raw_target_xyz
                )
            if bbox_record is not None:
                # Use the table's full-precision center for both the goal heading
                # and preview, even if fmt_goto rounded the transmitted center.
                x, y, target_z = self._apply_transform_3d(*bbox_record['center'])
                target_label = (
                    f"{bbox_record['name']}[{bbox_record['instance_id']}]"
                )

            target_x, target_y = x, y
            # The bbox footprint is no longer used for goal selection (only
            # drawn for debugging); the goal comes from the occupancy map
            # around the projected target point.
            (
                x,
                y,
                target_theta,
                standoff_applied,
                standoff_mode,
                bbox_boundary,
            ) = self._make_navigation_goal(
                target_x,
                target_y,
                requested_theta,
            )
            if target_theta is None and self._nav_approach_standoff_distance > 0.0:
                self.get_logger().warn(
                    "⚠️ No robot pose yet; navigating directly to the target, "
                    "so the stand-off distance was not applied."
                )
            elif standoff_mode == 'fixed':
                self.get_logger().info(
                    "📏 Applying fallback %.2f m fixed stand-off: "
                    "target=(%.2f, %.2f), "
                    "nav_goal=(%.2f, %.2f)"
                    % (
                        self._nav_approach_standoff_distance,
                        target_x,
                        target_y,
                        x,
                        y,
                    )
                )

            # Show the actual navigation endpoint and its final heading.
            try:
                if hasattr(self, 'visualizer') and self.visualizer:
                    self.visualizer.reset()
                    if bbox_footprint is not None:
                        footprint_pixels = np.asarray(
                            [
                                self.visualizer.world_to_pixel(point[0], point[1])
                                for point in bbox_footprint
                            ],
                            dtype=np.int32,
                        ).reshape((-1, 1, 2))
                        with self.visualizer._lock:
                            cv2.polylines(
                                self.visualizer._buffer,
                                [footprint_pixels],
                                True,
                                (0, 165, 255),
                                2,
                            )
                        self.visualizer.draw_marker(
                            target_x,
                            target_y,
                            label='OBJECT_CENTER',
                            color=(0, 255, 255),
                            radius=self._nav_goal_marker_radius,
                        )
                    marker_label = (
                        'FREESPACE_GOAL'
                        if standoff_mode == 'occ_ring'
                        else 'STANDOFF_GOAL'
                        if standoff_applied
                        else 'NAV_TARGET'
                    )
                    self.visualizer.draw_marker(
                        x,
                        y,
                        label=marker_label,
                        color=(0, 0, 255),
                        radius=self._nav_goal_marker_radius,
                    )
                    if target_theta is not None:
                        end_px, end_py = self.visualizer.world_to_pixel(
                            x + 0.5 * math.cos(target_theta),
                            y + 0.5 * math.sin(target_theta),
                        )
                        start_px, start_py = self.visualizer.world_to_pixel(x, y)
                        with self.visualizer._lock:
                            cv2.arrowedLine(
                                self.visualizer._buffer,
                                (start_px, start_py),
                                (end_px, end_py),
                                (0, 0, 255),
                                3,
                                tipLength=0.3,
                            )
            except Exception as e:
                self.get_logger().warn(f"⚠️ Navigation target visualization error: {e}")

            self._publish_preview_goal(
                event="active",
                label=target_label,
                action=cmd,
                x=x,
                y=y,
                z=target_z,
                for_grasp=for_grasp,
            )

            # --- SEND TO NAV2 ---
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("❌ Nav2 server not available.")
                self._clear_preview_goal(reason="nav_server_unavailable")
                return False

            goal = Navigate.Goal()
            goal.target_x = float(x)
            goal.target_y = float(y)
            goal.target_theta = float(target_theta) if target_theta is not None else 0.0
            goal.has_target_theta = target_theta is not None

            yaw_text = f", yaw={target_theta:.2f} rad" if target_theta is not None else ", yaw=auto"
            self.get_logger().info(f"🚀 Sending Nav Goal: ({x:.2f}, {y:.2f}{yaw_text})")
            
            fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
            start = time.time()
            while not fut.done():
                if time.time() - start > 10.0:
                    self.get_logger().error("⏰ NAV goal send timeout")
                    self._clear_preview_goal(reason="nav_goal_send_timeout")
                    return False
                time.sleep(0.05)

            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("NAV goal rejected.")
                self._clear_preview_goal(reason="nav_goal_rejected")
                return False

            res_future = gh.get_result_async()
            start = time.time()
            threshold_triggered = False
            while not res_future.done():
                if time.time() - start > 900.0:
                    self.get_logger().warn("⏰ NAV timeout")
                    self._send_cancel()
                    self._clear_preview_goal(reason="nav_timeout")
                    return False
                
                # Finish the navigation goal by default so the navigation server
                # can complete its final turn.  The legacy early-exit behavior is
                # opt-in for deployments that deliberately need it.
                if (
                    for_grasp
                    and not self._nav_complete_grasp_goal
                    and self._nav_distance_remaining < self._grasp_threshold
                ):
                    self.get_logger().info(f"Within grasp threshold ({self._nav_distance_remaining:.2f}m), proceeding to grasp.")
                    gh.cancel_goal_async()
                    threshold_triggered = True
                    self._publish_preview_goal(
                        event="arrived",
                        label=target_label,
                        action=cmd,
                        x=x,
                        y=y,
                        z=target_z,
                        for_grasp=for_grasp,
                    )
                    break
                time.sleep(0.1)

            if not threshold_triggered:             # check whether the robot is close enough to the object
                nav_result = res_future.result().result
                if not nav_result.success:
                    self.get_logger().warn(f"❌ NAV failed: {nav_result.message}")
                    self._clear_preview_goal(reason="nav_failed")
                    return False
            # reset flag for next invocation
            threshold_triggered = False
            self._publish_preview_goal(
                event="arrived",
                label=target_label,
                action=cmd,
                x=x,
                y=y,
                z=target_z,
                for_grasp=for_grasp,
            )
            self.get_logger().info("✅ NAV success.")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ NAV error: {e}")
            self._clear_preview_goal(reason="nav_exception")
            return False

    def _send_task_command(self, command: str, label: str, timeout_sec: float = 900.0) -> bool:
        """Send a TaskCommand goal and wait for the result. Used by both grasp and place."""
        try:
            if not self.task_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"❌ TaskCommand server not available for {label}.")
                return False

            goal = TaskCommand.Goal()
            goal.command = command

            fut = self.task_client.send_goal_async(goal, feedback_callback=self._on_task_feedback)
            start = time.time()
            while not fut.done():
                if time.time() - start > 30.0:
                    self.get_logger().error(f"⏰ {label} goal send timeout")
                    return False
                time.sleep(0.05)

            handle = fut.result()
            if not handle.accepted:
                self.get_logger().warn(f"{label} goal rejected.")
                return False

            res_future = handle.get_result_async()
            start = time.time()
            while not res_future.done():
                if time.time() - start > timeout_sec:
                    self.get_logger().warn(f"⏰ {label} timeout")
                    self._send_cancel()
                    return False
                time.sleep(0.1)

            result = res_future.result().result
            if not result.success:
                self.get_logger().warn(f"❌ {label} failed: {result.message}")
                return False

            self.get_logger().info(f"✅ {label} success: {result.message}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ {label} error: {e}")
            return False

    def _execute_grasp(self, cmd: str) -> bool:
        obj = cmd.split(':', 1)[1].strip()

        # pos = self._query_object_position(obj)
        # if not pos:
        #    self.get_logger().warn(f"⚠️ Skipping grasp — position unavailable for '{obj}'.")
        #    return False
        time.sleep(5.0)
        return self._send_task_command(f'grasp the {obj}', 'GRASP', timeout_sec=300.0)

    def _execute_place(self, cmd: str) -> bool:
        payload = cmd.split(':', 1)[1].strip()
        parts = [part.strip() for part in payload.split(':', 1)]
        if len(parts) == 2 and parts[0] and parts[1]:
            obj, dest = parts
            return self._send_task_command(f'place {obj} to {dest}', 'PLACE', timeout_sec=300.0)

        # Legacy primitive form: "place:<destination>".
        return self._send_task_command(f'place to {payload}', 'PLACE', timeout_sec=300.0)

    def _execute_handover(self, cmd: str) -> bool:
        obj = cmd.split(':', 1)[1].strip()
        return self._send_task_command(f'handover {obj}', 'HANDOVER', timeout_sec=300.0)

    # =============================================================
    # FEEDBACK / CANCEL / UTILITIES
    # =============================================================
    def _on_nav_feedback(self, fb): 
        dist = fb.feedback.distance_remaining
        self._nav_distance_remaining = dist
        self.get_logger().debug(f"NAV feedback: distance_remaining={dist:.2f} m")
    
    def _on_task_feedback(self, feedback_msg):
        state = feedback_msg.feedback.feedback
        self.get_logger().info(f"🤖 Task feedback: {state}")
    
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
        try:
            if hasattr(self, 'visualizer') and self.visualizer:
                self.visualizer.stop()
        except Exception:
            pass
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
    normal_n = np.array(d["plane_fit"]["normal_n"], dtype=float)

    s = float(d["sim2"]["s"])
    R = np.array(d["sim2"]["R"], dtype=float)  # 2x2
    t = np.array([d["sim2"]["t"]["x"], d["sim2"]["t"]["y"]], dtype=float)

    return mu, e1, e2, normal_n, s, R, t


def map3d_point_to_map2d_xy(p_xyz, mu, e1, e2, s, R, t):
    """Project 3D point onto 2D plane, then apply similarity transform."""
    p = np.array(p_xyz, dtype=float)
    d = p - mu
    uv = np.array([d @ e1, d @ e2], dtype=float)
    xy = s * (R @ uv) + t
    return xy  # (x, y)


def map3d_point_to_map_frame(p_xyz, mu, e1, e2, normal_n, s, R, t):
    """Project 3D point into aligned Kachaka map frame and keep scaled height."""
    p = np.array(p_xyz, dtype=float)
    d = p - mu
    uv = np.array([d @ e1, d @ e2], dtype=float)
    height = float(d @ normal_n)
    xy = s * (R @ uv) + t
    z = s * height
    return float(xy[0]), float(xy[1]), float(z)
