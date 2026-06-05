import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import String
from object_query_interfaces.srv import ObjectQuery
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import struct
import cv2
import numpy as np
import json
import os
import random
import re
import select
import sys
import threading
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')

        # === Declare parameters ===
        # self.declare_parameter('3dmap_path', 'data/lab/accumulated_gaussians.npz') predemo
        self.declare_parameter('3dmap_path', 'data/lab/test/robot_deploy_slam_example0_rgb_color.npz')

        # self.declare_parameter('map_path', 'data/lab/semantic_pcd_accumulated_gaussians.npz') predemo
        self.declare_parameter('map_path', 'data/lab/test/robot_deploy_slam_example0_sem_color_fine.npz')

        self.declare_parameter('semantic_path', 'data/lab/semantic_pcd_accumulated_gaussians_meta.json')
        # self.declare_parameter('instance_path', 'data/lab/accumulated_gaussians_instance_semantic_info.json') predemo
        self.declare_parameter('instance_path', 'data/lab/test/robot_deploy_slam_example0_instance_semantic_table_fine.json')

        self.declare_parameter('alignment_path', 'data/Util/alignment.yaml')
        self.declare_parameter('auto_align', False) 
        self.declare_parameter('choice_timeout_sec', 60.0)
        self.declare_parameter('bev_preview_enabled', True)
        self.declare_parameter('bev_tmp_dir', '/home/acm/robotic_agent/robotic_system/robot_ws/data/tmp')
        self.declare_parameter('bev_resolution', 0.015)
        self.declare_parameter('bev_margin_m', 0.5)
        self.declare_parameter('bev_max_size_px', 2200)
        self.declare_parameter('bev_point_stride', 4)
        self.declare_parameter('bev_z_mode', 'auto')
        self.declare_parameter('bev_z_offset', 0.0)
        self.declare_parameter('bev_min_z', -1000000.0)
        self.declare_parameter('bev_max_z', 1.8)

        map_3d_path = self.get_parameter('3dmap_path').get_parameter_value().string_value
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        sem_path = self.get_parameter('semantic_path').get_parameter_value().string_value
        instance_path = self.get_parameter('instance_path').get_parameter_value().string_value
        alignment_path = self.get_parameter('alignment_path').get_parameter_value().string_value
        auto_align = self.get_parameter('auto_align').get_parameter_value().bool_value
        self.choice_timeout_sec = (
            self.get_parameter('choice_timeout_sec').get_parameter_value().double_value
        )

        # === ROS Entities ===
        self.choice_callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            ObjectQuery,
            '/object_query',
            self.handle_query,
            callback_group=self.choice_callback_group,
        )
        self.pub_objects = self.create_publisher(String, '/object_list', 10)
        self.choice_pub = self.create_publisher(String, '/object_query_choice', 10)
        self.reply_sub = self.create_subscription(
            String,
            '/object_query_reply',
            self.handle_choice_reply,
            10,
            callback_group=self.choice_callback_group,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map_markers', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/map_pointcloud', 10)
        preview_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.candidate_pub = self.create_publisher(
            String,
            '/semantic_preview/object_candidates',
            preview_qos,
        )

        # === Data container ===
        self.object_db = defaultdict(list) 
        self.object_instance_ids = defaultdict(list)
        self._choice_lock = threading.Lock()
        self._choice_condition = threading.Condition(self._choice_lock)
        self._choice_replies = {}
        self._choice_counter = 0
        self.instance_records = []
        
        self.map_points = None
        self.map_colors = None
        self.map_3d_points = None
        self.map_3d_colors = None
        self.alignment = None
        self.load_alignment(alignment_path)

        # === Load Data ===
        if os.path.exists(instance_path):
            self.load_instance_map(instance_path, map_3d_path)
        else:
            self.get_logger().warn(f'⚠️ instance_path not found ({instance_path}), falling back to semantic map.')
            self.load_semantic_map(map_path, sem_path, auto_align, map_3d_path)

        # === Publish at startup ===
        self.publish_object_list()
        self.publish_point_cloud()
        # Note: We do NOT publish markers here anymore. 
        # Markers appear only after a query.

        # === Timer for continuous point cloud publishing ===
        self.pcl_timer = self.create_timer(1.0, self.publish_point_cloud)  # Publish every 1 second

        self.get_logger().info(f'✅ ObjectQuery service ready. Auto-align & Center: {auto_align}')


    def handle_choice_reply(self, msg: String):
        """Accept Avatar/user replies for object disambiguation.

        Supported payloads:
        - "0"
        - {"index": 0}
        - {"request_id": "table-1", "index": 0}
        """
        text = msg.data.strip()
        if not text:
            return

        request_id = None
        index = None
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                request_id = payload.get('request_id')
                index = payload.get('index', payload.get('choice'))
            else:
                index = payload
        except json.JSONDecodeError:
            request_id, index = self._parse_relaxed_choice_reply(text)

        try:
            selected_idx = int(index)
        except (TypeError, ValueError):
            self.get_logger().warn(
                "Invalid /object_query_reply payload. Expected '0', "
                "{\"index\":0}, or {\"request_id\":\"table-1\",\"index\":0}. "
                f"Got: {msg.data}"
            )
            return

        with self._choice_condition:
            key = str(request_id) if request_id else '__latest__'
            self._choice_replies[key] = selected_idx
            self._choice_condition.notify_all()

        self.get_logger().info(
            f"Received /object_query_reply: request_id={request_id or '<latest>'}, index={selected_idx}"
        )

    def _parse_relaxed_choice_reply(self, text: str):
        """Parse common non-JSON replies from UI layers.

        Accepts:
        - 0
        - {request_id: table-1, index: 0}
        - request_id=table-1,index=0
        """
        stripped = text.strip()
        if stripped.isdigit():
            return None, stripped

        request_match = re.search(
            r"request_id\s*[:=]\s*[\"']?([^,}\"']+)[\"']?",
            stripped,
        )
        index_match = re.search(
            r"(?:index|choice)\s*[:=]\s*[\"']?(\d+)[\"']?",
            stripped,
        )
        request_id = request_match.group(1).strip() if request_match else None
        index = index_match.group(1).strip() if index_match else None
        return request_id, index

    # --------------------------------------------------------------
    def load_alignment(self, alignment_path: str):
        if not os.path.exists(alignment_path):
            self.get_logger().warn(f'⚠️ alignment file not found: {alignment_path}')
            return

        try:
            with open(alignment_path, 'r') as f:
                data = yaml.safe_load(f)

            self.alignment = {
                'mu': np.array(
                    [
                        data['plane_fit']['mu']['x'],
                        data['plane_fit']['mu']['y'],
                        data['plane_fit']['mu']['z'],
                    ],
                    dtype=float,
                ),
                'e1': np.array(data['plane_fit']['basis_e1'], dtype=float),
                'e2': np.array(data['plane_fit']['basis_e2'], dtype=float),
                'normal': np.array(data['plane_fit']['normal_n'], dtype=float),
                'scale': float(data['sim2']['s']),
                'rot': np.array(data['sim2']['R'], dtype=float),
                'trans': np.array([data['sim2']['t']['x'], data['sim2']['t']['y']], dtype=float),
            }
            self.get_logger().info(f'Loaded 3D->map alignment from {alignment_path}')
        except Exception as e:
            self.alignment = None
            self.get_logger().warn(f'⚠️ Failed to load alignment file {alignment_path}: {e}')

    def transform_point_to_map(self, point_xyz):
        if self.alignment is None:
            return float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2])

        point = np.array(point_xyz, dtype=float)
        delta = point - self.alignment['mu']
        uv = np.array(
            [
                delta @ self.alignment['e1'],
                delta @ self.alignment['e2'],
            ],
            dtype=float,
        )
        height = float(delta @ self.alignment['normal'])
        xy = self.alignment['scale'] * (self.alignment['rot'] @ uv) + self.alignment['trans']
        z = self.alignment['scale'] * height
        return float(xy[0]), float(xy[1]), float(z)

    def transform_points_to_map(self, points_xyz):
        if self.alignment is None:
            return np.asarray(points_xyz, dtype=np.float32)

        points = np.asarray(points_xyz, dtype=float)
        delta = points - self.alignment['mu'][None, :]
        uv = np.column_stack(
            (
                delta @ self.alignment['e1'],
                delta @ self.alignment['e2'],
            )
        )
        height = delta @ self.alignment['normal']
        xy = self.alignment['scale'] * (uv @ self.alignment['rot'].T) + self.alignment['trans'][None, :]
        z = self.alignment['scale'] * height
        return np.column_stack((xy, z)).astype(np.float32)

    def publish_candidate_options(self, name: str, instances, instance_ids, request_id: str | None = None, preview_paths=None):
        payload = {
            'event': 'active',
            'name': name,
            'request_id': request_id,
            'reply_topic': '/object_query_reply',
            'options': [],
        }
        if preview_paths:
            payload['bev_preview'] = preview_paths
        for i, pos in enumerate(instances):
            map_x, map_y, map_z = self.transform_point_to_map(pos)
            payload['options'].append(
                {
                    'index': i,
                    'instance_id': instance_ids[i] if i < len(instance_ids) else '',
                    'raw_x': float(pos[0]),
                    'raw_y': float(pos[1]),
                    'raw_z': float(pos[2]),
                    'map_x': map_x,
                    'map_y': map_y,
                    'map_z': map_z,
                }
            )

        msg = String()
        msg.data = json.dumps(payload)
        self.candidate_pub.publish(msg)
        self.choice_pub.publish(msg)

    def clear_candidate_options(self):
        msg = String()
        msg.data = json.dumps({'event': 'clear'})
        self.candidate_pub.publish(msg)
        self.choice_pub.publish(msg)

    @staticmethod
    def _coerce_xyz(value):
        if value is None:
            return None
        try:
            if len(value) < 3:
                return None
            return (float(value[0]), float(value[1]), float(value[2]))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _safe_filename_token(text: str) -> str:
        token = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in text.strip().lower())
        return token or 'object'

    def _bbox_corners(self, bbox_min, bbox_max):
        mn = self._coerce_xyz(bbox_min)
        mx = self._coerce_xyz(bbox_max)
        if mn is None or mx is None:
            return None
        return np.array(
            [
                [mn[0], mn[1], mn[2]],
                [mn[0], mn[1], mx[2]],
                [mn[0], mx[1], mn[2]],
                [mn[0], mx[1], mx[2]],
                [mx[0], mn[1], mn[2]],
                [mx[0], mn[1], mx[2]],
                [mx[0], mx[1], mn[2]],
                [mx[0], mx[1], mx[2]],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _world_to_pixel_xy(xy, bounds, resolution, height):
        min_x, _, min_y, _ = bounds
        px = np.rint((xy[:, 0] - min_x) / resolution).astype(np.int32)
        py = height - 1 - np.rint((xy[:, 1] - min_y) / resolution).astype(np.int32)
        return px, py

    @staticmethod
    def _z_value_in_slice(z_value, slice_min_z, slice_max_z):
        return slice_min_z <= z_value <= slice_max_z

    @staticmethod
    def _choose_visual_z_sign(points_xyz: np.ndarray, z_mode: str) -> float:
        if z_mode == 'normal':
            return 1.0
        if z_mode == 'inverted':
            return -1.0

        z_values = points_xyz[:, 2]
        finite_z = z_values[np.isfinite(z_values)]
        if finite_z.size == 0:
            return 1.0
        return -1.0 if float(np.median(finite_z)) < 0.0 else 1.0

    @staticmethod
    def _rotate_pixel_clockwise(px: int, py: int, source_height: int):
        return source_height - 1 - py, px

    def export_bev_preview_for_selection(self, name: str, instances, instance_ids):
        if not self.get_parameter('bev_preview_enabled').value:
            return None
        if self.map_3d_points is None or self.map_3d_colors is None:
            self.get_logger().warn('BEV preview skipped: RGB map point cloud is not loaded.')
            return None

        tmp_dir = self.get_parameter('bev_tmp_dir').value
        os.makedirs(tmp_dir, exist_ok=True)

        stride = max(1, int(self.get_parameter('bev_point_stride').value))
        points = self.map_3d_points[::stride]
        colors = self.map_3d_colors[::stride]
        finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1]) & np.isfinite(points[:, 2])
        points = points[finite]
        colors = colors[finite]
        if points.shape[0] == 0:
            self.get_logger().warn('BEV preview skipped: RGB map has no finite points.')
            return None

        z_mode = str(self.get_parameter('bev_z_mode').value).strip().lower()
        if z_mode not in ('auto', 'normal', 'inverted'):
            self.get_logger().warn(f"Invalid bev_z_mode='{z_mode}', falling back to auto.")
            z_mode = 'auto'
        visual_z_sign = self._choose_visual_z_sign(points, z_mode)
        visual_z_offset = float(self.get_parameter('bev_z_offset').value)
        visual_z = (visual_z_sign * points[:, 2]) + visual_z_offset

        raw_min_z = float(self.get_parameter('bev_min_z').value)
        raw_max_z = float(self.get_parameter('bev_max_z').value)
        slice_min_z = min(raw_min_z, raw_max_z)
        slice_max_z = max(raw_min_z, raw_max_z)
        before_z_filter = points.shape[0]
        height_mask = (visual_z >= slice_min_z) & (visual_z <= slice_max_z)
        points = points[height_mask]
        colors = colors[height_mask]
        removed_by_z = before_z_filter - points.shape[0]
        if points.shape[0] == 0:
            self.get_logger().warn(
                f'BEV preview skipped: height slice z=[{slice_min_z:.2f}, {slice_max_z:.2f}] '
                'removed all RGB map points.'
            )
            return None

        xy_sources = [points[:, :2]]
        candidate_instance_map = []
        skipped_candidates_by_z = 0
        for i, inst in enumerate(instances):
            instance_id = str(instance_ids[i]) if i < len(instance_ids) else ''
            center_map = self.transform_point_to_map(inst)
            visual_center_z = (visual_z_sign * center_map[2]) + visual_z_offset
            if not self._z_value_in_slice(visual_center_z, slice_min_z, slice_max_z):
                skipped_candidates_by_z += 1
                continue
            center_xy = np.array([center_map[0], center_map[1]], dtype=np.float32)
            xy_sources.append(center_xy.reshape(1, 2))
            candidate_instance_map.append(
                {
                    'index': i,
                    'name': name,
                    'instance_id': instance_id,
                    'center_raw': inst,
                    'center_map': center_map,
                    'visual_z': visual_center_z,
                }
            )

        all_xy = np.vstack(xy_sources)
        margin = float(self.get_parameter('bev_margin_m').value)
        min_x = float(np.nanmin(all_xy[:, 0]) - margin)
        max_x = float(np.nanmax(all_xy[:, 0]) + margin)
        min_y = float(np.nanmin(all_xy[:, 1]) - margin)
        max_y = float(np.nanmax(all_xy[:, 1]) + margin)

        resolution = max(float(self.get_parameter('bev_resolution').value), 1e-4)
        max_size_px = max(64, int(self.get_parameter('bev_max_size_px').value))
        width = int(np.ceil((max_x - min_x) / resolution)) + 1
        height = int(np.ceil((max_y - min_y) / resolution)) + 1
        largest = max(width, height)
        if largest > max_size_px:
            resolution *= largest / max_size_px
            width = int(np.ceil((max_x - min_x) / resolution)) + 1
            height = int(np.ceil((max_y - min_y) / resolution)) + 1

        bounds = (min_x, max_x, min_y, max_y)
        image = np.full((height, width, 3), 18, dtype=np.uint8)
        px, py = self._world_to_pixel_xy(points[:, :2], bounds, resolution, height)
        in_bounds = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        image[py[in_bounds], px[in_bounds]] = colors[in_bounds][:, ::-1]

        annotated_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        rotated_height, rotated_width = annotated_image.shape[:2]
        color = (0, 255, 255)
        text_color = (0, 255, 255)
        radius = 7

        for rec in candidate_instance_map:
            center_xy = np.array([[rec['center_map'][0], rec['center_map'][1]]], dtype=np.float32)
            center_px, center_py = self._world_to_pixel_xy(center_xy, bounds, resolution, height)
            raw_cx = int(center_px[0])
            raw_cy = int(center_py[0])
            if 0 <= raw_cx < width and 0 <= raw_cy < height:
                cx, cy = self._rotate_pixel_clockwise(raw_cx, raw_cy, height)
                cx = int(np.clip(cx, 0, rotated_width - 1))
                cy = int(np.clip(cy, 0, rotated_height - 1))
                cv2.circle(annotated_image, (cx, cy), radius + 2, (0, 0, 0), -1)
                cv2.circle(annotated_image, (cx, cy), radius, color, -1)
                label = str(rec['index'])
                font_scale = 0.8
                text_thickness = 2
                outline_thickness = 4
                text_size, baseline = cv2.getTextSize(
                    label,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    text_thickness,
                )
                text_w, text_h = text_size
                label_x = cx + 9
                if label_x + text_w >= rotated_width:
                    label_x = cx - text_w - 9
                label_y = cy - 9
                label_x = int(np.clip(label_x, 0, max(0, rotated_width - text_w - 1)))
                label_y = int(np.clip(label_y, text_h + 2, max(text_h + 2, rotated_height - baseline - 2)))
                cv2.putText(
                    annotated_image,
                    label,
                    (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    (0, 0, 0),
                    outline_thickness,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    annotated_image,
                    label,
                    (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    text_color,
                    text_thickness,
                    cv2.LINE_AA,
                )

        # timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        name_token = self._safe_filename_token(name)
        # rgb_path = os.path.join(tmp_dir, f'{timestamp}_{name_token}_rgb_bev.png')
        overlay_path = os.path.join(tmp_dir, f'{name_token}_instances_bev.png')
        # metadata_path = os.path.join(tmp_dir, f'{timestamp}_{name_token}_bev_options.json')

        # if not cv2.imwrite(rgb_path, annotated_image):
        #     self.get_logger().warn(f'Failed to write RGB BEV image: {rgb_path}')
        if not cv2.imwrite(overlay_path, annotated_image):
            self.get_logger().warn(f'Failed to write instance BEV image: {overlay_path}')

        options = []
        for i, pos in enumerate(instances):
            map_x, map_y, map_z = self.transform_point_to_map(pos)
            options.append(
                {
                    'index': i,
                    'instance_id': instance_ids[i] if i < len(instance_ids) else '',
                    'raw_xyz': [float(pos[0]), float(pos[1]), float(pos[2])],
                    'map_xyz': [map_x, map_y, map_z],
                }
            )

        metadata = {
            'generated_at': datetime.now().isoformat(timespec='seconds'),
            'query_name': name,
            # 'rgb_bev_path': rgb_path,
            'instances_bev_path': overlay_path,
            # 'options_path': metadata_path,
            'resolution_m_per_px': resolution,
            'height_slice_z': {
                'min_z': slice_min_z,
                'max_z': slice_max_z,
                'visual_z_mode': z_mode,
                'visual_z_sign': visual_z_sign,
                'visual_z_offset': visual_z_offset,
                'rgb_points_removed': int(removed_by_z),
                'candidate_instances_removed': int(skipped_candidates_by_z),
            },
            'visual_rotation': 'clockwise_90_degrees',
            'bounds_xy': {
                'min_x': min_x,
                'max_x': max_x,
                'min_y': min_y,
                'max_y': max_y,
            },
            'candidate_options': options,
            'all_instances': [
                {
                    'name': rec['name'],
                    'index': rec['index'],
                    'instance_id': rec['instance_id'],
                    'center_map_xyz': [
                        float(rec['center_map'][0]),
                        float(rec['center_map'][1]),
                        float(rec['center_map'][2]),
                    ],
                    'visual_z': float(rec['visual_z']),
                }
                for rec in candidate_instance_map
            ],
        }
        # with open(metadata_path, 'w', encoding='utf-8') as f:
        #     json.dump(metadata, f, indent=2)

        # self.get_logger().info(f'Saved BEV RGB map: {rgb_path}')
        # self.get_logger().info(f'Saved BEV instance preview: {overlay_path}')
        return {
            # 'rgb_bev_path': rgb_path,
            'instances_bev_path': overlay_path,
            # 'metadata_path': metadata_path,
        }

    # --------------------------------------------------------------
    def load_instance_map(self, instance_path: str, map_3d_path: str):
        """Load object positions from instance JSON (centroid per instance). Also loads 3D point cloud."""
        try:
            with open(instance_path, 'r') as f:
                inst_data = json.load(f)

            instances = inst_data.get('instances', {})
            if not instances:
                self.get_logger().error(f'❌ No "instances" key found in {instance_path}')
                return

            self.object_db.clear()
            self.object_instance_ids.clear()
            self.instance_records.clear()

            if isinstance(instances, dict):
                instance_iter = instances.items()
                total_instances = len(instances)
            elif isinstance(instances, list):
                instance_iter = (
                    (str(inst.get('inst_id', i)), inst)
                    for i, inst in enumerate(instances)
                    if isinstance(inst, dict)
                )
                total_instances = len(instances)
            else:
                self.get_logger().error(
                    f'❌ Unsupported "instances" format in {instance_path}: {type(instances).__name__}'
                )
                return

            loaded_instances = 0
            skipped_instances = 0
            for inst_id, inst in instance_iter:
                if inst.get('filtered', False):
                    skipped_instances += 1
                    continue

                raw_name = (
                    inst.get('semantic_name')
                    or inst.get('class_name')
                    or inst.get('category_name')
                    or inst.get('label')
                    or 'unknown'
                )
                name = str(raw_name).strip().lower() or 'unknown'
                centroid = inst.get('centroid') or inst.get('center')

                if centroid is None and 'bbox_min' in inst and 'bbox_max' in inst:
                    bbox_min = inst.get('bbox_min')
                    bbox_max = inst.get('bbox_max')
                    if (
                        isinstance(bbox_min, list) and len(bbox_min) >= 3
                        and isinstance(bbox_max, list) and len(bbox_max) >= 3
                    ):
                        centroid = [
                            (float(bbox_min[0]) + float(bbox_max[0])) / 2.0,
                            (float(bbox_min[1]) + float(bbox_max[1])) / 2.0,
                            (float(bbox_min[2]) + float(bbox_max[2])) / 2.0,
                        ]

                if centroid is None or len(centroid) < 3:
                    skipped_instances += 1
                    continue

                center_xyz = (float(centroid[0]), float(centroid[1]), float(centroid[2]))
                self.object_db[name].append(center_xyz)
                self.object_instance_ids[name].append(str(inst_id))
                self.instance_records.append(
                    {
                        'name': name,
                        'instance_id': str(inst_id),
                        'center': center_xyz,
                        'bbox_min': self._coerce_xyz(inst.get('bbox_min')),
                        'bbox_max': self._coerce_xyz(inst.get('bbox_max')),
                    }
                )
                loaded_instances += 1

            self.get_logger().info(
                f'Loaded {loaded_instances}/{total_instances} instances '
                f'({skipped_instances} skipped), {len(self.object_db)} categories from {instance_path}')

            # Load 3D point cloud for visualization
            if os.path.exists(map_3d_path):
                data_3d = np.load(map_3d_path)
                self.map_3d_points = self.transform_points_to_map(data_3d['points'])
                if 'colors' in data_3d:
                    c = data_3d['colors']
                    self.map_3d_colors = (c * 255).astype(np.uint8) if c.max() <= 1.0 else c.astype(np.uint8)
                else:
                    self.map_3d_colors = np.full((self.map_3d_points.shape[0], 3), 128, dtype=np.uint8)
            else:
                self.get_logger().warn(f'⚠️ 3D map not found: {map_3d_path}')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load instance map: {e}')
            import traceback
            traceback.print_exc()

    # --------------------------------------------------------------
    def load_semantic_map(self, map_path: str, sem_path: str, auto_align: bool, map_3d_path: str):
        """Load semantic map with robust JSON parsing for 'segments_info'."""
        if not os.path.exists(map_path):
            self.get_logger().error(f'Map file not found: {map_path}')
            return
        if not os.path.exists(sem_path):
            self.get_logger().error(f'Semantic JSON not found: {sem_path}')
            return
        if not os.path.exists(map_3d_path):
            self.get_logger().error(f'3D map file not found: {map_3d_path}')
            return

        try:
            # 1. Load NPZ Data
            data = np.load(map_path)
            data_3d = np.load(map_3d_path)
            # semantic
            if 'pts' in data: points = data['pts']
            elif 'means3D' in data: points = data['means3D']
            else:
                self.get_logger().error(f"❌ Could not find 'pts' or 'means3D' in {map_path}")
                return

            if 'pan' in data: semantic_ids = data['pan']
            elif 'semantic_ids' in data: semantic_ids = data['semantic_ids']
            else:
                self.get_logger().error(f"❌ Could not find 'pan' or 'semantic_ids' in {map_path}")
                return
            # 3D points and colors
            points_3d = data_3d['points']
            if 'colors' in data_3d:
                colors_3d = data_3d['colors']
                if colors_3d.max() <= 1.0:
                    colors_3d = (colors_3d * 255).astype(np.uint8)
                else:
                    colors_3d = colors_3d.astype(np.uint8)
            else:
                # Default gray color if no colors available
                colors_3d = np.full((points_3d.shape[0], 3), 128, dtype=np.uint8)
            # 2. Load JSON & Build Mapping
            with open(sem_path, 'r') as f:
                sem_json = json.load(f)
            
            segments = []
            if isinstance(sem_json, list): segments = sem_json
            elif isinstance(sem_json, dict):
                if 'segments_info' in sem_json: segments = sem_json['segments_info']
                elif 'segmentation' in sem_json: segments = sem_json['segmentation']
                elif 'segments' in sem_json: segments = sem_json['segments']
            
            # 3. Build Name Mapping
            id_to_name = {}
            for seg in segments:
                seg_id = seg.get('id', None)
                if seg_id is None: continue
                name = seg.get('category_name', seg.get('class', seg.get('label', 'unknown')))
                id_to_name[seg_id] = name

            if not id_to_name:
                self.get_logger().error("❌ JSON loaded but no categories found.")
                return

            # 4. Auto-Align (apply same transformation to both semantic and 3D maps)
            if auto_align:
                rot_matrix = self.compute_alignment_matrix(points, semantic_ids, id_to_name)
                if rot_matrix is not None:
                    points = points @ rot_matrix.T
                    points_3d = points_3d @ rot_matrix.T
                    self.get_logger().info(f"✅ Applied alignment to both semantic and 3D maps")
            

            self.map_points = points
            self.map_3d_points = self.transform_points_to_map(points_3d)
            self.map_3d_colors = colors_3d

            # 5. Generate Colors
            if 'rgb' in data:
                raw_rgb = data['rgb']
                if raw_rgb.max() <= 1.0: self.map_colors = (raw_rgb * 255).astype(np.uint8)
                else: self.map_colors = raw_rgb.astype(np.uint8)
            else:
                unique_ids = np.unique(semantic_ids)
                color_map = {}
                for uid in unique_ids:
                    color_map[uid] = (random.randint(50, 255), random.randint(50, 255), random.randint(50, 255))
                self.map_colors = np.zeros((self.map_points.shape[0], 3), dtype=np.uint8)
                for uid, color in color_map.items():
                    self.map_colors[semantic_ids == uid] = color

            # 6. Build Object Database
            self.object_db.clear()
            self.object_instance_ids.clear()
            self.instance_records.clear()
            for sid, name in id_to_name.items():
                mask = semantic_ids == sid
                if np.any(mask):
                    pts = self.map_points[mask]
                    min_pt = np.min(pts, axis=0)
                    max_pt = np.max(pts, axis=0)
                    centroid = (min_pt + max_pt) / 2.0
                    key = name.lower()
                    self.object_db[key].append(tuple(centroid.tolist()))
                    instance_id = f'semantic_{sid}'
                    self.object_instance_ids[key].append(instance_id)
                    self.instance_records.append(
                        {
                            'name': key,
                            'instance_id': instance_id,
                            'center': tuple(centroid.tolist()),
                            'bbox_min': tuple(min_pt.tolist()),
                            'bbox_max': tuple(max_pt.tolist()),
                        }
                    )

            self.get_logger().info(f'✅ Built object DB with {len(self.object_db)} categories.')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load semantic map: {e}')
            import traceback
            traceback.print_exc()

    # --------------------------------------------------------------
    def compute_alignment_matrix(self, points, semantic_ids, id_to_name):
        """Compute rotation matrix to align floor to XY plane. Returns None if alignment fails."""
        floor_ids = []
        floor_keywords = ['floor', 'ground', 'carpet', 'tile', 'wood']
        
        for sid, name in id_to_name.items():
            if any(k in name.lower() for k in floor_keywords):
                floor_ids.append(sid)
        
        if not floor_ids:
            self.get_logger().warn("⚠️ No floor objects found for alignment")
            return None

        mask = np.isin(semantic_ids, floor_ids)
        floor_pts = points[mask]
        
        if len(floor_pts) < 50:
            self.get_logger().warn("⚠️ Not enough floor points for reliable alignment")
            return None

        self.get_logger().info(f"📐 Aligning map using {len(floor_pts)} floor points...")

        floor_mean = np.mean(floor_pts, axis=0)
        centered_floor = floor_pts - floor_mean
        u, s, vh = np.linalg.svd(centered_floor, full_matrices=False)
        normal = -vh[2, :] 

        target_axis = np.array([0, 0, 1])
        rot_axis = np.cross(normal, target_axis)
        rot_sin = np.linalg.norm(rot_axis)
        rot_cos = np.dot(normal, target_axis)

        if rot_sin > 1e-6:
            rot_axis = rot_axis / rot_sin
            angle = np.arccos(np.clip(rot_cos, -1.0, 1.0))
            r = R.from_rotvec(rot_axis * angle)
            rot_matrix = r.as_matrix()
            self.get_logger().info(f"   ↪ Computed rotation by {np.degrees(angle):.2f} degrees.")
            return rot_matrix
        else:
            self.get_logger().info("   ↪ Floor already aligned, no rotation needed.")
            return np.eye(3)

    # --------------------------------------------------------------
    def handle_query(self, request, response):
        """Handles query. Returns position and UPDATES visualization for that object only."""
        name = request.name.strip().lower()
        self.get_logger().info(f"Received ObjectQuery request: {name}")
        found, point = self.search_object(name)
        
        response.found = found
        response.position = Point(x=point.x, y=point.y, z=point.z)
        
        if found:
            response.message = f'Found {name} at ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})'
            # VISUALIZATION CHANGE: Show ONLY this object
            self.publish_object_marker(name)
        else:
            response.message = f'Object {name} not found.'
            # Optional: Clear markers if object not found
            self.clear_markers()
             
        self.get_logger().info(f'Query: {name} -> {response.message}')
        return response

    def search_object(self, name: str):
        if name in self.object_db:
            instances = self.object_db[name]
            instance_ids = self.object_instance_ids.get(name, [])
            if len(instances) > 1:
                self.get_logger().info(f"Multiple instances of '{name}' found. Waiting for user selection.")
                request_id = self._next_choice_request_id(name)
                preview_paths = self.export_bev_preview_for_selection(name, instances, instance_ids)
                self.publish_candidate_options(
                    name,
                    instances,
                    instance_ids,
                    request_id=request_id,
                    preview_paths=preview_paths,
                )
                if preview_paths:
                    self.get_logger().info(
                        f"Open BEV preview before selecting: {preview_paths['instances_bev_path']}"
                    )
                for i, inst in enumerate(instances):
                    instance_id = instance_ids[i] if i < len(instance_ids) else 'n/a'
                    map_x, map_y, map_z = self.transform_point_to_map(inst)
                    self.get_logger().info(
                        f"  [{i}] instance_id={instance_id} "
                        f"raw=({inst[0]:.2f}, {inst[1]:.2f}, {inst[2]:.2f}) "
                        f"map=({map_x:.2f}, {map_y:.2f}, {map_z:.2f})"
                    )
                selected_idx = self.wait_for_choice(name, request_id, len(instances))
                if selected_idx is None:
                    selected_idx = random.randrange(len(instances))
                    self.get_logger().info(f"Randomly selected index [{selected_idx}] for '{name}'.")
                self.clear_candidate_options()
                best_pt = instances[selected_idx]
            else:
                best_pt = min(instances, key=lambda p: p[0]**2 + p[1]**2 + p[2]**2)
                self.clear_candidate_options()
            return True, Point(x=best_pt[0], y=best_pt[1], z=best_pt[2])
        else:
            self.clear_candidate_options()
            return False, Point(x=0.0, y=0.0, z=0.0)



    def _next_choice_request_id(self, name: str) -> str:
        with self._choice_lock:
            self._choice_counter += 1
            return f"{name}-{self._choice_counter}"

    def wait_for_choice(self, name: str, request_id: str, option_count: int):
        prompt = (
            f"Multiple instances of '{name}' found. Enter index (0-{option_count - 1}) to select, "
            "or press Enter for random: "
        )
        try:
            print(prompt, end='', flush=True)
        except Exception:
            pass

        deadline = self.get_clock().now().nanoseconds / 1e9 + self.choice_timeout_sec
        with self._choice_condition:
            while True:
                for key in (request_id, '__latest__'):
                    if key in self._choice_replies:
                        idx = self._choice_replies.pop(key)
                        if 0 <= idx < option_count:
                            self.get_logger().info(
                                f"Selected index [{idx}] for '{name}' from /object_query_reply."
                            )
                            return idx
                        self.get_logger().warn(
                            f"Invalid /object_query_reply index {idx}; selecting randomly."
                        )
                        return None

                if self._terminal_has_line_ready():
                    text = sys.stdin.readline().strip()
                    if not text:
                        return None
                    if text.isdigit():
                        idx = int(text)
                        if 0 <= idx < option_count:
                            self.get_logger().info(
                                f"Selected index [{idx}] for '{name}' from terminal input."
                            )
                            return idx
                    self.get_logger().warn("Invalid terminal choice; selecting randomly.")
                    return None

                remaining = deadline - self.get_clock().now().nanoseconds / 1e9
                if remaining <= 0:
                    self.get_logger().warn(
                        f"Timed out waiting for '{name}' choice after {self.choice_timeout_sec:.1f}s."
                    )
                    return None
                self._choice_condition.wait(timeout=min(0.2, remaining))

    def _terminal_has_line_ready(self) -> bool:
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            return bool(readable)
        except Exception:
            return False

    def publish_object_list(self):
        serializable_db = {k: v for k, v in self.object_db.items()}
        msg = String()
        msg.data = json.dumps(serializable_db, indent=2)
        self.pub_objects.publish(msg)

    def clear_markers(self):
        """Helper to clear all markers from Rviz."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)

    def publish_object_marker(self, target_name: str):
        """
        Clears previous markers and displays ONLY the requested object.
        """
        marker_array = MarkerArray()
        
        # 1. DELETE ALL previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 2. Add new markers for the specific object
        if target_name in self.object_db:
            locations = self.object_db[target_name]
            now = self.get_clock().now().to_msg()
            id_counter = 0
            
            for i, pos in enumerate(locations):
                x, y, z = self.transform_point_to_map(pos)
                display_name = f"{target_name} ({i+1})" if len(locations) > 1 else target_name

                # Sphere
                sphere = Marker()
                sphere.header.frame_id = "map"; sphere.header.stamp = now
                sphere.ns = "semantic_objects"; sphere.id = id_counter; id_counter += 1
                sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
                sphere.pose.position.x = x; sphere.pose.position.y = y; sphere.pose.position.z = z
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.3; sphere.scale.y = 0.3; sphere.scale.z = 0.3
                sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0; sphere.color.a = 1.0 # Green for active query
                sphere.lifetime.sec = 0 # Persistent until next query clears it
                marker_array.markers.append(sphere)
                
                # Text
                text = Marker()
                text.header.frame_id = "map"; text.header.stamp = now
                text.ns = "semantic_names"; text.id = id_counter; id_counter += 1
                text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
                text.pose.position.x = x; text.pose.position.y = y; text.pose.position.z = z + 0.3
                text.pose.orientation.w = 1.0
                text.scale.z = 0.2
                text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
                text.text = display_name; text.lifetime.sec = 0
                marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def publish_point_cloud(self):
        # Publish 3D map point cloud instead of semantic map
        if self.map_3d_points is None or self.map_3d_colors is None:
            self.get_logger().warn("⚠️ 3D map not loaded, cannot publish point cloud", throttle_duration_sec=5.0)
            return
        points = self.map_3d_points
        colors = self.map_3d_colors
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1; msg.width = points.shape[0]
        msg.is_bigendian = False; msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        msg.point_step = 16; msg.row_step = msg.point_step * points.shape[0]
        buffer = []
        for i in range(points.shape[0]):
            x, y, z = points[i]
            r, g, b = colors[i]
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            buffer.append(struct.pack('ffff', x, y, z, rgb_float))
        msg.data = b''.join(buffer)
        self.pcl_pub.publish(msg)

def main():
    rclpy.init()
    node = ObjectQueryServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
