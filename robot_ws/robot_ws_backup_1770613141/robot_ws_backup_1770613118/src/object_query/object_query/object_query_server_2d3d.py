import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from object_query_interfaces.srv import ObjectQuery
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import json
import os
import random
import yaml  # NEW: To read your map config
import math  # NEW: For rotation math
from scipy.spatial.transform import Rotation as R
from collections import defaultdict

class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')

        # === Declare parameters ===
        self.declare_parameter('map_path', 'data/lab/panst3r_result_confge3_no_pts_local.npz')
        self.declare_parameter('semantic_path', 'data/lab/panst3r_meta.json')
        self.declare_parameter('auto_align', True)
        
        # NEW: Parameter for your 2D Map YAML
        self.declare_parameter('user_map_yaml', '') 

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        sem_path = self.get_parameter('semantic_path').get_parameter_value().string_value
        auto_align = self.get_parameter('auto_align').get_parameter_value().bool_value
        user_yaml_path = self.get_parameter('user_map_yaml').get_parameter_value().string_value

        # === Load 2D Alignment Data ===
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0
        self.map_yaw = 0.0
        
        if user_yaml_path and os.path.exists(user_yaml_path):
            self.load_2d_map_alignment(user_yaml_path)
        else:
            self.get_logger().warn("⚠️ No 'user_map_yaml' provided. 2D Map coordinates will be inaccurate.")

        # === ROS Entities ===
        self.srv = self.create_service(ObjectQuery, 'object_query', self.handle_query)
        self.pub_objects = self.create_publisher(String, '/object_list', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map_markers', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/map_pointcloud', 10)

        # === Data container ===
        self.object_db = defaultdict(list) 
        self.map_points = None
        self.map_colors = None

        # === Load Data ===
        self.load_semantic_map(map_path, sem_path, auto_align)

        # === Publish at startup ===
        self.publish_object_list()
        self.publish_all_markers()
        self.publish_point_cloud()

        self.get_logger().info(f'✅ ObjectQuery service ready.')

    # --------------------------------------------------------------
    # NEW: Load Alignment from YAML
    # --------------------------------------------------------------
    def load_2d_map_alignment(self, yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            origin = data.get('origin', [0.0, 0.0, 0.0])
            self.map_offset_x = float(origin[0])
            self.map_offset_y = float(origin[1])
            self.map_yaw = float(origin[2])
            
            self.get_logger().info(
                f"🗺️  2D Alignment Loaded: Offset=[{self.map_offset_x:.2f}, {self.map_offset_y:.2f}], Yaw={self.map_yaw:.2f}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load 2D YAML: {e}")

    # --------------------------------------------------------------
    # NEW: Convert 3D Point -> 2D Map Coordinate
    # --------------------------------------------------------------
    def transform_3d_to_2d_map(self, k_x, k_y):
        """
        Converts Robot/World Frame (3D Map) -> User Map Frame (2D PNG)
        Logic: (Point - Offset) Unrotated
        """
        # 1. Untranslate (Subtract Offset)
        dx = k_x - self.map_offset_x
        dy = k_y - self.map_offset_y
        
        # 2. Unrotate (Rotate by negative yaw)
        cos_t = math.cos(-self.map_yaw)
        sin_t = math.sin(-self.map_yaw)
        
        u_x = (dx * cos_t) - (dy * sin_t)
        u_y = (dx * sin_t) + (dy * cos_t)
        
        return u_x, u_y

    # --------------------------------------------------------------
    def load_semantic_map(self, map_path: str, sem_path: str, auto_align: bool):
        # ... (Reuse the ROBUST loading function from my previous answer here) ...
        # For brevity, I am assuming you pasted the FIXED version I gave you earlier.
        # Ensure it handles 'segments_info' and 'pts'/'pan'.
        # ...
        
        # [PASTE THE ROBUST LOAD FUNCTION HERE]
        # (If you need me to paste it again fully, let me know!)
        
        # --- Placeholder for the load logic just to make this script runnable in context ---
        if not os.path.exists(map_path): return
        try:
            data = np.load(map_path)
            # Minimal fallback for demonstration if you didn't paste the big block
            if 'pts' in data: points = data['pts']
            elif 'means3D' in data: points = data['means3D']
            else: return

            if 'pan' in data: semantic_ids = data['pan']
            elif 'semantic_ids' in data: semantic_ids = data['semantic_ids']
            else: return

            with open(sem_path, 'r') as f: sem_json = json.load(f)
            
            # Simplified segments logic for brevity
            segments = []
            if isinstance(sem_json, dict) and 'segments_info' in sem_json:
                segments = sem_json['segments_info']
            elif isinstance(sem_json, list):
                segments = sem_json
            
            id_to_name = {}
            for seg in segments:
                if 'id' in seg:
                    name = seg.get('category_name', seg.get('class', 'unknown'))
                    id_to_name[seg['id']] = name

            if auto_align:
                points = self.align_map_to_gravity(points, semantic_ids, id_to_name)
            
            self.map_points = points
            
            # Colors
            if 'rgb' in data:
                raw = data['rgb']
                self.map_colors = (raw * 255).astype(np.uint8) if raw.max() <= 1.0 else raw.astype(np.uint8)
            else:
                self.map_colors = np.zeros((points.shape[0], 3), dtype=np.uint8)

            # Build DB
            self.object_db.clear()
            for sid, name in id_to_name.items():
                mask = semantic_ids == sid
                if np.any(mask):
                    pts = self.map_points[mask]
                    centroid = (np.min(pts, axis=0) + np.max(pts, axis=0)) / 2.0
                    self.object_db[name.lower()].append(tuple(centroid.tolist()))
            
            self.get_logger().info(f'✅ Built object DB with {len(self.object_db)} categories.')
        except Exception as e:
            self.get_logger().error(f"Load Error: {e}")

    # --------------------------------------------------------------
    def align_map_to_gravity(self, points, semantic_ids, id_to_name):
        # (Reuse your existing alignment logic)
        floor_ids = [sid for sid, name in id_to_name.items() if 'floor' in name.lower()]
        if not floor_ids: return points
        mask = np.isin(semantic_ids, floor_ids)
        floor_pts = points[mask]
        if len(floor_pts) < 50: return points
        
        # PCA Rotation
        centered = floor_pts - np.mean(floor_pts, axis=0)
        u, s, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[2, :]
        
        # Align normal to Z (0,0,1)
        rot_axis = np.cross(normal, [0,0,1])
        rot_sin = np.linalg.norm(rot_axis)
        rot_cos = np.dot(normal, [0,0,1])
        
        if rot_sin > 1e-6:
            r = R.from_rotvec((rot_axis / rot_sin) * np.arccos(np.clip(rot_cos, -1.0, 1.0)))
            points = points @ r.as_matrix().T

        # Center Floor to 0,0,0
        floor_pts_rot = points[mask]
        translation = -np.mean(floor_pts_rot, axis=0)
        points += translation
        return points

    # --------------------------------------------------------------
    def handle_query(self, request, response):
        name = request.name.strip().lower()
        found, point = self.search_object(name)
        
        response.found = found
        response.position = point
        
        if found:
            # === NEW: Calculate 2D Map Coordinates ===
            map_x, map_y = self.transform_3d_to_2d_map(point.x, point.y)
            
            # Format message to include BOTH coordinates
            msg = (f"Found '{name}'\n"
                   f" -> 3D World: ({point.x:.2f}, {point.y:.2f})\n"
                   f" -> 2D Map:   ({map_x:.2f}, {map_y:.2f})")
            
            response.message = msg
            self.get_logger().info(f"📍 {msg}")
            
            self.publish_all_markers()
            self.publish_point_cloud()
        else:
             response.message = f"Object '{name}' not found."
             self.get_logger().warn(f"❓ {response.message}")
             
        return response

    def search_object(self, name: str):
        if name in self.object_db:
            instances = self.object_db[name]
            # Find instance closest to origin (0,0,0)
            best_pt = min(instances, key=lambda p: p[0]**2 + p[1]**2 + p[2]**2)
            return True, Point(x=best_pt[0], y=best_pt[1], z=best_pt[2])
        else:
            return False, Point(x=0.0, y=0.0, z=0.0)

    def publish_object_list(self):
        # (Same as before)
        serializable_db = {k: v for k, v in self.object_db.items()}
        msg = String()
        msg.data = json.dumps(serializable_db, indent=2)
        self.pub_objects.publish(msg)

    def publish_all_markers(self):
        # (Same as before)
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        id_counter = 0
        for name, locations in self.object_db.items():
            for i, pos in enumerate(locations):
                x, y, z = pos
                sphere = Marker()
                sphere.header.frame_id = "map"; sphere.header.stamp = now
                sphere.ns = "obj"; sphere.id = id_counter; id_counter += 1
                sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
                sphere.pose.position.x = x; sphere.pose.position.y = y; sphere.pose.position.z = z
                sphere.scale.x = 0.2; sphere.scale.y = 0.2; sphere.scale.z = 0.2
                sphere.color.r = 1.0; sphere.color.a = 0.9
                marker_array.markers.append(sphere)
                
                text = Marker()
                text.header.frame_id = "map"; text.header.stamp = now
                text.ns = "txt"; text.id = id_counter; id_counter += 1
                text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
                text.pose.position.x = x; text.pose.position.y = y; text.pose.position.z = z + 0.3
                text.scale.z = 0.2; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
                text.text = f"{name} ({i+1})"
                marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

    def publish_point_cloud(self):
        # (Same as before)
        if self.map_points is None or self.map_colors is None: return
        points = self.map_points
        colors = self.map_colors
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1; msg.width = points.shape[0]
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
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()