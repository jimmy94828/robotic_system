import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from collections import defaultdict
from scipy.spatial.transform import Rotation as R
import kachaka_api

class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')

        # === 1. DECLARE PARAMETERS (THE SWITCHES) ===
        # Default: Both ON
        self.declare_parameter('use_native_map', True)   # Switch for Kachaka Native Locations
        self.declare_parameter('use_semantic_map', True) # Switch for your 3D Map Objects
        
        self.declare_parameter('map_path', 'data/params.npz')
        self.declare_parameter('semantic_path', 'data/semantic.json')
        self.declare_parameter('kachaka_ip', '192.168.0.157:26400')
        self.declare_parameter('auto_align', True)

        # === 2. SETUP SERVICE & PUBLISHERS ===
        self.srv = self.create_service(ObjectQuery, 'object_query', self.handle_query)
        self.pub_objects = self.create_publisher(String, '/object_list', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_map_markers', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/map_pointcloud', 10)

        # === 3. DATABASE ===
        self.object_db = defaultdict(list)
        self.map_points = None
        self.map_colors = None
        self.kachaka_client = None

        # Connect to Kachaka once at startup
        robot_ip = self.get_parameter('kachaka_ip').value
        try:
            self.get_logger().info(f"Connecting to Kachaka API at {robot_ip}...")
            self.kachaka_client = kachaka_api.KachakaApiClient(robot_ip)
        except Exception as e:
            self.get_logger().error(f"Kachaka Connection Failed: {e}")

        # === 4. LOAD DATA BASED ON MODES ===
        self.refresh_database()

        # === 5. ENABLE RUNTIME SWITCHING ===
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info('✅ Modular Object Query Server Ready.')

    def parameters_callback(self, params):
        """Called whenever a parameter is changed via CLI."""
        reload_needed = False
        for param in params:
            if param.name in ['use_native_map', 'use_semantic_map', 'map_path', 'semantic_path']:
                reload_needed = True
                self.get_logger().info(f"🔄 Parameter '{param.name}' changed to: {param.value}")

        if reload_needed:
            # We need to accept the parameter change first, then reload
            # (Logic is split because we can't trigger reload *inside* the callback easily without threading issues, 
            # but for simple cases, calling a refresh method usually works if handled carefully).
            # To be safe, we just set a flag or call it directly if it's fast.
            pass 
            
        return rclpy.rclpy.impl.rcutils.SetParametersResult(successful=True)
    
    # We will simply check the parameters fresh every time we reload.
    # To actually trigger the reload logic after parameter update:
    def set_parameters(self, params):
        # Override standard set_parameters to trigger reload
        results = super().set_parameters(params)
        self.refresh_database()
        return results

    # --------------------------------------------------------------
    # CORE LOGIC: REFRESH DATABASE
    # --------------------------------------------------------------
    def refresh_database(self):
        """Clears DB and reloads based on current switches."""
        self.object_db.clear()
        self.map_points = None
        self.map_colors = None

        # Get latest switch states
        use_native = self.get_parameter('use_native_map').value
        use_semantic = self.get_parameter('use_semantic_map').value
        
        self.get_logger().info(f"--- Refreshing DB [Native: {use_native} | Semantic: {use_semantic}] ---")

        # 1. Load Semantic Map (If Enabled)
        if use_semantic:
            map_path = self.get_parameter('map_path').value
            sem_path = self.get_parameter('semantic_path').value
            auto_align = self.get_parameter('auto_align').value
            self.load_semantic_map(map_path, sem_path, auto_align)

        # 2. Load Native Locations (If Enabled)
        if use_native:
            self.load_native_locations()

        # Update Visuals
        self.publish_object_list()
        self.publish_all_markers()
        self.publish_point_cloud()

    # --------------------------------------------------------------
    # LOADERS
    # --------------------------------------------------------------
    def load_native_locations(self):
        if not self.kachaka_client: return
        try:
            locations = self.kachaka_client.get_locations()
            count = 0
            for loc in locations:
                name = loc.name.strip().lower()
                self.object_db[name].append((loc.pose.x, loc.pose.y, 0.0))
                count += 1
            self.get_logger().info(f"   + Loaded {count} Native Locations.")
        except Exception as e:
            self.get_logger().error(f"   ! Failed to load native locations: {e}")

    def load_semantic_map(self, map_path, sem_path, auto_align):
        if not os.path.exists(map_path) or not os.path.exists(sem_path):
            self.get_logger().warn("   ! Semantic map files not found.")
            return

        try:
            data = np.load(map_path)
            points = data['means3D']
            semantic_ids = data['semantic_ids']
            with open(sem_path, 'r') as f:
                sem_json = json.load(f)
            id_to_name = {seg['id']: seg['class'] for seg in sem_json.get('segmentation', [])}

            if auto_align:
                points = self.align_map_to_gravity(points, semantic_ids, id_to_name)
            
            self.map_points = points
            
            # --- Color Generation (Simplified for brevity) ---
            unique_ids = np.unique(semantic_ids)
            color_map = {uid: (random.randint(50,255), random.randint(50,255), random.randint(50,255)) for uid in unique_ids}
            self.map_colors = np.zeros((points.shape[0], 3), dtype=np.uint8)
            for uid, c in color_map.items(): self.map_colors[semantic_ids==uid] = c

            # --- Object Extraction ---
            count = 0
            for sid, name in id_to_name.items():
                mask = semantic_ids == sid
                if np.any(mask):
                    pts = points[mask]
                    centroid = (np.min(pts,0) + np.max(pts,0)) / 2.0
                    self.object_db[name.lower()].append(tuple(centroid))
                    count += 1
            self.get_logger().info(f"   + Loaded {count} Semantic Objects.")

        except Exception as e:
            self.get_logger().error(f"   ! Semantic Load Error: {e}")

    # --------------------------------------------------------------
    # UTILS (Alignment, Query, Viz) - KEEP AS IS
    # --------------------------------------------------------------
    def align_map_to_gravity(self, points, semantic_ids, id_to_name):
        # ... (Paste your existing alignment logic here) ...
        # For brevity, I am omitting the 50 lines of matrix math 
        # since you already have this working function.
        return points

    def handle_query(self, request, response):
        name = request.name.strip().lower()
        
        # Check if database is empty (Mode safety check)
        if not self.object_db:
            response.found = False
            response.message = "Database is empty! Check parameters."
            return response

        found, point = self.search_object(name)
        response.found = found
        response.position = Point(x=point.x, y=point.y, z=point.z)
        if found:
            response.message = f"Found '{name}' at ({point.x:.2f}, {point.y:.2f})"
        else:
            response.message = f"'{name}' not found in current maps."
        
        self.get_logger().info(f"Query '{name}' -> {response.message}")
        return response

    def search_object(self, name):
        if name in self.object_db:
            instances = self.object_db[name]
            best = min(instances, key=lambda p: p[0]**2 + p[1]**2 + p[2]**2)
            return True, Point(x=best[0], y=best[1], z=best[2])
        return False, Point(x=0.0, y=0.0, z=0.0)

    def publish_object_list(self):
        msg = String()
        msg.data = json.dumps({k:v for k,v in self.object_db.items()}, indent=2)
        self.pub_objects.publish(msg)

    def publish_all_markers(self):
        # ... (Paste your existing marker logic here) ...
        pass # Placeholder for your existing code

    def publish_point_cloud(self):
        # ... (Paste your existing pointcloud logic here) ...
        pass # Placeholder for your existing code

def main():
    rclpy.init()
    node = ObjectQueryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()