import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from object_query_interfaces.srv import ObjectQuery
import numpy as np
import json
import os


class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')

        # === Declare parameters (with safe defaults) ===
        self.declare_parameter('map_path', '/mnt/HDD1/phudh/home_robot/subproject1/data/params.npz')
        self.declare_parameter('semantic_path', '/mnt/HDD1/phudh/home_robot/subproject1/data/semantic.json')

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        sem_path = self.get_parameter('semantic_path').get_parameter_value().string_value

        # === ROS Entities ===
        self.srv = self.create_service(ObjectQuery, 'object_query', self.handle_query)
        self.pub_objects = self.create_publisher(String, '/object_list', 10)

        # === Data container ===
        self.object_db = {}

        # === Load Data ===
        self.load_semantic_map(map_path, sem_path)

        # === Publish at startup ===
        self.publish_object_list()

        self.get_logger().info(f'✅ ObjectQuery service ready with {len(self.object_db)} objects.')

    # --------------------------------------------------------------
    def load_semantic_map(self, map_path: str, sem_path: str):
        """Load semantic map (.npz) and class names (.json)."""
        if not os.path.exists(map_path):
            self.get_logger().error(f'Map file not found: {map_path}')
            return
        if not os.path.exists(sem_path):
            self.get_logger().error(f'Semantic JSON not found: {sem_path}')
            return

        try:
            data = np.load(map_path)
            means3D = data['means3D']
            semantic_ids = data['semantic_ids']

            # Load semantic class mapping from JSON
            with open(sem_path, 'r') as f:
                sem_json = json.load(f)

            id_to_name = {seg['id']: seg['class'] for seg in sem_json.get('segmentation', [])}
            self.get_logger().info(f'Loaded {len(id_to_name)} semantic labels from JSON.')

            # Compute centroid per object
            for sid, name in id_to_name.items():
                mask = semantic_ids == sid
                if np.any(mask):
                    pts = means3D[mask]
                    centroid = np.mean(pts, axis=0)
                    self.object_db[name.lower()] = tuple(centroid.tolist())

            self.get_logger().info(f'📦 Built object DB with {len(self.object_db)} entries.')

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load semantic map: {e}')

    # --------------------------------------------------------------
    def handle_query(self, request, response):
        """Handle a query for an object’s position."""
        name = request.name.strip().lower()
        found, point = self.search_object(name)

        # Explicitly set response fields
        response.found = found
        response.position = Point(x=point.x, y=point.y, z=point.z)
        response.message = f'Found {name}!' if found else f'Object {name} not found.'

        self.get_logger().info(
            f'🔍 Query: {name} -> found={found}, pos=({point.x:.2f}, {point.y:.2f}, {point.z:.2f})'
        )
        return response

    # --------------------------------------------------------------
    def search_object(self, name: str):
        """Look up object centroid in memory."""
        if name in self.object_db:
            x, y, z = self.object_db[name]
            return True, Point(x=x, y=y, z=z)
        else:
            return False, Point(x=0.0, y=0.0, z=0.0)

    # --------------------------------------------------------------
    def publish_object_list(self):
        """Publish all objects in /object_list topic."""
        msg = String()
        msg.data = json.dumps(self.object_db, indent=2)
        self.pub_objects.publish(msg)
        self.get_logger().info(f'📢 Published /object_list ({len(self.object_db)} objects).')


# --------------------------------------------------------------
def main():
    rclpy.init()
    node = ObjectQueryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
