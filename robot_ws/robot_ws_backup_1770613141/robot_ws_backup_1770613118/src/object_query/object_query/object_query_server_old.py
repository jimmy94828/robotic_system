# file: object_query_server.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from object_query_interfaces.srv import ObjectQuery

# 假資料：名稱 -> position
DB = {
    "apple": (1.0, 2.0, 0.5),
    "banana": (0.2, -0.3, 0.9),
}

class ObjectQueryServer(Node):
    def __init__(self):
        super().__init__('object_query_server')
        self.srv = self.create_service(ObjectQuery, 'object_query', self.handle_query)
        self.get_logger().info('ObjectQuery service ready on /object_query')

    def handle_query(self, request, response):
        name = request.name.strip().lower()
        response.found, response.position = self.search_object(name)
        if response.found:
            response.message = f'Success Found {name} !'
        else:
            response.message = f'Didnt have this Object: {name}'
        self.get_logger().info(f'Query: {request.name} -> found={response.found} {response.position} {response.message}')
        return response
    
    # serch object in DB(SLAM MAP)
    def search_object(self, name: str):
        name = name.strip().lower()
        if name in DB:
            x, y, z = DB[name]
            return True, Point(x=x, y=y, z=z)
        else:
            return False, Point(x=0.0, y=0.0, z=0.0)

def main():
    rclpy.init()
    node = ObjectQueryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
