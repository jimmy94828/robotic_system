# file: object_query_client.py
import sys
import rclpy
from rclpy.node import Node
from object_query_interfaces.srv import ObjectQuery

class ObjectQueryClient(Node):
    def __init__(self):
        super().__init__('object_query_client')
        self.cli = self.create_client(ObjectQuery, 'object_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def send(self, name: str):
        req = ObjectQuery.Request()
        req.name = name
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: object_query_client.py <name>')
        return
    name = sys.argv[1]              # set object name
    node = ObjectQueryClient()      # create client
    resp = node.send(name)          # send request (return: found (bool), position (x,y,z), message)
    print(f'found={resp.found}, position=({resp.position.x}, {resp.position.y}, {resp.position.z}), message="{resp.message}"')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
