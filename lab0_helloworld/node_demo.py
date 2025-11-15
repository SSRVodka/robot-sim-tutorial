
import rclpy

from rclpy.node import Node

def main(args=None):
    rclpy.init()
    node_name = "demo_py_node"
    node = Node(node_name)
    _logger = node.get_logger()
    _logger.info(f"Hello from {node_name}")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
