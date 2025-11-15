
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import os
import sys

import cv2
from cv_bridge import CvBridge

from obj_detector_if.srv import ObjectDetection

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image

PKG_NAME = "obj_det_client"
SRV_NAME = "/object_detect_srv"
DEMO_IMG = "demo.webp"


class ObjClient(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.client = self.create_client(ObjectDetection, SRV_NAME)
        self.bridge = CvBridge()

        self.demo_image = os.path.join(get_package_share_directory(PKG_NAME), DEMO_IMG)
    
    def sim_sensor_img(self) -> Image:
        cv2_img = cv2.imread(self.demo_image)
        return self.bridge.cv2_to_imgmsg(cv2_img)
    
    def send_detection_request(self):
        req = ObjectDetection.Request()
        req.image = self.sim_sensor_img()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("waiting for service endpoint to be ready...")
        
        self.get_logger().info("call the service asynchronously. It may cost some time to finish...")
        resp_future = self.client.call_async(req)
        resp_future.add_done_callback(self.srv_done_callback)

    def srv_done_callback(self, resp_fut: Future):
        self.get_logger().info("service done")
        if resp_fut.done():
            ex = resp_fut.exception()
            if ex is not None:
                self.get_logger().error(f"service error: {ex}")
            else:
                self.get_logger().info(f"service returns: {resp_fut.result()}")
        else:
            self.get_logger().warn("service call cancelled")


def main(args: list[str] = []):
    rclpy.init(args=args)

    _node_name = "obj_det_client_node"
    node = ObjClient(_node_name)
    _logger = node.get_logger()
    _logger.info(f"{_node_name} initialized")

    node.send_detection_request()
    rclpy.spin(node)
    rclpy.shutdown()
    _logger.info(f"shutting down...")


if __name__ == "__main__":
    main(sys.argv)
