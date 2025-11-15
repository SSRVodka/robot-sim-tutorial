
import rclpy
from rclpy.node import Node

import os
import sys
import time
from typing import List

from obj_detector_if.srv import ObjectDetection

import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory

PKG_NAME = "object_detector"
PREVIEW_IMG_DIR = "/tmp/obj_det/"

SRV_NAME = "/object_detect_srv"


class ObjDetector(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(
            ObjectDetection, SRV_NAME, self.handle_object_detect)
        self.cv_bridge = CvBridge()
        self.default_model_path = os.path.join(get_package_share_directory(PKG_NAME), "yolo11x.pt")
        self.get_logger().warn(self.default_model_path)
        self.model = YOLO(self.default_model_path)
        os.makedirs(PREVIEW_IMG_DIR, exist_ok=True)

    def handle_object_detect(self, request, response):
        self.get_logger().info("handling object detection request...")
        
        start_time = time.time()
        raw = request.image
        cv_image = self.cv_bridge.imgmsg_to_cv2(raw)
        results = self.model.predict(cv_image)
        self.get_logger().debug(results)
        
        response.number = 0
        for detection in results:
            boxes = detection.boxes.xyxy
            for (idx, box) in enumerate(boxes):
                x_min, y_min, x_max, y_max = box.tolist()
                cv2.rectangle(cv_image,
                            (int(x_min), int(y_min)),
                            (int(x_max), int(y_max)),
                            (0, 255, 0), 2)
                response.left.append(x_min)
                response.right.append(x_max)
                response.top.append(y_min)
                response.bottom.append(y_max)
                self.get_logger().info(f"detect box: [{x_min}, {y_min}]-[{x_max}, {y_max}]")
                response.number += 1
        
        cv2.imwrite(os.path.join(PREVIEW_IMG_DIR, ), cv_image)
        
        end_time = time.time()
        response.consume_time = end_time - start_time
        self.get_logger().info(f"finishing detection in {end_time-start_time} s")
        return response


def main(args: List[str] = []):
    rclpy.init(args=args)
    
    _node_name = "obj_detector_srv_node"
    det_node = ObjDetector(_node_name)
    _logger = det_node.get_logger()
    _logger.info(f"{_node_name} initialized")

    rclpy.spin(det_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

