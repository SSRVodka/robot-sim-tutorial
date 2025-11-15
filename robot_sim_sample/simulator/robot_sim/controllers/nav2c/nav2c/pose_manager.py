
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator

from tf_transformations import quaternion_from_euler, euler_from_quaternion

import math
from interfaces.msg import Pose2D
from interfaces.srv import SetPose2D, GetPose2D

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class PoseManager(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.shared_cbg = MutuallyExclusiveCallbackGroup()

        self.init_pose = PoseStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.header.stamp = self.get_clock().now().to_msg()
        self.init_pose.pose.position.x = 0.0
        self.init_pose.pose.position.y = 0.0
        self.init_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, 0)  # r, p, y (rad)
        self.init_pose.pose.orientation.x = q[0]
        self.init_pose.pose.orientation.y = q[1]
        self.init_pose.pose.orientation.z = q[2]
        self.init_pose.pose.orientation.w = q[3]

        self.qos_profile = 10
        self.current_pose = self.init_pose

        self.create_service(GetPose2D, "pose_manager/get_pose", self.get_pose, callback_group=self.shared_cbg)
        self.create_service(SetPose2D, "pose_manager/set_pose", self.set_pose, callback_group=self.shared_cbg)

    @staticmethod
    def pose_stamped_to_pose2d(pose: PoseStamped) -> Pose2D:
        result = Pose2D()
        result.x = pose.pose.position.x
        result.y = pose.pose.position.y

        r, p, y = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])
        result.roll = r
        result.pitch = p
        result.yaw = y
        return result

    def get_pose(self, request, response):
        self.get_logger().info("get_pose request received")
        response.success = True
        # response.pose2d = PoseManager.pose_stamped_to_pose2d(self.current_pose)
        response.pose = self.current_pose
        self.get_logger().info(f"get_pose return with {response}")
        return response
    
    def set_pose(self, request, response):
        data = request.data
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        q = quaternion_from_euler(
            math.radians(data.roll),
            math.radians(data.pitch),
            math.radians(data.yaw))
        self.current_pose.pose.position.x = data.x
        self.current_pose.pose.position.y = data.y
        self.current_pose.pose.orientation.x = q[0]
        self.current_pose.pose.orientation.y = q[1]
        self.current_pose.pose.orientation.z = q[2]
        self.current_pose.pose.orientation.w = q[3]

        response.success = True
        response.message = "pose_manager/set_pose success"

        # self.pose_pub.publish(PoseManager.pose_stamped_to_pose2d(self.current_pose))
        # self.pose_pub.publish(self.current_pose)
        return response


def main(args = None):
    rclpy.init(args=args)
    pose_manager = PoseManager('nav2sim_pose_manager')

    executor = MultiThreadedExecutor()
    executor.add_node(pose_manager)
    executor.spin()
    # rclpy.spin(pose_manager)
    pose_manager.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
