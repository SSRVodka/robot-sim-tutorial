#!/usr/bin/env python3
# encoding: utf-8
# @date 2025/07/31
# @author SSRVodka

import math
import numpy as np

import rclpy
from rclpy.node import Node

### Data structures
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, Empty
from rclpy.duration import Duration
# Note: SetPose2D {input: Pose2D{x,y,r,p,y(float,degrees)}, output: success(bool),message(str) }
from interfaces.srv import GetPose2D, SetPose2D
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

### Helper classes & methods

# from rcl_interfaces.srv import GetParameters

from rclpy.executors import MultiThreadedExecutor, Executor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf_transformations import quaternion_from_euler


class NavigationController(Node):
    markerArray = MarkerArray()
    
    def __init__(self, name, executor_ref: Executor):
        super().__init__(name)
        
        self.shared_cbg = ReentrantCallbackGroup()

        self.goal_pose = PoseStamped()
        self.navigator = BasicNavigator('nav2controller_base_nav')

        self.qos_prof = 10

        self.map_frame = 'map'#self.get_parameter('map_frame').value
        # 是否必须？
        self.nav_goal = '/nav_goal'#self.get_parameter('nav_goal').value

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', self.qos_prof, callback_group=self.shared_cbg)
        # 是否必须？为何不直接回调？
        self.nav_pub = self.create_publisher(
            PoseStamped, self.nav_goal, self.qos_prof, callback_group=self.shared_cbg)
        self.reach_pub = self.create_publisher(
            Bool, 'navigation_controller/reach_goal', self.qos_prof, callback_group=self.shared_cbg)

        self.create_subscription(
            PoseStamped, self.nav_goal, self.goal_callback, self.qos_prof, callback_group=self.shared_cbg)

        self.create_service(
            SetPose2D, 'navigation_controller/set_pose', self.move_srv_callback, callback_group=self.shared_cbg)
        self.create_service(
            Trigger, 'navigation_controller/stop', self.stop_navigation_callback, callback_group=self.shared_cbg)
        
        # initiate initial pose
        self.pose_manager_get_client = self.create_client(
            GetPose2D, "pose_manager/get_pose", callback_group=self.shared_cbg)
        self.pose_manager_get_client.wait_for_service()

        init_pose_future = self.pose_manager_get_client.call_async(GetPose2D.Request())
        self.get_logger().info("waiting for initial pose from pose manager...")
        rclpy.spin_until_future_complete(self, init_pose_future, executor=executor_ref)
        resp = init_pose_future.result()
        self.get_logger().info(f"initial pose response received: {resp}. Now publish it for AMCL")
        self.navigator.setInitialPose(resp.pose)
        self.navigator.waitUntilNav2Active()
        
        self.create_service(
            Empty, 'navigation_controller/init_fin', self.get_node_state, callback_group=self.shared_cbg)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Navigation controller started')

    def get_node_state(self, request, response):
        return response
    
    # def clear_marker(self):
    #     marker_Array = MarkerArray()
    #     marker = Marker()
    #     marker.header.frame_id = self.map_frame
    #     marker.action = Marker.DELETEALL
    #     marker_Array.markers.append(marker)

    #     self.mark_pub.publish(marker_Array)
    
    # def add_marker(self, pose: PoseStamped):
    #     markerArray = MarkerArray()
    #     # mark the point with number to display
    #     marker = Marker()
    #     marker.header.frame_id = self.map_frame

    #     marker.type = marker.MESH_RESOURCE
    #     marker.mesh_resource = "package://example/resource/flag.dae"
    #     marker.action = marker.ADD
        
    #     marker.scale.x = 0.08
    #     marker.scale.y = 0.08
    #     marker.scale.z = 0.2
        
    #     color = list(np.random.choice(range(256), size=3))
    #     marker.color.a = 1.0
    #     marker.color.r = color[0] / 255.0
    #     marker.color.g = color[1] / 255.0
    #     marker.color.b = color[2] / 255.0
    #     # marker.lifetime = rospy.Duration(10)  # display time. If not set, it will be kept by default
    #     marker.pose.position.x = pose.pose.position.x
    #     marker.pose.position.y = pose.pose.position.y
    #     marker.pose.orientation = pose.pose.orientation
    #     markerArray.markers.append(marker)

    #     self.mark_pub.publish(markerArray)

    def move_srv_callback(self, request, response):
        self.get_logger().info('set_pose request received')

        # self.clear_marker()

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # pose.header.stamp.sec = 0
        # pose.header.stamp.nanosec = 0
        data = request.data
        q = quaternion_from_euler(
            math.radians(data.roll),
            math.radians(data.pitch),
            math.radians(data.yaw))
        pose.pose.position.x = data.x
        pose.pose.position.y = data.y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        # self.add_marker(pose)
        self.nav_pub.publish(pose)
        
        response.success = True
        response.message = "Navigation started..."
        return response

    def stop_navigation_callback(self, request, response):
        if self.navigator.isTaskComplete():
            self.get_logger().info('No active navigation task to stop.')
            response.success = False
            response.message = 'No active task.'
        else:
            self.navigator.cancelTask()
            self.get_logger().info('Navigation canceled via service.')
            response.success = True
            response.message = 'Navigation stopped.'
        return response

    def goal_callback(self, msg):
        self.get_logger().info('\033[1;32m New goal point request received: %s\033[0m' % str(msg))

        self.navigator.goToPose(msg)
        i = 0
        # feedback = self.navigator.getFeedback()
        # total_time = Duration.from_msg(feedback.estimated_time_remaining).nanosecond / 1e9
        # self.get_logger().info('\033[1;32m%s\033[0m' % 'total_time')
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            # self.get_logger().info(f'{feedback.navigation_time} {feedback.estimated_time_remaining}')
            if feedback and i % 5 == 0:
                # self.get_logger().info(
                    # 'Estimated time of arrival: '
                    # + '{0:.0f}'.format(
                        # Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        # / 1e9
                    # )
                    # + ' seconds.'
                # )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'timeout...')
                    self.navigator.cancelTask()

                # # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=36.0):
                #     self.get_logger().info('\033[1;32m%s\033[0m' % 'preempt...')
                #     self.goal_pub.publish(self.goal_pose)
            # self.get_logger().info('\033[1;32m%s\033[0m' % 'feedback')
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            msg = Bool()
            msg.data = True
            self.reach_pub.publish(msg)
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed')
            msg = Bool()
            msg.data = False
            self.reach_pub.publish(msg)
        else:
            self.get_logger().info('Goal has an invalid return status: %s' % result)

def main(args = None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = NavigationController('navigation_controller', executor_ref=executor)
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == "__main__":
    main()

