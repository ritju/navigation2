#!/usr/bin/env python
#coding=utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
import math
import time
# from move_base_msgs.msg import *

class MultiMarkNavNode(Node):
    def __init__(self):
        super().__init__(node_name='multi_navigator')
        self.markerArray = MarkerArray()
        self.count = 0       #total goal num
        self.index = 0       #current goal point index
        self.add_more_point = 0 # after all goal arrive, if add some more goal
        self.try_again = 1  # try the fail goal once again
        self.init_fallow = True
        self.mark_pub = self.create_publisher(MarkerArray, "path_point", 100)
        self.click_sub = self.create_subscription(PointStamped, "clicked_point", self.click_callback, 10)
        self.checkstu_time = self.create_timer(0.5, self.timer_callback)
        self.navigator = BasicNavigator()
        # self.goal_pub = rclpy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
        # self.goal_status_sub = rclpy.Subscriber('/move_base/result',MoveBaseActionResult, status_callback)
        #after all goal arrive, if add some more goal
        #we deleberate pub a topic to trig the goal sent
        # self.goal_status_pub = rclpy.Publisher('/move_base/result',MoveBaseActionResult, queue_size=1)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 3.45
        self.initial_pose.pose.position.y = 2.15
        self.initial_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)

        self.navigator.waitUntilNav2Active()
        self.goal_poses = []
        self.nav_start = self.navigator.get_clock().now()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.distance_tolerance = 1.5
        self.local_poses = []
        self.goal_cout = 0

    def timer_callback(self):
        self.robot_to_goal_distance = 2
        pose = PoseStamped()
        
        
        if self.count > 0:
            #set first point
            if self.init_fallow:
                self.init_fallow = False
                self.nav_start = self.navigator.get_clock().now()
                # pose = self.goal_poses[0]
                self.local_poses = [self.goal_poses[0]]
                self.index += 1
                self.navigator.followWaypoints(self.local_poses)
            now = self.navigator.get_clock().now()

            try:
                self.tf_buffer.wait_for_transform_async('map','base_footprint', Time())
                trans = self.tf_buffer.lookup_transform(target_frame='map',source_frame='base_footprint', time=Time())
                robot_location = PoseStamped()
                robot_location.pose.position.x = trans.transform.translation.x
                robot_location.pose.position.y = trans.transform.translation.y
                if len(self.local_poses) > 0:
                    self.robot_to_goal_distance = math.sqrt(math.pow((robot_location.pose.position.x - self.local_poses[0].pose.position.x), 2) + 
                                                            math.pow(robot_location.pose.position.y - self.local_poses[0].pose.position.y, 2))
                    # self.get_logger().info('self.robot_to_goal_distance: %f!'% (self.robot_to_goal_distance))
                    # self.get_logger().info('goal_pose (x, y): (%f, %f)!'% (self.local_poses[0].pose.position.x, self.local_poses[0].pose.position.y))
                    # self.get_logger().info('robot_pose (x, y): (%f, %f)!'% (robot_location.pose.position.x, robot_location.pose.position.y))
                    # self.get_logger().info('self.goal_cout: (%d)!'% (self.goal_cout))
            except:
                self.get_logger().info("no transform !")
            
            #set next point 
            if (self.navigator.isTaskComplete()  or (self.robot_to_goal_distance < self.distance_tolerance)) and self.index < self.count:
            # if (self.navigator.isTaskComplete() ) and self.index < self.count:
                
                self.get_logger().info('Send next goal!')
                self.local_poses = [self.goal_poses[self.index]]
                self.goal_cout += 1
                self.navigator.followWaypoints(self.local_poses)
                self.index += 1
                if now - self.nav_start > Duration(seconds = 600.0):
                    self.navigator.cancelTask()
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                pass
                # self.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                pass
                # self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                pass
                # self.get_logger().info('Goal failed!')
            else:
                pass
                # self.get_logger().info('Goal has an invalid return status!')

    def click_callback(self, msg):
        # global markerArray, count, MARKERS_MAX
        # global goal_pub, index
        # global add_more_point

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.text = str(self.count)
        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        # if(count > MARKERS_MAX):
        #    markerArray.markers.pop(0)
        self.markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.mark_pub.publish(self.markerArray)

        #append goal
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1.0
        self.goal_poses.append(pose)
        

        self.count += 1
        self.get_logger().info('add a path goal point')

def main(args = None):
    """
    ros2运行该节点的入口函数，可配置函数名称
    """
    rclpy.init(args = args) # 初始化rclpy
    node = MultiMarkNavNode()  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）  
    rclpy.shutdown() # rcl关闭

if __name__ == '__main__':
    main()