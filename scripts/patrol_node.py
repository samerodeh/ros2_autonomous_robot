#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import time

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        
        self.waypoints = [
            self.create_pose(1.0, 0.0, 0.0),
            self.create_pose(2.0, 1.0, 1.57),
            self.create_pose(0.0, 1.0, 3.14),
        ]
        self.current_waypoint = 0
        
    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = sin(yaw / 2)
        pose.pose.orientation.w = cos(yaw / 2)
        return pose
        
    def run_patrol(self):
        self.navigator.waitUntilNav2Active()
        
        while rclpy.ok():
            goal = self.waypoints[self.current_waypoint]
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            
            self.get_logger().info(f'Going to waypoint {self.current_waypoint}')
            self.navigator.goToPose(goal)
            
            # Poll rather than busy-wait: isTaskComplete() spins the rclpy executor
            # internally, so a bare `pass` loop just burns a core.
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
            
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)

def main():
    rclpy.init()
    node = PatrolNode()
    node.run_patrol()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
