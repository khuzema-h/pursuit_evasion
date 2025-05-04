#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from tf_transformations import euler_from_quaternion
import math
import random
import time

from rrt_planner import RRTPlanner
from red_detector import RedDetector

class RRTExplorer(Node):
    def __init__(self):
        super().__init__('rrt_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/Pursuer/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Pursuer/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/Pursuer/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.current_pose = None
        self.map = None
        self.map_info = None

        self.exploring = True

        self.red_detector = RedDetector()

        self.target_path = []
        self.target_index = 0

        self.speed_linear = 0.15
        self.speed_angular = 0.5

    def map_callback(self, msg):
        self.map = msg.data
        self.map_info = msg.info

    def odom_callback(self, msg):
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        self.current_pose = (position.x, position.y, yaw)

    def timer_callback(self):
        if not self.exploring:
            return

        if self.red_detector.is_red_detected():
            self.get_logger().info("Red detected! Stopping exploration.")
            self.stop_robot()
            self.exploring = False
            return

        if self.map is None or self.current_pose is None:
            return

        if not self.target_path:
            self.pick_new_goal_and_plan()
        else:
            self.follow_path()

    def pick_new_goal_and_plan(self):
        self.get_logger().info("Picking new goal...")

        start = (self.current_pose[0], self.current_pose[1])

        goal = self.sample_random_free_point()

        planner = RRTPlanner(
            self.map,
            self.map_info.width,
            self.map_info.height,
            self.map_info.resolution,
            (self.map_info.origin.position.x, self.map_info.origin.position.y)
        )

        path = planner.plan(start, goal)

        if path is not None:
            self.get_logger().info(f"Path found with {len(path)} waypoints.")
            self.target_path = path
            self.target_index = 0
        else:
            self.get_logger().info("Failed to find path, retrying...")
            time.sleep(0.5)

    def sample_random_free_point(self):
        width = self.map_info.width
        height = self.map_info.height
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        while True:
            rand_x = random.uniform(origin_x, origin_x + width * resolution)
            rand_y = random.uniform(origin_y, origin_y + height * resolution)

            map_x = int((rand_x - origin_x) / resolution)
            map_y = int((rand_y - origin_y) / resolution)

            if 0 <= map_x < width and 0 <= map_y < height:
                idx = map_y * width + map_x
                if self.map[idx] < 30:  # Free space
                    return (rand_x, rand_y)

    def follow_path(self):
        if self.target_index >= len(self.target_path):
            self.get_logger().info("Reached goal!")
            self.target_path = []
            return

        goal_x, goal_y = self.target_path[self.target_index]

        robot_x, robot_y, robot_theta = self.current_pose

        dx = goal_x - robot_x
        dy = goal_y - robot_y

        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        angle_diff = self.normalize_angle(angle_to_goal - robot_theta)

        cmd = Twist()

        if abs(angle_diff) > 0.2:
            cmd.angular.z = self.speed_angular if angle_diff > 0 else -self.speed_angular
        elif distance > 0.05:
            cmd.linear.x = self.speed_linear
        else:
            self.target_index += 1  # Reached waypoint

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    explorer = RRTExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
