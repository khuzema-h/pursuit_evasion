#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import os
import cv2
from planner import run_planner
import numpy as np

class PursuerController(Node):
    def __init__(self):
        super().__init__('pursuer_controller')

        # Tuning parameters
        self.linear_kp = 0.7
        self.angular_kp = 0.5
        self.goal_tolerance = 0.5
        self.angle_tolerance = math.radians(15)
        self.max_angular_speed = math.pi / 2
        self.min_angular_speed = 0.2
        self.max_linear_speed = 1.0

        # Load waypoints
        package_share = get_package_share_directory('turtlebot3_gazebo')
        waypoint_file = os.path.join(package_share, 'scripts', 'path.txt')
        self.waypoints = self.load_waypoints_from_file(waypoint_file)

        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded. Shutting down.")
            rclpy.shutdown()
            return

        self.current_index = 0
        self.reached_position = False

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.is_caught = False

        # Evader state
        self.evader_x = 0.0
        self.evader_y = 0.0


        # Set up publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/Pursuer/cmd_vel', 10)
        self.create_subscription(Odometry, '/Pursuer/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/Evader/odom', self.evader_odom_callback, 10)
        self.create_subscription(Bool, '/red_obstacle_detected', self.caught_callback, 10)  # Fixed type and name
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Pursuer Controller node has started.")


    def load_maze_map(self):
        """Load and process the maze map image."""
        package_share = get_package_share_directory('turtlebot3_gazebo')
        # image_path = '/home/adil/project_5_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/scripts/maze.png'
        image_path = os.path.join(package_share, 'scripts', 'maze.png')

        
        maze = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        
        if maze is None:
            raise FileNotFoundError(f"Could not load image at path: {image_path}")

        self.maze_width = 1600
        self.maze_height = 800
        maze = cv2.resize(maze, (self.maze_width, self.maze_height), interpolation=cv2.INTER_NEAREST)
        
        _, binary_maze = cv2.threshold(maze, 127, 255, cv2.THRESH_BINARY)
        binary_maze = cv2.bitwise_not(binary_maze)
        
        kernel = np.ones((30, 30), np.uint8)
        binary_maze = cv2.dilate(binary_maze, kernel, iterations=2)
        binary_maze = cv2.bitwise_not(binary_maze)
        
        self.occupancy_grid = (binary_maze == 255).astype(np.uint8)
        
        self.get_logger().info("Successfully loaded and processed maze map")


    def evader_odom_callback(self, msg):
        """Callback to update the evader's position and log its position as the goal."""
        self.evader_x = msg.pose.pose.position.x
        self.evader_y = msg.pose.pose.position.y


    def load_waypoints_from_file(self, filename):
        """Load waypoints from the specified file and scale x, y by 100."""
        waypoints = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    parts = line.strip().split(',')
                    if len(parts) == 3:
                        x, y, theta = map(float, parts)
                        x_scaled = x / 100.0  # Scale x by 100
                        y_scaled = y / 100.0  # Scale y by 100
                        waypoints.append((x_scaled, y_scaled, theta))
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from file: {e}")
        return waypoints

    def caught_callback(self, msg):
        """Callback to stop the pursuer if caught."""
        self.is_caught = msg.data  # Access the data field
        if self.is_caught:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Pursuer caught! Stopping.")

    def odom_callback(self, msg):
        """Callback to update the position and orientation from Odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
        """Control loop to follow the waypoints and avoid obstacles."""
        if self.is_caught:  # Check if caught first
            return

        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached.")
            return

        goal_x, goal_y, goal_theta_deg = self.waypoints[self.current_index]
        goal_theta_rad = math.radians(goal_theta_deg)

        cmd = Twist()

        if not self.reached_position:
            # Phase 1: Navigate to waypoint position
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.hypot(dx, dy)
            heading_to_goal = math.atan2(dy, dx)

            raw_error = heading_to_goal - self.yaw
            alt_error = raw_error - 2 * math.pi if raw_error > 0 else raw_error + 2 * math.pi
            heading_error = raw_error if abs(raw_error) < abs(alt_error) else alt_error

            if abs(heading_error) > math.pi - 0.2:
                heading_error = math.copysign(math.pi, heading_error)

            self.get_logger().info(
                f"[Waypoint {self.current_index + 1}/{len(self.waypoints)}] "
                f"Current: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°, "
                f"Distance: {distance:.3f} m, Heading Error: {math.degrees(heading_error):.1f}°"
            )

            if distance > self.goal_tolerance:
                if abs(heading_error) > math.radians(15):
                    cmd.angular.z = math.copysign(
                        min(self.angular_kp * abs(heading_error), self.max_angular_speed),
                        heading_error
                    )
                else:
                    cmd.linear.x = min(self.linear_kp * distance, self.max_linear_speed)
            else:
                self.get_logger().info(f"Reached position of waypoint {self.current_index + 1}")
                self.reached_position = True

        else:
            # Phase 2: Rotate to final heading
            final_yaw_error = self.normalize_angle(goal_theta_rad - self.yaw)

            self.get_logger().info(
                f"[Waypoint {self.current_index + 1}/{len(self.waypoints)}] "
                f"Aligning to final heading: Current yaw={math.degrees(self.yaw):.1f}°, "
                f"Target yaw={math.degrees(goal_theta_rad):.1f}°, "
                f"Yaw error={math.degrees(final_yaw_error):.1f}°"
            )

            if abs(final_yaw_error) > self.angle_tolerance:
                cmd.angular.z = math.copysign(
                    min(self.angular_kp * abs(final_yaw_error), self.max_angular_speed),
                    final_yaw_error
                )
            else:
                self.get_logger().info(f"Waypoint {self.current_index + 1} completed (position + heading)")
                self.current_index += 1
                self.reached_position = False

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot by sending zero velocity commands."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Pursuer stopped.")

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = PursuerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
