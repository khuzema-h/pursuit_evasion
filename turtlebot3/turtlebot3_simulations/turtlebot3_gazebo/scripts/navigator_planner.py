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
import time


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

        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.evader_x = 0.0
        self.evader_y = 0.0

        self.waypoints = []
        self.current_index = 0
        self.reached_position = False
        self.is_caught = False

        self.started = True
        # Set up publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/Pursuer/cmd_vel', 10)
        self.create_subscription(Odometry, '/Pursuer/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/Evader/odom', self.evader_odom_callback, 10)
        self.create_subscription(Bool, '/red_obstacle_detected', self.caught_callback, 10)
        self.create_timer(0.1, self.control_loop)

        # Timer to plan every 2 seconds
        # self.create_timer(5.0, self.update_waypoints)  # Plan every 2 seconds
        # if self.evader_x != 0.0:
        #     if self.started == True:
        #         self.update_waypoints()
        #         self.started == False

        # self.update_waypoints() 
        self.get_logger().info("Pursuer Controller node has started.")

    def evader_odom_callback(self, msg):
        """Callback to update the evader's position."""
        self.evader_x = msg.pose.pose.position.x
        self.evader_y = msg.pose.pose.position.y
        # self.update_waypoints() 
        if self.started == True:
            self.update_waypoints()
            self.started = False


    def update_waypoints(self):
        """Update the waypoints using the planner with the evader's position as the goal."""
        

        # Use the current position of the pursuer as the start point and evader's position as the goal
        occupancy_grid = self.load_maze_map()  # Load the maze map
        goal = (int(self.evader_x *100),int(self.evader_y*100 + 50 ))
        start = (int(self.x * 100 + 100), int(self.y * 100 + 100))
        self.get_logger().info(f"Planning new path towards start at ({start})")
        self.get_logger().info(f"Planning new path towards evader at ({goal})")
        new_path = run_planner(occupancy_grid, start, goal)  # Get new path using the planner
        all_path = []
        for i in range(len(new_path)):
            x,y,theta = new_path[i]
            if i == 0:
                all_path.append(((x - 60)/100,(y-60)/100,0))
            elif theta != None:
                all_path.append(((x - 60)/100,(y-60)/100,theta))


        self.get_logger().info(f"New Path ({all_path})")
        # Update the waypoints
        self.waypoints = all_path
        self.current_index = 0  # Start from the first waypoint
        self.reached_position = False  # Reset position reached flag

    def load_maze_map(self):
        """Load and process the maze map image."""
        package_share = get_package_share_directory('turtlebot3_gazebo')
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
        return self.occupancy_grid

    def caught_callback(self, msg):
        """Callback to stop the pursuer if caught."""
        self.is_caught = msg.data
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
        if self.is_caught:
            return

        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached.")
            return

        goal_x, goal_y, goal_theta_deg = self.waypoints[self.current_index]
        self.get_logger().info(f"goal_theta_deg ({goal_theta_deg})")
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
