#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Added import
import math
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import os


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Tuning parameters
        self.linear_kp = 0.5
        self.angular_kp = 0.5
        self.goal_tolerance = 0.1
        self.angle_tolerance = math.radians(15)
        self.max_angular_speed = math.pi/2
        self.min_angular_speed = 0.2
        self.max_linear_speed = 1.0

        # Load waypoints
        package_share = get_package_share_directory('turtlebot3_gazebo')
        waypoint_file = os.path.join(package_share, 'scripts', 'waypoints.txt')
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
        self.is_caught = False  # Renamed from caught to avoid conflict

        # Set up publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/Evader/cmd_vel', 10)
        self.create_subscription(Odometry, '/Evader/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/red_obstacle_detected', self.caught_callback, 10)  # Fixed type and name
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Waypoint Navigator node has started.")

    def load_waypoints_from_file(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    parts = line.strip().split(',')
                    if len(parts) == 3:
                        x, y, theta = map(float, parts)
                        waypoints.append((x, y, theta))
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from file: {e}")
        return waypoints

    def caught_callback(self, msg):  # Renamed method
        self.is_caught = msg.data  # Access the data field
        if self.is_caught:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Robot Caught!")
            
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
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
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Robot stopped.")

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()