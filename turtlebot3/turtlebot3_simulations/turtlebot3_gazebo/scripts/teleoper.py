#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select  # Importing the select module
import termios
import tty

class TeleopPursuer(Node):
    def __init__(self):
        super().__init__('teleop_pursuer')
        
        # Publisher to send commands to the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/Pursuer/cmd_vel', 10)

        # Create subscriptions for keypress inputs
        self.create_timer(0.1, self.teleop_control)  # 10 Hz update

        # Initialize speed values
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        self.get_logger().info("Teleop Pursuer Node started. Use WASD keys or joystick for control.")

    def get_key(self):
        """ Read a single keypress from stdin. """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def teleop_control(self):
        """ Update control values based on keyboard input. """
        key = self.get_key()  # Get keypress from user
        cmd = Twist()

        # Linear movement (forward/backward)
        if key == 'w':
            self.linear_speed = 0.5
        elif key == 's':
            self.linear_speed = -0.5
        else:
            self.linear_speed = 0.0

        # Angular movement (turn left/right)
        if key == 'a':
            self.angular_speed = 0.5
        elif key == 'd':
            self.angular_speed = -0.5
        else:
            self.angular_speed = 0.0

        # Stop movement with the 'q' key
        if key == 'q':
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        # Publish the Twist command to the robot
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(cmd)

        # Output the current movement state to the console
        self.get_logger().info(f"Linear Speed: {self.linear_speed}, Angular Speed: {self.angular_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPursuer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
