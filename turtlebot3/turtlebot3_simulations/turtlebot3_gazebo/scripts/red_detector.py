#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # Import Bool message type
import cv2
import numpy as np
from cv_bridge import CvBridge

class RedObstacleDetector(Node):
    def __init__(self):
        super().__init__("red_obstacle_detector")

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Subscriptions
        self.subscription_image = self.create_subscription(
            Image, "/Pursuer/camera/image_raw", self.image_callback, 10
        )
        self.subscription_depth = self.create_subscription(
            Image, "/Pursuer/camera/depth/image_raw", self.depth_callback, 10
        )

        # Publisher for obstacle detection state
        self.obstacle_pub = self.create_publisher(
            Bool, "/red_obstacle_detected", 10
        )

        # Internal state
        self.depth_data = None
        self.obstacle_center = None
        self.target_distance = 2  # 1.2 meters threshold
        self.obstacle_detected = False

    def depth_callback(self, msg):
        """Process depth image and update depth data."""
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_data = np.nan_to_num(self.depth_data, nan=np.inf)  # Replace NaNs with infinity
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        """Process RGB image and detect red obstacles."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Convert to HSV color space for better color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define range for red color (accounts for red's position in HSV space)
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            
            # Create masks for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Find contours in the binary image
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Reset detection state
            # previous_detection = self.obstacle_detected
            self.obstacle_detected = False
            self.obstacle_center = None
            
            if len(contours) > 0 and self.depth_data is not None:
                # Find the largest red contour
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center of the obstacle
                self.obstacle_center = (x + w//2, y + h//2)
                
                # Check if center is within depth image bounds
                if (0 <= self.obstacle_center[0] < self.depth_data.shape[1] and 
                    0 <= self.obstacle_center[1] < self.depth_data.shape[0]):
                    
                    # Get depth at center point
                    obstacle_depth = self.depth_data[self.obstacle_center[1], self.obstacle_center[0]]
                    
                    # Check if obstacle is within target distance
                    if obstacle_depth <= self.target_distance:
                        self.obstacle_detected = True
                        msg = Bool()
                        msg.data = self.obstacle_detected
                        self.obstacle_pub.publish(msg)
                        self.get_logger().info(f"Obstacle detection state changed to: {self.obstacle_detected}")
                        
                        # Draw rectangle and text
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                        cv2.putText(frame, "RED OBSTACLE DETECTED (<2m)", 
                                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (0, 0, 255), 2)
                        cv2.putText(frame, f"Distance: {obstacle_depth:.2f}m", 
                                   (x, y+h+25), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (0, 0, 255), 2)
                    else:
                        # Draw rectangle for objects beyond 2m
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                        cv2.putText(frame, f"Distance: {obstacle_depth:.2f}m", 
                                   (x, y+h+25), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (0, 255, 255), 2)
                        msg = Bool()
                        msg.data = self.obstacle_detected
                        self.obstacle_pub.publish(msg)
            
            # # Publish the detection state if it has changed
            # if self.obstacle_detected:
            #     msg = Bool()
            #     msg.data = self.obstacle_detected
            #     self.obstacle_pub.publish(msg)
            #     self.get_logger().info(f"Obstacle detection state changed to: {self.obstacle_detected}")
            
            # Display the result
            cv2.namedWindow('Red Obstacle Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Red Obstacle Detection', frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RedObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()