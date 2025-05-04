#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class RedDetector(Node):
    def __init__(self):
        super().__init__('red_detector')

        self.bridge = CvBridge()
        self.red_detected = False

        self.subscription = self.create_subscription(
            Image,
            '/Pursuer/camera/image_raw',  # Change if your camera topic is different
            self.image_callback,
            10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for RED color
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = mask1 + mask2

        red_area = cv2.countNonZero(red_mask)

        threshold = 800  # number of red pixels to trigger detection (adjust if needed)

        if red_area > threshold:
            self.get_logger().info('Red object detected!')
            self.red_detected = True
        else:
            self.red_detected = False

    def is_red_detected(self):
        return self.red_detected
