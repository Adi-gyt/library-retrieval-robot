#!/usr/bin/env python3
"""
Sim Step 2 check — reads a single depth frame from the D405 sensor and
prints the distance at the center pixel, to confirm against the known
test-target placement (0.3m in front of the camera).

This is a minimal ROS2 "node" — a running program that subscribes to a
topic, in this case /d405/d405/depth/image_raw, and does something with
each message it receives.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        # bridge converts the raw ROS Image message into a numpy array
        # OpenCV can actually work with — this is the standard tool for
        # exactly that conversion, used constantly in ROS2 vision code.
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/d405/d405/depth/image_raw',
            self.depth_callback,
            10  # queue size — how many unread messages to buffer
        )
        self.got_one = False

    def depth_callback(self, msg):
        if self.got_one:
            return  # we only want one frame, then we're done

        # desired_encoding="passthrough" keeps the original 32FC1 format
        # (32-bit float, 1 channel) instead of converting to something else
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        height, width = depth_image.shape
        center_y, center_x = height // 2, width // 2
        center_distance = depth_image[center_y, center_x]

        self.get_logger().info(
            f'Image size: {width}x{height}. '
            f'Distance at center pixel ({center_x}, {center_y}): {center_distance:.3f} m'
        )
        self.get_logger().info(
            'Expected: ~0.300 m (test target placed 0.3m in front of the camera)'
        )

        self.got_one = True
        rclpy.shutdown()  # stop the node once we have our answer


def main():
    rclpy.init()
    node = DepthChecker()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
