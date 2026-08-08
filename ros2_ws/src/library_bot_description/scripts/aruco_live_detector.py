#!/usr/bin/env python3
"""
Sim Step 3, Phase B — detect ArUco markers in the live simulated camera
feed. Deliberately avoids cv_bridge (broken on this machine due to a
NumPy 1.x/2.x binary mismatch) — instead, the raw ROS Image message is
decoded straight into a NumPy array by hand.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ArucoLiveDetector(Node):
    def __init__(self):
        super().__init__('aruco_live_detector')
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        self.subscription = self.create_subscription(
            Image, '/d405/d405/image_raw', self.callback, 10
        )
        self.got_one = False

    def callback(self, msg):
        if self.got_one:
            return

        self.get_logger().info(f'Received frame: {msg.width}x{msg.height}, encoding={msg.encoding}')

        # Manual decode: msg.data is a flat byte array. Reshape it using
        # the image's actual width/height, letting numpy infer the
        # channel count (-1) from however many bytes are left over per
        # pixel — this works for both rgb8/bgr8 (3 channels) without
        # hardcoding it.
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        # Gazebo's camera plugin typically publishes rgb8; OpenCV expects
        # BGR order internally. Converting to grayscale for detection
        # sidesteps needing to get this exactly right, since aruco
        # detection only needs single-channel intensity data.
        if msg.encoding == 'rgb8':
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            self.get_logger().info(f'Detected marker(s): {ids.flatten().tolist()}')
            self.get_logger().info(f'Corner pixel coordinates: {corners}')
            annotated = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
            bgr_for_save = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else annotated
            cv2.imwrite('live_detection_result.png', bgr_for_save)
            self.get_logger().info('Saved live_detection_result.png')
        else:
            self.get_logger().info('No marker detected in this frame.')

        self.got_one = True
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArucoLiveDetector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
