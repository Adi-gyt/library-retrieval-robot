import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2

class ArucoBackproject(Node):
    def __init__(self):
        super().__init__('aruco_backproject')
        self.depth_img = None
        self.intrinsics = None  # (fx, fy, cx, cy)

        self.create_subscription(
            CameraInfo, '/d405/d405/depth/camera_info', self.info_cb, 1)
        self.create_subscription(
            Image, '/d405/d405/depth/image_raw', self.depth_cb, 1)
        self.create_subscription(
            Image, '/d405/d405/image_raw', self.rgb_cb, 1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.reported = False

    def info_cb(self, msg):
        if self.intrinsics is None:
            k = msg.k  # row-major 3x3
            self.intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy
            self.get_logger().info(f"Got intrinsics: fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.2f} cy={k[5]:.2f}")

    def depth_cb(self, msg):
        self.depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def rgb_cb(self, msg):
        if self.intrinsics is None or self.depth_img is None:
            return  # wait until we have both

        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        # Center pixel of the first detected marker
        c = corners[0][0]
        self.get_logger().info(f"Raw corners: {c.tolist()}")
        u = int(np.mean(c[:, 0]))
        v = int(np.mean(c[:, 1]))

        depth = self.depth_img[v, u]
        if not np.isfinite(depth):
            self.get_logger().warn(f"Marker at pixel ({u},{v}) has invalid depth: {depth}")
            return

        fx, fy, cx, cy = self.intrinsics
        # Pinhole back-projection, result in camera OPTICAL frame (Z forward, X right, Y down)
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        if not self.reported:
            marker_id = int(np.array(ids).flatten()[0])
            self.get_logger().info(
                f"Marker id={marker_id} pixel=({u},{v}) depth={depth:.4f} "
                f"-> camera-optical-frame point: X={X:.4f} Y={Y:.4f} Z={Z:.4f}"
            )
            self.reported = True
            rclpy.shutdown()

rclpy.init()
node = ArucoBackproject()
rclpy.spin(node)
