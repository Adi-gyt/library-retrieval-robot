import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  -- registers PointStamped transform support


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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def info_cb(self, msg):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = (k[0], k[4], k[2], k[5])
            self.get_logger().info(f"Got intrinsics: fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.2f} cy={k[5]:.2f}")

    def depth_cb(self, msg):
        self.depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def rgb_cb(self, msg):
        if self.intrinsics is None or self.depth_img is None:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return
        c = corners[0][0]
        self.get_logger().info(f"Raw corners: {c.tolist()}")
        u = int(np.mean(c[:, 0]))
        v = int(np.mean(c[:, 1]))
        depth = self.depth_img[v, u]
        if not np.isfinite(depth):
            self.get_logger().warn(f"Marker at pixel ({u},{v}) has invalid depth: {depth}")
            return
        fx, fy, cx, cy = self.intrinsics
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        if not self.reported:
            marker_id = int(np.array(ids).flatten()[0])
            self.get_logger().info(
                f"Marker id={marker_id} pixel=({u},{v}) depth={depth:.4f} "
                f"-> camera-optical-frame point: X={X:.4f} Y={Y:.4f} Z={Z:.4f}"
            )

            point_cam = PointStamped()
            point_cam.header.frame_id = 'd405_optical_frame'
            point_cam.header.stamp = Time().to_msg()
            point_cam.point.x = float(X)
            point_cam.point.y = float(Y)
            point_cam.point.z = float(Z)

            try:
                point_base = self.tf_buffer.transform(
                    point_cam, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info(
                    f"-> base_footprint-frame point: "
                    f"X={point_base.point.x:.4f} Y={point_base.point.y:.4f} Z={point_base.point.z:.4f}"
                )
            except Exception as e:
                self.get_logger().error(f"tf2 transform failed: {e}")

            self.reported = True
            rclpy.shutdown()


rclpy.init()
node = ArucoBackproject()
rclpy.spin(node)
