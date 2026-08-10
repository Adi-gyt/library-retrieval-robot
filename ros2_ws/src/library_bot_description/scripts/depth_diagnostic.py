#!/usr/bin/env python3
"""
Diagnostic — saves the RGB frame and a color-coded depth visualization
side by side. Finite depth values are shown as grayscale (darker = closer,
within 0.07-0.5m). Non-finite (-inf/inf/nan) pixels are marked bright red,
so we can see exactly WHERE the invalid readings are, correlated with
what's actually in the RGB frame at those same pixel locations.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters


class DepthDiagnostic(Node):
    def __init__(self):
        super().__init__('depth_diagnostic')
        self.got_one = False
        rgb_sub = message_filters.Subscriber(self, Image, '/d405/d405/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/d405/d405/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

    def callback(self, rgb_msg, depth_msg):
        if self.got_one:
            return

        rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite('diag_rgb.png', bgr)

        finite_mask = np.isfinite(depth)
        depth_vis = np.zeros((*depth.shape, 3), dtype=np.uint8)

        if finite_mask.any():
            # normalize finite values into 0-255 grayscale: near=bright, far=dark
            d = depth.copy()
            d[~finite_mask] = 0.5  # placeholder, overwritten below anyway
            normalized = np.clip((0.5 - d) / (0.5 - 0.07), 0, 1)  # 0.07m->1.0 (bright), 0.5m->0.0 (dark)
            gray = (normalized * 255).astype(np.uint8)
            depth_vis[:, :, 0] = gray
            depth_vis[:, :, 1] = gray
            depth_vis[:, :, 2] = gray

        # mark invalid pixels bright red, overwriting whatever grayscale was there
        depth_vis[~finite_mask] = [0, 0, 255]  # BGR red

        cv2.imwrite('diag_depth.png', depth_vis)

        self.get_logger().info('Saved diag_rgb.png and diag_depth.png')
        self.get_logger().info(f'{finite_mask.sum()}/{depth.size} finite. Red = invalid (-inf/inf/nan) in diag_depth.png')

        self.got_one = True
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DepthDiagnostic()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
