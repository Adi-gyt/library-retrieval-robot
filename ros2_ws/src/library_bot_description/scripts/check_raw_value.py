import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class RawCheck(Node):
    def __init__(self):
        super().__init__('raw_check')
        self.sub = self.create_subscription(
            Image, '/d405/d405/depth/image_raw', self.cb, 1)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        cy, cx = msg.height // 2, msg.width // 2
        print(f"center pixel value: {arr[cy, cx]}")
        print(f"center pixel dtype: {arr.dtype}")
        print(f"is -inf: {np.isneginf(arr[cy, cx])}, is +inf: {np.isposinf(arr[cy, cx])}, is nan: {np.isnan(arr[cy, cx])}")
        print(f"min in frame: {np.nanmin(arr[np.isfinite(arr)]) if np.any(np.isfinite(arr)) else 'none finite'}")
        print(f"max in frame: {np.nanmax(arr[np.isfinite(arr)]) if np.any(np.isfinite(arr)) else 'none finite'}")
        self.done = True
        rclpy.shutdown()

rclpy.init()
node = RawCheck()
rclpy.spin(node)
