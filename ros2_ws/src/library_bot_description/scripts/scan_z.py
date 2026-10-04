#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
x, y, _ = s7.SCAN_XYZ


def grab(timeout=5.0):
    node.rgb = None
    node.depth = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.rgb is not None and node.depth is not None:
            return node.rgb, node.depth
    return None, None


for z in (0.45, 0.50, 0.55, 0.60, 0.45, 0.50, 0.55, 0.60):
    ok = node.move_to(node.make_pose(x, y, z + 0.10))
    ok = node.move_to(node.make_pose(x, y, z)) and ok
    node.spin_for(1.0)
    rgb, dep = grab()
    if rgb is None:
        print(f"z={z:.2f}: no frame")
        continue
    img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, -1)
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if rgb.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)
    c, ids, _ = node.detector.detectMarkers(g)
    found = {}
    if ids is not None:
        for cc, m in zip(c, ids.flatten().tolist()):
            u, v = cc.reshape(4, 2).mean(axis=0)
            found[m] = (int(u), int(v))
    print(f"z={z:.2f} reached={ok}: {dict(sorted(found.items()))}")
