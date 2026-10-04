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
x, y, z0 = s7.SCAN_XYZ


def grab(timeout=5.0):
    node.rgb = None
    node.depth = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.rgb is not None and node.depth is not None:
            return node.rgb, node.depth
    return None, None


for i in range(6):
    node.move_to(node.make_pose(x, y, z0 + (0.10 if i % 2 else 0.17)))
    node.move_to(node.make_pose(x, y, z0))
    node.spin_for(1.0)
    rgb, dep = grab()
    if rgb is None:
        print(f"landing {i+1}: no frame")
        continue
    img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, -1)
    is_rgb = rgb.encoding == "rgb8"
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if is_rgb else cv2.COLOR_BGR2GRAY)
    corners, ids, _ = node.detector.detectMarkers(gray)
    found = {}
    if ids is not None:
        for c, mid in zip(corners, ids.flatten().tolist()):
            u, v = c.reshape(4, 2).mean(axis=0)
            found[mid] = (int(u), int(v))
    print(f"landing {i+1}: img {rgb.width}x{rgb.height} markers (u,v): {found}")
    if 3 in found:
        d = np.frombuffer(dep.data, dtype=np.float32).reshape(dep.height, dep.width)
        u, v = found[3]
        patch = d[max(v - 5, 0):v + 6, max(u - 5, 0):u + 6]
        fin = patch[np.isfinite(patch)]
        good = fin[(fin > 0.07) & (fin < 0.49)]
        if fin.size:
            print(f"  box3 depth: finite {fin.size}, in-range {good.size}, "
                  f"min {fin.min():.3f} med {np.median(fin):.3f} max {fin.max():.3f}")
        else:
            print("  box3 depth: no finite values")
    out = np.ascontiguousarray(img[:, :, ::-1]) if is_rgb else img.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(out, corners, ids)
    cv2.imwrite(f"/tmp/scan_{i+1}.png", out)
