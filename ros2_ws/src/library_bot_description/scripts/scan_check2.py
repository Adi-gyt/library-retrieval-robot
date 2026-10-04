#!/usr/bin/env python3
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
x, y, z0 = s7.SCAN_XYZ
s7.MARKER_ID = 3
for i in range(5):
    node.move_to(node.make_pose(x, y, z0 + (0.10 if i % 2 else 0.17)))
    node.move_to(node.make_pose(x, y, z0))
    node.spin_for(1.0)
    a = node.measure(n_samples=3, timeout=4.0)
    node.spin_for(3.0)
    b = node.measure(n_samples=3, timeout=4.0)
    f = lambda m: "not seen" if m is None else str(m[1].round(3))
    print(f"scan {i+1}: at 1s: {f(a)} | at 4s: {f(b)}")
