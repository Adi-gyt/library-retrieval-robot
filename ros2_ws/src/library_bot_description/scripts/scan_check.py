#!/usr/bin/env python3
import subprocess
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

def truth(n):
    o = subprocess.run(["gz", "model", "-m", f"aruco_box_{n}", "-p"],
                       capture_output=True, text=True).stdout.split()
    return [float(v) for v in o]

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
x, y, _ = s7.SCAN_XYZ; z0 = 0.60
t = truth(3)
print(f"truth: marker face expected at x={t[0]-0.04:.3f} y={t[1]:.3f} z={t[2]:.3f}")
s7.MARKER_ID = 3
for i in range(5):
    node.move_to(node.make_pose(x, y, z0 + (0.10 if i % 2 else 0.17)))
    node.move_to(node.make_pose(x, y, z0))
    node.spin_for(1.0)
    m = node.measure(n_samples=3, timeout=4.0)
    print(f"scan {i+1}: " + ("not seen" if m is None else str(m[1].round(3))))
