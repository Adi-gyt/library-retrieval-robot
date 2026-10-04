#!/usr/bin/env python3
"""Move the scan pose up and report which marker IDs the camera finds."""
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
orig = s7.MARKER_ID
x, y, _ = s7.SCAN_XYZ
for z in (0.45, 0.55, 0.62):
    ok = node.move_to(node.make_pose(x, y, z))
    print(f"scan z={z}: move {'ok' if ok else 'FAILED'}")
    if not ok:
        continue
    node.spin_for(1.0)
    for mid in (1, 2, 3):
        s7.MARKER_ID = mid
        m = node.measure(n_samples=3, timeout=4.0)
        print(f"   id {mid}: " + ("not seen" if m is None else str(m[1].round(3))))
s7.MARKER_ID = orig
node.move_to(node.make_pose(x, y, s7.SCAN_XYZ[2]))
