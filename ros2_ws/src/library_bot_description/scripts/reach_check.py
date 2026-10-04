#!/usr/bin/env python3
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
for y in (0.15, 0.30, 0.45):
    ok = node.move_to(node.make_pose(0.55, y, 0.60))
    print(f"reach x=0.55 y={y:.2f} z=0.60: {'OK' if ok else 'FAILED'}")
node.move_to(node.make_pose(*s7.SCAN_XYZ))
