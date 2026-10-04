#!/usr/bin/env python3
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8

rclpy.init()
node = s8.Grasp()
node.spin_for(1.0)
for y in (0.30, 0.45):
    for x in (0.45, 0.55):
        for z in (0.60, 0.52, 0.49):
            ok = node.move_to(node.make_pose(x, y, z))
            print(f"reach x={x:.2f} y={y:.2f} z={z:.2f}: {'OK' if ok else 'FAILED'}")
node.move_to(node.make_pose(*s7.SCAN_XYZ))
