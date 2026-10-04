#!/usr/bin/env python3
"""Open the gripper and back the arm straight out to x=0.57."""
import rclpy
from rclpy.time import Time
import step7_marker_reach as s7
import step8_grasp as s8


def main():
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    node.spin_for(1.0)
    t = node.tf_buffer.lookup_transform(s7.BASE, "ee_link", Time()).transform.translation
    log.info(f"ee_link now at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
    log.info("opening gripper")
    node.grip(s8.OPEN)
    log.info("straight retreat")
    ok = node.line_to(0.57, t.y, t.z)
    log.info("retreat done" if ok else "retreat failed")


if __name__ == "__main__":
    main()
