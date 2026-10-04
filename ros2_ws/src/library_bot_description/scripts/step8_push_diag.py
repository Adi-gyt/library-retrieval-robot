#!/usr/bin/env python3
"""Push toward the box in 2 cm steps and log which gripper part is near the box face."""
import rclpy
from rclpy.time import Time
import step7_marker_reach as s7
import step8_grasp as s8


def pos(node, f):
    t = node.tf_buffer.lookup_transform(s7.BASE, f, Time()).transform.translation
    return t.x, t.y, t.z


def main():
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    node.spin_for(1.0)
    if not node.move_to(node.make_pose(*s7.SCAN_XYZ)):
        return
    node.spin_for(1.0)
    m = node.measure()
    if m is None:
        log.error("no marker")
        return
    pb = m[1]
    gx = pb[0] + s8.BOX_HALF - s8.TCP_AHEAD
    gz = pb[2] + s8.GRASP_DZ
    node.grip(s8.OPEN)
    if not node.move_to(node.make_pose(gx - s8.PRE_BACK, pb[1], gz)):
        return
    node.spin_for(1.0)
    b1 = s8.box_pose()
    log.info(f"box before push: {b1}  face x={b1[0] - 0.04:.3f}")
    for k in range(1, 6):
        x = gx - s8.PRE_BACK + 0.02 * k
        if not node.line_to(x, pb[1], gz):
            return
        node.spin_for(1.0)
        b = s8.box_pose()
        d = [round((a - c) * 1000, 1) for a, c in zip(b, b1)]
        ee, pal = pos(node, "ee_link"), pos(node, "gripper_palm")
        fl, cam = pos(node, "finger_left"), pos(node, "d405_optical_frame")
        log.info(f"step {k}: ee x={ee[0]:.3f} palm_front={pal[0] + 0.015:.3f} "
                 f"tip_front={fl[0] + 0.045:.3f} (z {fl[2]:.3f}) cam x={cam[0]:.3f} z={cam[2]:.3f} "
                 f"| box face x={b[0] - 0.04:.3f} top z=0.609 moved {d} mm")
        if max(abs(v) for v in d) > 2.0:
            log.info("box moved, stopping")
            break


if __name__ == "__main__":
    main()
