#!/usr/bin/env python3
"""Same approach as step8, but close the fingers in small steps and log everything."""
import rclpy
from sensor_msgs.msg import JointState
import step7_marker_reach as s7
import step8_grasp as s8


def main():
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    fing = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: fing.update({n: p for n, p in zip(m.name, m.position) if "finger" in n}), 10)
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
    node.spin_for(0.5)
    log.info(f"fingers after open: { {k: round(v, 4) for k, v in fing.items()} }")
    b0 = s8.box_pose()
    log.info(f"box at start: {b0}")
    if not node.move_to(node.make_pose(gx - s8.PRE_BACK, pb[1], gz)):
        return
    node.spin_for(1.0)
    b1 = s8.box_pose()
    log.info(f"box after pre-grasp: {b1}")
    if not node.line_to(gx, pb[1], gz):
        return
    node.spin_for(1.0)
    b2 = s8.box_pose()
    log.info(f"box after push: {b2}  moved {[round((a - b) * 1000, 1) for a, b in zip(b2, b1)]} mm")
    for q in (0.055, 0.050, 0.046, 0.043, 0.041, 0.040, 0.039, 0.038, 0.037, 0.036, 0.035):
        node.grip(q, secs=2.0)
        node.spin_for(0.5)
        b = s8.box_pose()
        d = [round((a - c) * 1000, 1) for a, c in zip(b, b2)]
        log.info(f"cmd {q:.3f}: fingers { {k: round(v, 4) for k, v in fing.items()} }  box moved {d} mm")
        if max(abs(v) for v in d) > 5.0:
            log.info("box moved more than 5 mm, stopping here")
            break


if __name__ == "__main__":
    main()
