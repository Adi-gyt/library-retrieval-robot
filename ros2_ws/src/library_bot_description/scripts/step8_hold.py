#!/usr/bin/env python3
"""Grasp up to the close, log full box pose before/after, hold 25 s for a contact check."""
import subprocess
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8


def full():
    out = subprocess.run(["gz", "model", "-m", "aruco_box_1", "-p"],
                         capture_output=True, text=True).stdout.split()
    return [round(float(v), 4) for v in out]


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
    log.info(f"box before push (x y z roll pitch yaw): {full()}")
    for k in range(1, 6):
        if not node.line_to(gx - s8.PRE_BACK + s8.PRE_BACK * k / 5, pb[1], gz):
            return
        node.spin_for(0.3)
    node.spin_for(1.0)
    log.info(f"box after push, BEFORE close: {full()}  fingers {node.fing}")
    node.grip(s8.CLOSE_TO, secs=3.0)
    node.spin_for(1.0)
    log.info(f"box after close: {full()}  fingers {node.fing}")
    log.info("HOLDING 25 s: run the contacts command in another terminal now")
    node.spin_for(5.0)
    subprocess.run("timeout 5 stdbuf -oL gz topic -e /gazebo/default/physics/contacts > /tmp/contacts.txt 2>&1", shell=True)
    node.spin_for(5.0)
    log.info(f"box after hold: {full()}")
    node.grip(s8.OPEN)
    node.line_to(0.57, pb[1], gz)
    log.info("opened and backed out")


if __name__ == "__main__":
    main()
