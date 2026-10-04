#!/usr/bin/env python3
"""Slow close + hold: time trace of box pose, finger and arm joint positions."""
import subprocess
import threading
import time
import rclpy
from sensor_msgs.msg import JointState
import step7_marker_reach as s7
import step8_grasp as s8

rows = []
js = {}
stop = [False]


def box():
    out = subprocess.run(["gz", "model", "-m", "aruco_box_1", "-p"],
                         capture_output=True, text=True).stdout.split()
    return [float(v) for v in out]


def sampler(t0):
    while not stop[0]:
        b = box()
        if len(b) == 6:
            rows.append((time.time() - t0, dict(js), b))


def main():
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.update(zip(m.name, m.position)), 10)
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
    for k in range(1, 6):
        if not node.line_to(gx - s8.PRE_BACK + s8.PRE_BACK * k / 5, pb[1], gz):
            return
        node.spin_for(0.3)
    node.spin_for(1.0)
    t0 = time.time()
    th = threading.Thread(target=sampler, args=(t0,))
    th.start()
    node.spin_for(2.0)
    t_cmd = time.time() - t0
    node.grip(s8.CLOSE_TO, secs=6.0)
    t_ret = time.time() - t0
    node.spin_for(12.0)
    stop[0] = True
    th.join()
    print(f"close commanded at t={t_cmd:.1f}s, grip() returned at t={t_ret:.1f}s")
    print("    t    qL     qR     box_x  box_z  pitch   j2     j3     j4     j5")
    jn = ["joint2_shoulder", "joint3_elbow", "joint4_wrist1", "joint5_wrist2"]
    nan = float("nan")
    for i, (t, j, b) in enumerate(rows):
        if i % 3:
            continue
        a = [j.get(n, nan) for n in jn]
        print(f"{t:5.1f} {j.get('finger_left_joint', nan):.4f} "
              f"{j.get('finger_right_joint', nan):.4f} "
              f"{b[0]:.4f} {b[2]:.4f} {b[4]:+.4f} "
              + " ".join(f"{v:+.4f}" for v in a))
    node.grip(s8.OPEN)
    node.line_to(0.57, pb[1], gz)


if __name__ == "__main__":
    main()
