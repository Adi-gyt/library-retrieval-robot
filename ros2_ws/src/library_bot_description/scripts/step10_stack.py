#!/usr/bin/env python3
"""Move a 3-box stack from y=+0.15 to y=PLACE_Y, one box at a time, chosen by marker ID."""
import os
import subprocess
import time
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8
s7.SCAN_XYZ = (0.57, 0.0, float(os.environ.get("SCAN_Z", "0.60")))

ORDER = [3, 2, 1]                 # top box first
PLACE_Y = float(os.environ.get("PLACE_Y", "-0.15"))
BASE_Z = 0.568                    # hand height for a box sitting on the table
LAYER = 0.08                      # box height


PICK_X, PICK_Y = 0.96, 0.15   # expected marker position of the source stack


def box_pose(name):
    out = subprocess.run(["gz", "model", "-m", name, "-p"],
                         capture_output=True, text=True).stdout.split()
    return [float(v) for v in out] if len(out) == 6 else None


def call(srv, typ, body, tries=3):
    for i in range(tries):
        r = subprocess.run(["ros2", "service", "call", srv,
                            "boeing_gazebo_model_attachment_plugin_msgs/srv/" + typ, body],
                           capture_output=True, text=True)
        if "success=True" in r.stdout:
            return True
        tail = (r.stdout.strip().splitlines() or [""])[-1] + " | " + r.stderr.strip()[-200:]
        print(f"  service {srv} try {i + 1} failed: {tail}")
        time.sleep(1.0)
    return False


def attach(name):
    return call("/gazebo/attach", "Attach",
                "{joint_name: grasp_fix, model_name_1: library_bot, link_name_1: ee_link, "
                "model_name_2: %s, link_name_2: link}" % name)


def detach(name):
    return call("/gazebo/detach", "Detach",
                "{joint_name: grasp_fix, model_name_1: library_bot, model_name_2: %s}" % name)


def pick(node, log, mid):
    name = f"aruco_box_{mid}"
    s7.MARKER_ID = mid
    if not node.move_to(node.make_pose(*s7.SCAN_XYZ)):
        return None
    pb = None
    for attempt in range(3):
        node.spin_for(1.0)
        m = node.measure()
        if m is not None and abs(m[1][0] - PICK_X) < 0.02 and abs(m[1][1] - PICK_Y) < 0.03:
            pb = m[1]
            break
        log.warn(f"  box {mid}: reading {None if m is None else m[1].round(3)} is outside the expected zone; rescanning")
        node.move_to(node.make_pose(s7.SCAN_XYZ[0], s7.SCAN_XYZ[1], s7.SCAN_XYZ[2] + 0.05))
        node.move_to(node.make_pose(*s7.SCAN_XYZ))
    if pb is None:
        log.error(f"marker {mid} not seen in the expected zone; not moving the hand")
        return None
    gx = pb[0] + s8.BOX_HALF - s8.TCP_AHEAD
    gz = pb[2] + s8.GRASP_DZ
    log.info(f"  box {mid}: marker at ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f}) -> hand z={gz:.3f}")
    node.grip(s8.OPEN)
    if not node.move_to(node.make_pose(gx - s8.PRE_BACK, pb[1], gz)):
        return None
    node.spin_for(1.0)
    for k in range(1, 6):
        if not node.line_to(gx - s8.PRE_BACK + s8.PRE_BACK * k / 5, pb[1], gz):
            return None
        node.spin_for(0.3)
    node.spin_for(0.5)
    node.grip(s8.CLOSE_TO, secs=3.0)
    bp, fg = box_pose(name), node.fing
    held = (bp is not None
            and all(s8.CLOSE_TO + 0.003 < fg[k] < s8.OPEN - 0.010 for k in fg)
            and abs(bp[4]) < 0.1 and abs(bp[1] - pb[1]) < 0.02)
    if not held:
        log.error(f"no stable grasp (fingers {fg}, box {bp}); not attaching")
        node.grip(s8.OPEN)
        return None
    if not attach(name):
        log.error("attach failed")
        node.grip(s8.OPEN)
        return None
    node.spin_for(1.0)
    return gx, pb[1], gz


def place(node, log, mid, gx, y0, gz, layer):
    name = f"aruco_box_{mid}"
    z_place = BASE_Z + LAYER * layer + 0.004
    z_carry = max(gz + s8.LIFT, z_place + 0.05)
    z = gz
    while z < z_carry - 1e-6:
        z = min(z + 0.025, z_carry)
        if not node.line_to(gx, y0, z):
            return False
        node.spin_for(0.2)
    n = 6
    for k in range(1, n + 1):
        if not node.line_to(gx, y0 + (PLACE_Y - y0) * k / n, z_carry):
            return False
        node.spin_for(0.3)
    z = z_carry
    while z > z_place + 1e-6:
        z = max(z - 0.05, z_place)
        if not node.line_to(gx, PLACE_Y, z):
            return False
        node.spin_for(0.3)
    node.spin_for(1.0)
    log.info(f"  before release: {box_pose(name)}")
    log.info("  detach: " + ("ok" if detach(name) else "FAILED"))
    node.grip(s8.OPEN)
    node.spin_for(0.5)
    node.line_to(0.57, PLACE_Y, z_place)
    node.spin_for(1.0)
    return True


def main():
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    node.spin_for(1.0)
    for layer, mid in enumerate(ORDER):
        log.info(f"=== box {mid} -> layer {layer}")
        r = pick(node, log, mid)
        if r is None:
            log.error(f"pick of box {mid} failed; stopping")
            break
        if not place(node, log, mid, *r, layer):
            log.error(f"place of box {mid} failed; it may still be attached")
            break
        log.info(f"  placed box {mid}: {box_pose(f'aruco_box_{mid}')}")
    for mid in ORDER:
        log.info(f"FINAL box {mid}: {box_pose(f'aruco_box_{mid}')}")
    node.move_to(node.make_pose(*s7.SCAN_XYZ))


if __name__ == "__main__":
    main()
