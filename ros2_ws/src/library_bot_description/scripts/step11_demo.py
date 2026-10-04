#!/usr/bin/env python3
"""Table A -> table B: move boxes by marker ID onto the pedestal slots, in the order given."""
import os
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8
import step10_stack as st

ORDER = [int(a) for a in os.environ.get("ORDER", "2,3").split(",")]
SRC_Y = {1: 0.20, 2: 0.0, 3: -0.20}   # box positions on table A
PED_Y = 0.45                           # pedestal centre y
SLOTS_X = [0.60, 0.50]                 # box-centre x on the pedestal, far slot first
PED_Z = 0.492                          # hand z to rest a box on the pedestal (top is 0.45)
CARRY_Z = 0.68                         # hand z while carrying
SCAN_Z = 0.60


def set_source(mid):
    st.PICK_Y = SRC_Y[mid]
    s7.SCAN_XYZ = (0.57, SRC_Y[mid], SCAN_Z)


def place_ped(node, log, mid, gx, y0, gz, slot_x):
    name = f"aruco_box_{mid}"
    hx = slot_x - s8.TCP_AHEAD
    z = gz
    while z < CARRY_Z - 1e-6:
        z = min(z + 0.025, CARRY_Z)
        if not node.line_to(gx, y0, z):
            return False
        node.spin_for(0.2)
    n = 8
    for k in range(1, n + 1):
        f = k / n
        if not node.line_to(gx + (hx - gx) * f, y0 + (PED_Y - y0) * f, CARRY_Z):
            return False
        node.spin_for(0.3)
    z = CARRY_Z
    while z > PED_Z + 1e-6:
        z = max(z - 0.05, PED_Z)
        if not node.line_to(hx, PED_Y, z):
            return False
        node.spin_for(0.3)
    node.spin_for(1.0)
    log.info(f"  before release: {st.box_pose(name)}")
    log.info("  detach: " + ("ok" if st.detach(name) else "FAILED"))
    node.grip(s8.OPEN)
    node.spin_for(0.5)
    z = PED_Z
    while z < CARRY_Z - 1e-6:
        z = min(z + 0.05, CARRY_Z)
        if not node.line_to(hx, PED_Y, z):
            return False
        node.spin_for(0.2)
    return True


def preflight(node, log, mid, slot_x):
    s7.MARKER_ID = mid
    node.move_to(node.make_pose(*s7.SCAN_XYZ))
    node.spin_for(1.0)
    m = node.measure(n_samples=3, timeout=4.0)
    log.info(f"  scan: {None if m is None else m[1].round(3)} (expect x 0.960, y {SRC_Y[mid]:+.2f})")
    hx = slot_x - s8.TCP_AHEAD
    node.move_to(node.make_pose(hx, PED_Y, 0.60))
    ok = node.move_to(node.make_pose(hx, PED_Y, PED_Z))
    log.info(f"  slot reach hand ({hx:.3f}, {PED_Y}, {PED_Z}): {'OK' if ok else 'FAILED'}")


def main():
    pre = os.environ.get("PREFLIGHT") == "1"
    rclpy.init()
    node = s8.Grasp()
    log = node.get_logger()
    node.spin_for(1.0)
    for i, mid in enumerate(ORDER):
        slot_x = SLOTS_X[i]
        set_source(mid)
        log.info(f"=== box {mid}: table A y={SRC_Y[mid]:+.2f} -> pedestal slot x={slot_x:.2f}")
        if pre:
            preflight(node, log, mid, slot_x)
            continue
        r = st.pick(node, log, mid)
        if r is None:
            log.error(f"pick of box {mid} failed; stopping")
            break
        if not place_ped(node, log, mid, *r, slot_x):
            log.error(f"place of box {mid} failed; it may still be attached")
            break
        log.info(f"  placed box {mid}: {st.box_pose(f'aruco_box_{mid}')}")
    for mid in ORDER:
        log.info(f"FINAL box {mid}: {st.box_pose(f'aruco_box_{mid}')}")
    node.move_to(node.make_pose(0.57, 0.0, SCAN_Z))


if __name__ == "__main__":
    main()
