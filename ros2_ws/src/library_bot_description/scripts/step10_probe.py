#!/usr/bin/env python3
import rclpy
import step7_marker_reach as s7
import step8_grasp as s8
import step10_stack as st

def show(tag):
    b1, b2 = st.box_pose("aruco_box_1"), st.box_pose("aruco_box_2")
    print(f"{tag}: b1 {None if b1 is None else [round(v, 3) for v in b1[:3]]} "
          f"b2 {None if b2 is None else [round(v, 3) for v in b2[:3]]}")

rclpy.init()
node = s8.Grasp()
log = node.get_logger()
node.spin_for(1.0)
show("start")
r = st.pick(node, log, 3)
show("after pick")
if r is None:
    raise SystemExit("pick failed")
gx, y0, gz = r
z_carry = max(gz + s8.LIFT, st.BASE_Z + 0.004 + 0.05)
print(f"LIFT={s8.LIFT} gz={gz:.3f} z_carry={z_carry:.3f}")
z = gz
while z < z_carry - 1e-6:
    z = min(z + 0.025, z_carry)
    node.line_to(gx, y0, z)
    node.spin_for(0.5)
    show(f"lift z={z:.3f}")
for k in range(1, 7):
    y = y0 + (st.PLACE_Y - y0) * k / 6
    node.line_to(gx, y, z_carry)
    node.spin_for(0.5)
    show(f"carry y={y:.3f}")
