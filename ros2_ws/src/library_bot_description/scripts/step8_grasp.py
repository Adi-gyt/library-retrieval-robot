#!/usr/bin/env python3
"""Step 8: scan -> open -> pre-grasp -> straight push -> close -> lift -> check."""
import subprocess
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import step7_marker_reach as s7

TCP_AHEAD = 0.092   # ee_link -> middle of finger pads
BOX_HALF = 0.04     # marker face -> box centre
PRE_BACK = 0.10     # pre-grasp is this far behind the grasp pose
GRASP_DZ = 0.0    # grasp height above marker centre
OPEN, CLOSE_TO = 0.065, 0.030
LIFT = 0.05
JOINT_SPEED = 0.15  # rad/s along straight-line moves


def dur(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class Grasp(s7.Step7):
    def __init__(self):
        super().__init__()
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.arm_ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip_ac = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.fing = {}
        self.create_subscription(
            JointState, "/joint_states",
            lambda m: self.fing.update({n: round(v, 4) for n, v in zip(m.name, m.position) if "finger" in n}), 10)

    def _send(self, ac, traj):
        if not ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("controller action server not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        if not h.accepted:
            self.get_logger().error("trajectory goal rejected")
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.error_code == 0

    def grip(self, q, secs=2.0):
        tr = JointTrajectory()
        tr.joint_names = ["finger_left_joint", "finger_right_joint"]
        tr.points = [JointTrajectoryPoint(positions=[q, q], time_from_start=dur(secs))]
        ok = self._send(self.grip_ac, tr)
        self.spin_for(0.5)
        return ok

    def line_to(self, x, y, z):
        pose = self.make_pose(x, y, z)
        req = GetCartesianPath.Request()
        req.header = pose.header
        req.start_state.is_diff = True
        req.group_name = "arm"
        req.link_name = "ee_link"
        req.waypoints = [pose.pose]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        if not self.cart.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_cartesian_path not available")
            return False
        fut = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        self.get_logger().info(f"  straight line to ({x:.3f}, {y:.3f}, {z:.3f}): fraction {res.fraction:.2f}")
        if res.fraction < 0.95:
            return False
        traj = res.solution.joint_trajectory
        qs = np.array([pt.positions for pt in traj.points])
        jump = float(np.max(np.abs(np.diff(qs, axis=0)))) if len(qs) > 1 else 0.0
        self.get_logger().info(f"  path: {len(qs)} points, max joint step {jump:.3f} rad (joint {int(np.argmax(np.max(np.abs(np.diff(qs, axis=0)), axis=0))) if len(qs) > 1 else 0}), "
                               f"shoulder range {qs[:, 1].min():+.2f}..{qs[:, 1].max():+.2f}")
        if jump > 0.15 or qs[:, 1].max() > 0.0:
            self.get_logger().error("  path jumps between IK branches, refusing to execute")
            return False
        t, prev = 0.0, None
        for p in traj.points:
            q = np.array(p.positions)
            if prev is not None:
                t += max(float(np.max(np.abs(q - prev))) / JOINT_SPEED, 0.05)
            p.time_from_start = dur(t)
            p.velocities, p.accelerations = [], []
            prev = q
        return self._send(self.arm_ac, traj)


def box_pose():
    out = subprocess.run(["gz", "model", "-m", "aruco_box_1", "-p"],
                         capture_output=True, text=True).stdout.split()
    return [float(v) for v in out[:3]]


def main():
    rclpy.init()
    node = Grasp()
    log = node.get_logger()
    node.spin_for(1.0)

    log.info("1/7 scan")
    if not node.move_to(node.make_pose(*s7.SCAN_XYZ)):
        return
    node.spin_for(1.0)
    m = node.measure()
    if m is None:
        log.error("marker not found, stopping")
        return
    pb = m[1]
    log.info(f"  marker face at ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f})")

    gx = pb[0] + BOX_HALF - TCP_AHEAD
    gz = pb[2] + GRASP_DZ
    log.info("2/7 open gripper")
    if not node.grip(OPEN):
        return
    log.info(f"3/7 pre-grasp ee_link x={gx - PRE_BACK:.3f}")
    if not node.move_to(node.make_pose(gx - PRE_BACK, pb[1], gz)):
        return
    node.spin_for(1.0)
    log.info(f"4/7 straight push to ee_link x={gx:.3f}")
    n = 5
    for k in range(1, n + 1):
        if not node.line_to(gx - PRE_BACK + PRE_BACK * k / n, pb[1], gz):
            return
        node.spin_for(0.3)
    node.spin_for(0.5)
    log.info("5/7 close gripper")
    node.grip(CLOSE_TO, secs=3.0)
    import subprocess
    r = subprocess.run(["ros2", "service", "call", "/gazebo/attach",
        "boeing_gazebo_model_attachment_plugin_msgs/srv/Attach",
        "{joint_name: grasp_fix, model_name_1: library_bot, link_name_1: ee_link, model_name_2: aruco_box_1, link_name_2: link}"],
        capture_output=True, text=True)
    log.info("attach: " + r.stdout.strip().replace("\n", " ")[-200:])
    node.spin_for(1.0)
    full = subprocess.run(["gz", "model", "-m", "aruco_box_1", "-p"], capture_output=True, text=True).stdout.split()
    log.info(f"  after close: fingers {node.fing}  box pose {full}")
    before = box_pose()
    log.info(f"6/7 lift {LIFT*100:.0f} cm (box before: z={before[2]:.3f})")
    for k in (1, 2):
        if not node.line_to(gx, pb[1], gz + LIFT * k / 2):
            return
        node.spin_for(0.3)
        log.info(f"  lift step {k}: fingers {node.fing}  box {box_pose()}")
    node.spin_for(1.5)
    after = box_pose()
    dz = after[2] - before[2]
    log.info(f"7/7 box after: ({after[0]:.3f}, {after[1]:.3f}, {after[2]:.3f}), rose {dz*1000:.0f} mm")
    log.info("GRASP HELD" if dz > 0.03 else "box NOT lifted with the arm")


if __name__ == "__main__":
    main()
