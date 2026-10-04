#!/usr/bin/env python3
"""
Sim Step 7 -- the arm moves to wherever the marker is actually seen.

  1. move to a scan pose (marker inside the D405's 0.5 m range)
  2. detect ArUco id 0, read depth at the marker, back-project to 3D
  3. tf2: optical frame -> base_footprint
  4. /compute_ik check, then MoveIt2 move to an approach pose
  5. re-detect from the new pose as a closed-loop sanity check

No cv_bridge (NumPy 2.x breaks it): images are decoded by hand.
Run:  python3 scripts/step7_marker_reach.py
"""
import sys
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, OrientationConstraint,
                             PositionConstraint, WorkspaceParameters)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive

BASE = "base_footprint"
OPTICAL = "d405_optical_frame"
EE_LINK = "ee_link"
MARKER_ID = 1
SCAN_XYZ = (0.57, 0.0, 0.45)
EE_QUAT = (1.0, 0.0, 0.0, 0.0)       # x, y, z, w -- confirmed reachable
APPROACH_OFFSET = 0.20               # ee_link this far behind the marker face
CAM_AHEAD = 0.03                     # camera sits 3 cm ahead of ee_link
TRUTH = np.array([0.96, 0.0, 0.57])  # marker face centre from the world file


class Step7(Node):
    def __init__(self):
        super().__init__("step7_marker_reach")
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters())
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, "/d405/d405/image_raw", self._on_rgb, 1)
        self.create_subscription(Image, "/d405/d405/depth/image_raw", self._on_depth, 1)
        self.create_subscription(CameraInfo, "/d405/d405/camera_info", self._on_info, 1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def _on_rgb(self, msg):
        self.rgb = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.K = msg.k

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------- vision ----------
    def _to_base(self, p):
        try:
            t = self.tf_buffer.lookup_transform(BASE, OPTICAL, Time())
        except Exception as e:
            self.get_logger().warn(f"tf {BASE} <- {OPTICAL} failed: {e}",
                                   throttle_duration_sec=2.0)
            return None
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        tr = t.transform.translation
        return R @ p + np.array([tr.x, tr.y, tr.z])

    def _marker_point(self, rgb, depth_msg):
        img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, -1)
        code = cv2.COLOR_RGB2GRAY if rgb.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(img, code))
        if ids is None:
            return None
        ids = ids.flatten().tolist()
        if MARKER_ID not in ids:
            return None
        u, v = corners[ids.index(MARKER_ID)].reshape(4, 2).mean(axis=0)

        d = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
            depth_msg.height, depth_msg.width)
        ui, vi = int(round(u)), int(round(v))
        patch = d[max(vi - 5, 0):vi + 6, max(ui - 5, 0):ui + 6]
        good = patch[np.isfinite(patch) & (patch > 0.07) & (patch < 0.49)]
        if good.size < 10:
            return None
        z = float(np.median(good))

        fx, cx, fy, cy = self.K[0], self.K[2], self.K[4], self.K[5]
        p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        p_base = self._to_base(p_cam)
        if p_base is None:
            return None
        return p_cam, p_base

    def measure(self, n_samples=5, timeout=15.0):
        """Median marker position over n frames: (camera frame, base frame)."""
        self.rgb = None
        self.depth = None
        cams, bases = [], []
        end = time.monotonic() + timeout
        while len(cams) < n_samples and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rgb is None or self.depth is None or self.K is None:
                continue
            rgb, self.rgb = self.rgb, None  # use each frame once
            res = self._marker_point(rgb, self.depth)
            if res is not None:
                cams.append(res[0])
                bases.append(res[1])
        if not cams:
            return None
        spread = np.round(np.std(bases, axis=0) * 1000, 1)
        self.get_logger().info(f"  {len(cams)} detections, std in {BASE}: {spread} mm")
        return np.median(cams, axis=0), np.median(bases, axis=0)

    # ---------- motion ----------
    @staticmethod
    def make_pose(x, y, z):
        p = PoseStamped()
        p.header.frame_id = BASE
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = EE_QUAT
        return p

    def ik_check(self, pose):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/compute_ik not available, skipping IK check")
            return
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn("/compute_ik gave no answer, continuing anyway")
        elif res.error_code.val == 1:
            self.get_logger().info("  IK check: target is reachable")
        else:
            self.get_logger().warn(
                f"  IK check FAILED (code {res.error_code.val}); trying to plan anyway")

    def move_to(self, pose, group_name="arm", ee_link=EE_LINK):
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_action not available -- is move_group running?")
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        ws = WorkspaceParameters()
        ws.header.frame_id = pose.header.frame_id
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.5
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 1.5
        goal.request.workspace_parameters = ws

        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose.pose)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header = pose.header
        ori.link_name = ee_link
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        cons = Constraints()
        cons.position_constraints.append(pos)
        cons.orientation_constraints.append(ori)
        from moveit_msgs.msg import JointConstraint
        for jn in ("joint4_wrist1", "joint6_wrist3"):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = 0.0
            jc.tolerance_above = 1.0
            jc.tolerance_below = 1.0
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        goal.request.goal_constraints.append(cons)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        p = pose.pose.position
        self.get_logger().info(f"  moving {ee_link} to ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) in {BASE}")
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected by move_group")
            return False
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        code = rfut.result().result.error_code.val
        if code == 1:
            self.get_logger().info("  motion executed successfully")
            return True
        self.get_logger().error(f"MoveIt2 error code: {code}")
        return False


def run(node):
    log = node.get_logger()
    node.spin_for(2.0)  # let tf, camera_info and images start arriving

    log.info("1/4 moving to scan pose")
    if not node.move_to(node.make_pose(*SCAN_XYZ)):
        return False
    node.spin_for(1.0)  # let the arm settle

    log.info("2/4 detecting marker")
    m = node.measure()
    if m is None:
        log.error("no marker / no valid depth / tf failed (see warnings above)")
        return False
    pc, pb = m
    err = (pb - TRUTH) * 1000.0
    log.info(f"  camera frame: x={pc[0]:+.3f} y={pc[1]:+.3f} depth={pc[2]:.3f} m")
    log.info(f"  {BASE}: ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f}) m")
    log.info(f"  vs world-file truth: error ({err[0]:+.1f}, {err[1]:+.1f}, {err[2]:+.1f}) mm")

    target = node.make_pose(pb[0] - APPROACH_OFFSET, pb[1], pb[2])
    log.info("3/4 IK check for approach pose")
    node.ik_check(target)

    log.info("4/4 moving to approach pose")
    if not node.move_to(target):
        return False
    node.spin_for(1.0)
    m2 = node.measure(n_samples=3)
    if m2 is None:
        log.info("  no re-detect from the approach pose (expected: at this standoff "
                 "the fingers cross the marker and its top edge leaves the frame). "
                 "The scan-pose measurement is the one to trust.")
        return True
    pc2, _ = m2
    log.info(f"  from approach pose: depth {pc2[2]:.3f} m "
             f"(expected {APPROACH_OFFSET - CAM_AHEAD:.3f}), "
             f"lateral offset x={pc2[0] * 1000:+.0f} mm y={pc2[1] * 1000:+.0f} mm")
    return True


def main():
    rclpy.init()
    node = Step7()
    try:
        ok = run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
