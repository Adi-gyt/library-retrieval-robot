#!/usr/bin/env python3
"""
send_hardcoded_pose.py

Closes out Sim Step 6: sends one hardcoded target pose to MoveIt2's
move_group action server and lets it plan + execute against KDL.

Written as a direct moveit_msgs/action/MoveGroup client rather than using
moveit_commander (ROS1 only) or moveit_py (not available on Humble,
introduced later) -- this is the standard low-level approach for
Python + ROS2 Humble.

Target pose is expressed in `base_footprint`, matching Sim Step 5's
convention (that's the frame the vision pipeline's transformed pose
already lands in downstream, per the project plan).

Usage:
  ros2 run library_bot_moveit_config send_hardcoded_pose.py
  # or, if not colcon-installed yet:
  python3 scripts/send_hardcoded_pose.py
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class HardcodedPoseCommander(Node):
    def __init__(self):
        super().__init__("send_hardcoded_pose")
        self._client = ActionClient(self, MoveGroup, "move_action")

    def send_target(self, pose: PoseStamped, group_name="arm", ee_link="ee_link"):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_action server not available -- is move_group running?")
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header.frame_id = pose.header.frame_id
        goal.request.workspace_parameters.min_corner.x = -1.5
        goal.request.workspace_parameters.min_corner.y = -1.5
        goal.request.workspace_parameters.min_corner.z = -1.5
        goal.request.workspace_parameters.max_corner.x = 1.5
        goal.request.workspace_parameters.max_corner.y = 1.5
        goal.request.workspace_parameters.max_corner.z = 1.5

        # --- Position constraint: small sphere around the target point ---
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = ee_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        # --- Orientation constraint: loose tolerance around target quaternion ---
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose.header
        orient_constraint.link_name = ee_link
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False  # plan AND execute
        goal.planning_options.replan = False

        self.get_logger().info(
            f"Sending target pose in '{pose.header.frame_id}': "
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by move_group.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # error_code.val == 1 is MoveItErrorCodes.SUCCESS
        if result.error_code.val == 1:
            self.get_logger().info("Motion plan executed successfully.")
            return True
        else:
            self.get_logger().error(f"MoveIt2 returned error code: {result.error_code.val}")
            return False


def main():
    rclpy.init()
    node = HardcodedPoseCommander()

    # --- HARDCODED TARGET POSE ---
    # Adjust these to a pose you know is reachable given L1=L2=0.53m
    # (fully-extended horizontal reach x~=1.06m per the plan doc) before
    # relying on this for anything beyond a first smoke test.
    target = PoseStamped()
    target.header.frame_id = "base_footprint"
    target.pose.position.x = 0.9
    target.pose.position.y = 0.0
    target.pose.position.z = 0.57  # NOTE: base_footprint frame; shoulder height, confirmed reachable via /compute_ik
    target.pose.orientation.x = 1.0  # 180 deg about X -- confirmed achievable via /compute_ik
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    success = node.send_target(target)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
