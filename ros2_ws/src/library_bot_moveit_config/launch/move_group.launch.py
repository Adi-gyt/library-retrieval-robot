"""
move_group.launch.py

Brings up the full MoveIt2 pipeline for library_bot with no RViz/display
dependency: robot_state_publisher, ros2_control_node (mock hardware),
controller spawners, and the move_group node itself.

This is the launch file to close out Sim Step 6: once this is running,
scripts/send_hardcoded_pose.py can command a target pose against it.

Usage:
  ros2 launch library_bot_moveit_config move_group.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
import xacro
import yaml


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config_share = get_package_share_directory("library_bot_moveit_config")

    # --- Robot description (URDF via xacro) ---
    # Uses the wrapper file (real geometry + ros2_control tag combined),
    # not library_bot_description's raw xacro directly -- see
    # config/library_bot_with_ros2_control.xacro for why.
    xacro_path = os.path.join(
        moveit_config_share, "config", "library_bot_with_ros2_control.xacro"
    )
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # --- SRDF ---
    srdf_path = os.path.join(moveit_config_share, "config", "library_bot.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # --- Kinematics / joint limits / controllers config ---
    kinematics_yaml = load_yaml(
        "library_bot_moveit_config", "config/kinematics.yaml"
    )
    joint_limits_yaml = load_yaml(
        "library_bot_moveit_config", "config/joint_limits.yaml"
    )
    moveit_controllers_yaml = load_yaml(
        "library_bot_moveit_config", "config/moveit_controllers.yaml"
    )

    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }

    moveit_controller_manager = {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    # --- ros2_control ---
    ros2_controllers_path = os.path.join(
        moveit_config_share, "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipeline_config,
            moveit_controllers_yaml,
            moveit_controller_manager,
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_node,
        ]
    )
