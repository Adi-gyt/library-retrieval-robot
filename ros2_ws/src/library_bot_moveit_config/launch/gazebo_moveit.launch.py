"""
gazebo_moveit.launch.py

Sim Step 6.5: Gazebo physics + gazebo_ros2_control + MoveIt2 in ONE launch.

Order of events (each step waits for the previous one to finish):
  1. Gazebo starts with library_world.world (marker included)
  2. robot_state_publisher publishes the URDF (use_sim:=true variant)
  3. spawn_entity.py places library_bot in Gazebo; the gazebo_ros2_control
     plugin inside the model starts a controller_manager
  4. joint_state_broadcaster spawner
  5. arm_controller spawner
  6. move_group

Do NOT also run sim.launch.py or move_group.launch.py; this file replaces
both. Never run two D405 robots in one Gazebo instance (segfault).

Usage:
  ros2 launch library_bot_moveit_config gazebo_moveit.launch.py
"""

import os
import re

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

USE_SIM_TIME = {"use_sim_time": True}


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    with open(os.path.join(package_share, file_path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_share = get_package_share_directory("library_bot_moveit_config")
    desc_share = get_package_share_directory("library_bot_description")

    # --- Robot description: wrapper xacro with the Gazebo variant switched on
    xacro_path = os.path.join(
        moveit_share, "config", "library_bot_with_ros2_control.xacro"
    )
    robot_description = {
        "robot_description": re.sub(
            r"<!--.*?-->",
            "",
            xacro.process_file(
                xacro_path, mappings={"use_sim": "true"}
            ).toxml(),
            flags=re.S,
        )
    }

    # --- SRDF
    with open(os.path.join(moveit_share, "config", "library_bot.srdf"), "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # --- MoveIt configs (same as move_group.launch.py)
    kinematics_yaml = load_yaml("library_bot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("library_bot_moveit_config", "config/joint_limits.yaml")
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

    # --- 1. Gazebo
    world_file = os.path.join(desc_share, "worlds", "library_world.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # --- 2. robot_state_publisher (gazebo_ros2_control also reads
    #        robot_description from this node, so keep its default name)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, USE_SIM_TIME],
    )

    # --- 3. Spawn into Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description", "-entity", "library_bot",
        ],
        output="screen",
    )

    # --- 4/5. Controller spawners (the controller_manager lives inside Gazebo)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[USE_SIM_TIME],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[USE_SIM_TIME],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        parameters=[USE_SIM_TIME],
        output="screen",
    )

    # --- 6. move_group
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
            USE_SIM_TIME,
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[gripper_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper_controller_spawner,
                    on_exit=[move_group_node],
                )
            ),
        ]
    )
