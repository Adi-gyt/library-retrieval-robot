"""
demo.launch.py

Same as move_group.launch.py, plus an OPTIONAL RViz with the MotionPlanning
plugin pre-loaded -- for when you have a real display (not the Pi5/SSH
session), e.g. to visually sanity-check the arm the way moveit_setup_assistant's
GUI would have let you do interactively.

Usage:
  ros2 launch library_bot_moveit_config demo.launch.py use_rviz:=true
  ros2 launch library_bot_moveit_config demo.launch.py            # no RViz, default
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz with the MotionPlanning plugin (needs a real display).",
    )

    moveit_config_share = get_package_share_directory("library_bot_moveit_config")

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group.launch.py")
        )
    )

    # Rebuild the same params RViz's MotionPlanning plugin needs. Uses the
    # same wrapper (real geometry + ros2_control tag) as move_group.launch.py
    # for consistency, even though RViz itself doesn't need the ros2_control
    # block.
    xacro_path = os.path.join(
        moveit_config_share, "config", "library_bot_with_ros2_control.xacro"
    )
    robot_description = {
        "robot_description": xacro.process_file(xacro_path).toxml()
    }
    srdf_path = os.path.join(moveit_config_share, "config", "library_bot.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}
    kinematics_yaml = load_yaml("library_bot_moveit_config", "config/kinematics.yaml")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            move_group_launch,
            rviz_node,
        ]
    )
