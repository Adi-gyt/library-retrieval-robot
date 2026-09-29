import os
import re
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("library_bot_moveit_config")

    urdf = xacro.process_file(
        os.path.join(share, "config", "library_bot_with_ros2_control.xacro"),
        mappings={"use_sim": "true"},
    ).toxml()
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.S)

    with open(os.path.join(share, "config", "library_bot.srdf")) as f:
        srdf = f.read()
    with open(os.path.join(share, "config", "kinematics.yaml")) as f:
        kinematics = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", os.path.join(share, "config", "moveit.rviz")],
            parameters=[
                {"robot_description": urdf},
                {"robot_description_semantic": srdf},
                kinematics,
                {"use_sim_time": True},
            ],
        )
    ])
