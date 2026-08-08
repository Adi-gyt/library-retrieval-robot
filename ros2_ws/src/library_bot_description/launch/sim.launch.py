import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('library_bot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'library_bot.urdf.xacro')

    # xacro.process_file() expands all the <xacro:...> shorthand (includes,
    # macros, ${} math) into plain URDF/XML that ROS2 tools can read.
    robot_description = xacro.process_file(xacro_file).toxml()

    # Starts Gazebo itself, loaded with an empty world by default.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    # robot_state_publisher reads the URDF and broadcasts the tf tree
    # (the coordinate-frame relationships between every link) so other
    # tools — rviz2, MoveIt2, tf2 lookups later — know where everything is.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # spawn_entity.py takes the robot_description topic and actually
    # places the robot into the running Gazebo world.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'library_bot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
