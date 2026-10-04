#!/bin/bash
# Terminal 1: clean start of Gazebo + MoveIt. Leave this terminal open.
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group
pkill -9 -f robot_state_publisher; pkill -9 -f spawn_entity; pkill -9 -f spawner
sleep 2
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash ] && source ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash
source ~/gz_env.sh
ros2 launch library_bot_moveit_config gazebo_moveit.launch.py 2>&1 | tee /tmp/launch.log
