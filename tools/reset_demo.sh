#!/bin/bash
[ -f ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash ] && source ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash
source ~/gz_env.sh
cd ~/projects/library-retrieval-robot/ros2_ws/src/library_bot_description
for n in 1 2 3; do
  ros2 service call /gazebo/detach boeing_gazebo_model_attachment_plugin_msgs/srv/Detach "{joint_name: grasp_fix, model_name_1: library_bot, model_name_2: aruco_box_$n}" > /dev/null 2>&1
done
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [finger_left_joint, finger_right_joint], points: [{positions: [0.065, 0.065], time_from_start: {sec: 3}}]}}" | tail -1
python3 scripts/retreat.py 2>&1 | tail -1
gz model -m aruco_box_1 -x 1.0 -y 0.20 -z 0.575 -R 0 -P 0 -Y 0; sleep 2
gz model -m aruco_box_2 -x 1.0 -y 0.0 -z 0.575 -R 0 -P 0 -Y 0; sleep 2
gz model -m aruco_box_3 -x 1.0 -y -0.20 -z 0.575 -R 0 -P 0 -Y 0; sleep 3
for n in 1 2 3; do echo "box $n: $(gz model -m aruco_box_$n -p)"; done
