#!/bin/bash
# Terminal 2: wait for the sim, then set physics and put the boxes on table A.
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash ] && source ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash
source ~/gz_env.sh
echo "waiting for MoveIt and the gripper controller..."
until ros2 action list 2>/dev/null | grep -q "move_action"; do sleep 2; done
until ros2 action list 2>/dev/null | grep -q "gripper_controller/follow_joint_trajectory"; do sleep 2; done
sleep 5
for n in 1 2 3; do
  if [ -z "$(gz model -m aruco_box_$n -p 2>/dev/null)" ]; then
    echo "MISSING: aruco_box_$n is not in the world"; exit 1
  fi
done
gz physics -i 200
bash ~/projects/library-retrieval-robot/tools/reset_demo.sh
echo "READY. Run: cd ~/projects/library-retrieval-robot/ros2_ws/src/library_bot_description && ORDER=2,3 python3 scripts/step11_demo.py"
