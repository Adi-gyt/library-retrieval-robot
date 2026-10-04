#!/bin/bash
# Run the demo after ready.sh prints READY. ORDER=2,3 by default.
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash ] && source ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash
source ~/gz_env.sh
cd ~/projects/library-retrieval-robot/ros2_ws/src/library_bot_description
ORDER=${ORDER:-2,3} python3 scripts/step11_demo.py 2>&1 | grep -E "===|marker at|outside|no stable|service|before release|detach|placed box|FINAL|ERROR|failed|not seen"
