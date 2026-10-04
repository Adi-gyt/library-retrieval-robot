source /opt/ros/humble/setup.bash
source ~/projects/library-retrieval-robot/ros2_ws/install/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=$HOME/projects/library-retrieval-robot/ros2_ws/src/library_bot_description/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/projects/library-retrieval-robot/ros2_ws/install/boeing_gazebo_model_attachment_plugin/lib:$GAZEBO_PLUGIN_PATH
