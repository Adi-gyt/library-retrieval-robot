cmake_minimum_required(VERSION 3.8)
project(library_bot_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make the Python helper script executable/runnable via `ros2 run` style
# invocation (`python3 scripts/send_hardcoded_pose.py` also works directly).
install(PROGRAMS
  scripts/send_hardcoded_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
