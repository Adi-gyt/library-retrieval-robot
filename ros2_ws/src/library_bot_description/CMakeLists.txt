cmake_minimum_required(VERSION 3.8)
project(library_bot_description)

find_package(ament_cmake REQUIRED)

# This is the part that actually matters: it tells colcon (ROS2's build
# tool) to copy your urdf/ and launch/ folders into the installed package,
# which is why get_package_share_directory() can find them later.
install(DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
