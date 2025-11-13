cmake_minimum_required(VERSION 3.8)
project(controlled_env)

# Build system only. Do NOT find_package teleop_twist_keyboard here.
find_package(ament_cmake REQUIRED)

# If later you add C++ nodes, you'll add their find_package + executables.

# Install your non-code assets (so launch/worlds/urdf show up in share/)
install(DIRECTORY
  launch
  worlds
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
