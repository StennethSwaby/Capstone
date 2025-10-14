cmake_minimum_required(VERSION 3.8)
project(slam_toolbox_bringup)

find_package(ament_cmake REQUIRED)

# Install your resource folders into the package share dir
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description)

# (optional if you have these)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds)
endif()

ament_package()
