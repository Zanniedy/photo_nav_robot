cmake_minimum_required(VERSION 3.8)
project(photo_nav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scan_frame_fix.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
