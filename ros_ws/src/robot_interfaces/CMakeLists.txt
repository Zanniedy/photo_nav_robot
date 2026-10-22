cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NamedPoint.msg"
  "srv/ListPoints.srv"
  "srv/GetPoint.srv"
  "srv/SetPoint.srv"
  "srv/DeletePoint.srv"
  "srv/GotoPoint.srv"
  "action/ApproachPerson.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
