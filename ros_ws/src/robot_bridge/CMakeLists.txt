cmake_minimum_required(VERSION 3.8)
project(robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SKIP_RPATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

add_executable(robot_bridge_node
  src/robot_bridge_node.cpp
)

target_include_directories(robot_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(robot_bridge_node
  ${Boost_LIBRARIES}
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(robot_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  ament_index_cpp
)

install(TARGETS robot_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
