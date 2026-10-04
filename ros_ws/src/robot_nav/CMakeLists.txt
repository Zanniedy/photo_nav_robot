cmake_minimum_required(VERSION 3.8)
project(robot_nav)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

# Suppress conda rpath collision warning
set(CMAKE_SKIP_RPATH TRUE)

add_executable(pcd_map_node src/pcd_map/pcd_map_node.cpp)
target_include_directories(pcd_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(pcd_map_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(pcd_map_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs pcl_conversions
)

add_executable(ekf_combined_node src/ekf_odom/ekf_combined_node.cpp)
target_include_directories(ekf_combined_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ekf_combined_node Eigen3::Eigen)
ament_target_dependencies(ekf_combined_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS pcd_map_node ekf_combined_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
