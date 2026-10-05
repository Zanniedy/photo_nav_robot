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
find_package(libpointmatcher REQUIRED)

# Suppress conda rpath collision warning
set(CMAKE_SKIP_RPATH TRUE)

# libpointmatcher installs its headers under the ROS overlay; expose them globally
include_directories(${libpointmatcher_INCLUDE_DIRS})

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

add_executable(lio_slam_node src/lio/lio_slam_node.cpp)
target_include_directories(lio_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lio>
  ${EIGEN3_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)
target_link_libraries(lio_slam_node
  ${libpointmatcher_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(lio_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros
)

add_executable(odom_path_node src/odom_path/odom_path_node.cpp)
target_include_directories(odom_path_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(odom_path_node
  rclcpp nav_msgs geometry_msgs
)

add_executable(lio_ekf_node src/ekf_odom/lio_ekf_node.cpp)
target_include_directories(lio_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ekf_odom>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(lio_ekf_node Eigen3::Eigen)
ament_target_dependencies(lio_ekf_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS pcd_map_node lio_slam_node odom_path_node lio_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
