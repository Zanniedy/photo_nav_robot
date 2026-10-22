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
find_package(std_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)

# Suppress conda rpath collision warning
set(CMAKE_SKIP_RPATH TRUE)

# libpointmatcher installs its headers under the ROS overlay; expose them globally
include_directories(${libpointmatcher_INCLUDE_DIRS})

# ament overlay headers (tf2_ros, sensor_msgs, etc.)
include_directories(
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

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

add_executable(pcd_map_server_node src/pcd_map/pcd_map_server_node.cpp)
target_include_directories(pcd_map_server_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd_map_server_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcd_map_server_node
  rclcpp sensor_msgs pcl_conversions
)

find_package(TBB REQUIRED)

add_executable(lio_slam_node src/lio/lio_slam_node.cpp)
target_include_directories(lio_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lio>
  ${EIGEN3_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)
target_link_libraries(lio_slam_node
  ${libpointmatcher_LIBRARIES}
  Eigen3::Eigen
  TBB::tbb
)
ament_target_dependencies(lio_slam_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2 tf2_ros
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
  rclcpp nav_msgs std_msgs geometry_msgs tf2 tf2_ros
)

add_executable(sim_imu_bridge_node src/sim_imu_bridge/sim_imu_bridge_node.cpp)
ament_target_dependencies(sim_imu_bridge_node
  rclcpp sensor_msgs std_msgs
)

add_executable(map_localizer_node src/map_localizer/map_localizer_node.cpp)
target_link_libraries(map_localizer_node Eigen3::Eigen)
ament_target_dependencies(map_localizer_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2 tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_matcher_2d test/test_scan_matcher_2d.cpp)
  target_link_libraries(test_scan_matcher_2d Eigen3::Eigen)
  ament_add_gtest(test_velocity_kf test/test_velocity_kf.cpp)
  target_link_libraries(test_velocity_kf Eigen3::Eigen)
  ament_add_gtest(test_lio_deskew test/test_lio_deskew.cpp)
  target_include_directories(test_lio_deskew PRIVATE ${libpointmatcher_INCLUDE_DIRS})
  target_link_libraries(test_lio_deskew ${libpointmatcher_LIBRARIES} Eigen3::Eigen TBB::tbb)
  ament_target_dependencies(test_lio_deskew sensor_msgs)
endif()

install(TARGETS pcd_map_node pcd_map_server_node lio_slam_node odom_path_node lio_ekf_node sim_imu_bridge_node map_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/relocalize_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
