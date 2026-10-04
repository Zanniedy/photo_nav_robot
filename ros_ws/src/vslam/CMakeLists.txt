cmake_minimum_required(VERSION 3.8)
project(vslam)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

cmake_policy(SET CMP0077 NEW)
set(SPDLOG_BUILD_SHARED   OFF CACHE BOOL "" FORCE)
set(SPDLOG_FMT_EXTERNAL   OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL        OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/spdlog
  ${CMAKE_CURRENT_BINARY_DIR}/spdlog
  EXCLUDE_FROM_ALL
)

set(CMAKE_SKIP_RPATH TRUE)

# Interface library：让 clangd 通过 compile_commands.json 找到所有头文件
# 真正的可执行文件加进来后 target_link_libraries(<name> vslam_deps) 即可
add_library(vslam_deps INTERFACE)
target_include_directories(vslam_deps INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 占位编译单元，让 compile_commands.json 包含 vslam 的 include 路径供 clangd 使用
add_library(vslam_clangd_helper OBJECT src/.clangd_dummy.cpp)
target_include_directories(vslam_clangd_helper PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ament_target_dependencies 和 target_include_directories 模板，
# 添加可执行文件时照此格式补充即可：
#
# add_executable(<name> src/<subdir>/<file>.cpp)
# target_include_directories(<name> PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   ${OpenCV_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
# )
# target_link_libraries(<name>
#   ${OpenCV_LIBS}
#   Eigen3::Eigen
# )
# ament_target_dependencies(<name>
#   rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport
# )
# install(TARGETS <name> DESTINATION lib/${PROJECT_NAME})

find_package(yaml-cpp REQUIRED)

set(SPDLOG_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/spdlog/include)

add_executable(camera_open_test test/camera_open_test.cpp)
target_include_directories(camera_open_test BEFORE PRIVATE
  ${SPDLOG_INCLUDE}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(camera_open_test PRIVATE FMT_HEADER_ONLY)
target_link_libraries(camera_open_test
  ${OpenCV_LIBS}
  yaml-cpp
)

install(
  TARGETS camera_open_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_calibration test/camera_calibration.cpp)
target_include_directories(camera_calibration BEFORE PRIVATE
  ${SPDLOG_INCLUDE}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(camera_calibration PRIVATE FMT_HEADER_ONLY)
target_link_libraries(camera_calibration
  ${OpenCV_LIBS}
  yaml-cpp
)

install(
  TARGETS camera_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
