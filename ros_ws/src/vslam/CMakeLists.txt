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
find_package(Ceres REQUIRED)

# Pangolin：直接 add_subdirectory 源码，避免 find_package 找不到
set(PANGOLIN_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/pangolin)
if(EXISTS ${PANGOLIN_SRC_DIR}/CMakeLists.txt)
  set(BUILD_TOOLS          OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES       OFF CACHE BOOL "" FORCE)
  set(BUILD_PANGOLIN_PYTHON OFF CACHE BOOL "" FORCE)
  # 强制静态链接，避免运行时找不到 libpango_*.so
  set(_bsl_save ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  add_subdirectory(${PANGOLIN_SRC_DIR}
                   ${CMAKE_CURRENT_BINARY_DIR}/pangolin
                   EXCLUDE_FROM_ALL)
  set(BUILD_SHARED_LIBS ${_bsl_save} CACHE BOOL "" FORCE)
  # Pangolin 源码有 anonymous struct，关掉其 targets 上的 pedantic
  foreach(_pt pango_core pango_opengl pango_display pango_windowing
              pango_vars pango_scene pango_glgeometry pango_video pango_plot)
    if(TARGET ${_pt})
      target_compile_options(${_pt} PRIVATE -Wno-pedantic -Wno-extra)
    endif()
  endforeach()
  set(HAVE_PANGOLIN ON)
  set(PANGOLIN_INCLUDE
    ${PANGOLIN_SRC_DIR}/components/pango_core/include
    ${PANGOLIN_SRC_DIR}/components/pango_opengl/include
    ${PANGOLIN_SRC_DIR}/components/pango_display/include
    ${PANGOLIN_SRC_DIR}/components/pango_windowing/include
    ${PANGOLIN_SRC_DIR}/components/pango_vars/include
    ${PANGOLIN_SRC_DIR}/components/pango_scene/include
    ${PANGOLIN_SRC_DIR}/components/pango_plot/include
    ${PANGOLIN_SRC_DIR}/components/pango_geometry/include
    ${PANGOLIN_SRC_DIR}/components/pango_image/include
    ${PANGOLIN_SRC_DIR}/components/pango_video/include
    ${PANGOLIN_SRC_DIR}/components/tinyobj/include
    ${CMAKE_CURRENT_BINARY_DIR}/pangolin/components/pango_core/include)
  message(STATUS "Pangolin: building from source")
else()
  set(HAVE_PANGOLIN OFF)
  message(STATUS "Pangolin source not found — trajectory viewer disabled")
endif()

set(SPDLOG_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/spdlog/include)

# 公共 include + link 宏，避免重复
macro(vslam_target name)
  target_include_directories(${name} BEFORE PRIVATE
    ${SPDLOG_INCLUDE}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
  )
  target_compile_definitions(${name} PRIVATE FMT_HEADER_ONLY)
  target_link_libraries(${name}
    ${OpenCV_LIBS}
    yaml-cpp
    ${CERES_LIBRARIES}
  )
  if(HAVE_PANGOLIN)
    target_include_directories(${name} BEFORE PRIVATE ${PANGOLIN_INCLUDE})
    target_link_libraries(${name} pango_display pango_opengl pango_windowing pango_core)
    target_compile_definitions(${name} PRIVATE HAVE_PANGOLIN)
    target_compile_options(${name} PRIVATE -Wno-pedantic -Wno-extra)
  endif()
endmacro()

add_executable(camera_open_test test/camera_open_test.cpp)
vslam_target(camera_open_test)

add_executable(camera_calibration test/camera_calibration.cpp)
vslam_target(camera_calibration)

install(
  TARGETS camera_open_test camera_calibration
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(frame_pipeline_test test/frame_pipeline_test.cpp)
vslam_target(frame_pipeline_test)

install(
  TARGETS camera_open_test camera_calibration frame_pipeline_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(orb_test test/orb_test.cpp)
vslam_target(orb_test)

install(
  TARGETS orb_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(vslam_test test/vslam_test.cpp)
vslam_target(vslam_test)

install(
  TARGETS vslam_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
