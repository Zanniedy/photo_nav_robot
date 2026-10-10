cmake_minimum_required(VERSION 3.16)
project(face_detector CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- OpenVINO ---
find_package(OpenVINO REQUIRED
    PATHS /home/lsy/openvino/install/runtime/cmake
    NO_DEFAULT_PATH
)

# --- OpenCV (via ROS or system) ---
find_package(OpenCV REQUIRED)

# --- Library target ---
add_library(face_detector STATIC
    src/face_detector.cpp
)

target_include_directories(face_detector
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(face_detector
    PUBLIC
        openvino::runtime
        ${OpenCV_LIBS}
)

# --- Optional: small test executable ---
option(BUILD_DEMO "Build demo executable" OFF)
if(BUILD_DEMO)
    add_executable(face_demo demo/main.cpp)
    target_link_libraries(face_demo PRIVATE face_detector)
endif()
