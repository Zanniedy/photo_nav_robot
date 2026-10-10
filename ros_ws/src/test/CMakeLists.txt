cmake_minimum_required(VERSION 3.10)
project(serial_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(serial_test serial_test.cpp)
target_link_libraries(serial_test Boost::system pthread)
