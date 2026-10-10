#!/bin/bash
# 在远程机器上编译并运行 serial_test
# 用法: ./build_and_run_test.sh [device] [baud]

DEVICE=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}
TEST_DIR=~/max_robot/ros_ws/src/test
BUILD_DIR=$TEST_DIR/build

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

echo "编译完成，运行测试: $DEVICE @ $BAUD"
./serial_test "$DEVICE" "$BAUD"
