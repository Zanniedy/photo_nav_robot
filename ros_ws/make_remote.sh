#!/bin/bash
# 用法:
#   bash make_remote.sh          # 增量编译
#   bash make_remote.sh rebuild  # 清除 build/install/log 后全量编译

set -e
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

source /opt/ros/humble/setup.bash

_apt_install() {
    dpkg -s "$1" &>/dev/null || sudo apt-get install -y "$1"
}

_apt_install libeigen3-dev
_apt_install libyaml-cpp-dev
_apt_install libboost-system-dev
_apt_install libopencv-dev
_apt_install ros-humble-pcl-conversions
_apt_install ros-humble-slam-toolbox
_apt_install ros-humble-libpointmatcher

if [ "${1}" = "rebuild" ]; then
    echo "[rebuild] Cleaning build/ install/ log/ ..."
    rm -rf build install log
    echo "[rebuild] Resetting source file timestamps..."
    find src -type f | xargs touch
    shift
fi

echo "[build] Running colcon build..."
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
    --packages-ignore face_openvio_detector face_detector serial_test \
    "$@"

echo "[done] source install/setup.bash to use"
