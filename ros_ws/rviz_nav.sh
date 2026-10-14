#!/bin/bash
# 本机打开 nav2 rviz，连接实机（同网段 192.168.10.x，默认 ROS_DOMAIN_ID=0）
# 用法: bash rviz_nav.sh
# 用 env -i 剥掉 conda 环境，避免 libtinfo/OpenSSL 冲突

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CFG="$WORKSPACE_DIR/src/robot_nav/config/nav2.rviz"

exec env -i HOME="$HOME" DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" \
    WAYLAND_DISPLAY="$WAYLAND_DISPLAY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    PATH=/usr/bin:/bin \
    bash -c "source /opt/ros/humble/setup.bash && rviz2 -d '$RVIZ_CFG'"
