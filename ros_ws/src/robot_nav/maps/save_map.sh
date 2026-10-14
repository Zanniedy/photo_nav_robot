#!/bin/bash
# 保存当前 slam_toolbox 地图到 maps/ 目录
# 用法: bash save_map.sh [地图名]
# 默认名称: map_YYYYMMDD_HHMMSS

set -e

MAPS_DIR="$(cd "$(dirname "$0")" && pwd)"
MAP_NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "Saving map to: $MAP_PATH"

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_PATH'}}"

echo "Done. Files written:"
ls -lh "${MAP_PATH}"* 2>/dev/null || true
