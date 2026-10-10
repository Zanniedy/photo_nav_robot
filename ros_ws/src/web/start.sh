#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

# Source ros_ws so sllidar_ros2 is available
WS="$(dirname "$0")/../.."
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

export ROS_DOMAIN_ID=0

# Read serial port from config.yaml
SERIAL_PORT=$(python3 -c "
import yaml, os
cfg = yaml.safe_load(open(os.path.join('$(dirname "$0")', 'backend/config.yaml')))
print(cfg.get('lidar_serial_port', '/dev/ttyUSB0'))
" 2>/dev/null || echo "/dev/ttyUSB0")

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Start lidar if device exists
if [ -e "$SERIAL_PORT" ]; then
  echo "[start.sh] Starting sllidar_ros2 on $SERIAL_PORT"
  ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:="$SERIAL_PORT" &
else
  echo "[start.sh] Lidar device $SERIAL_PORT not found, skipping (use web UI to start manually)"
fi

sleep 2

# Start FastAPI
cd "$(dirname "$0")/backend"
exec uvicorn main:app --host 0.0.0.0 --port 8000
