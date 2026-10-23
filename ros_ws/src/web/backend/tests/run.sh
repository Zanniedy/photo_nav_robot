#!/bin/bash
# 本机跑后端测试：干净环境（避开 conda）+ ROS（ros_link 测试要 rclpy）+ 仓库根 .venv_web
# 第一次用：/usr/bin/python3 -m venv --system-site-packages <仓库根>/.venv_web
#           <仓库根>/.venv_web/bin/pip install -r ../requirements.txt httpx==0.27.0 pytest==7.4.4
# 用法：bash tests/run.sh [pytest 参数，比如 -k runner]
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../../../../.." && pwd)"
exec env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c \
  "source /opt/ros/humble/setup.bash && cd '$DIR/..' && '$ROOT/.venv_web/bin/python' -m pytest tests -q $*"
