#!/bin/bash
# MAX_NAV_ROBOT 一键启动：只起后端；rosbridge / 底盘 / 默认地图导航由后端按 config.yaml 的 autostart 拉起
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
WS_SETUP="$DIR/../../install/setup.bash"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

export PATH="$HOME/.local/bin:$PATH"
PORT=$(python3 -c "import yaml; print(yaml.safe_load(open('$DIR/backend/config.yaml')).get('port', 8000))" 2>/dev/null || echo 8000)
cd "$DIR/backend"
exec python3 -m uvicorn main:app --host 0.0.0.0 --port "$PORT"
