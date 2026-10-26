#!/bin/bash
# 机器人上干净重启（可选先编译）：停后端 → 停残留 ROS 进程 → 编译 → 清 FastDDS 共享内存 → 起后端
#   bash ~/max_robot/ros_ws/src/web/redeploy.sh                       # 只重启
#   bash ~/max_robot/ros_ws/src/web/redeploy.sh robot_nav robot_bridge  # 先编这几个包
# 从 ssh 调用时要后台跑，ssh 断了也不中断：
#   ssh sunrise@<ip> 'setsid nohup bash ~/max_robot/ros_ws/src/web/redeploy.sh robot_nav > ~/max_robot/redeploy.log 2>&1 < /dev/null &'
# 为什么要清共享内存：后端退不干净时残留的进程被强杀，/dev/shm/fastrtps_* 留下一堆坏段，
# 新起的节点之间发现不了（ros2 命令行卡住、/map 没有发布者），10-26 碰到过
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
ROOT="$(cd "$WS/.." && pwd)"
P="[r]os2 |[l]ib/robot_nav/|[l]ib/person_locator/|[l]ib/led_display/|[r]osbridge_websocket|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[l]ifecycle_manager|[m]ap_server|[s]llidar_node|[r]obot_bridge_node|[s]tatic_transform_publisher|[p]oints_server|[u]vicorn main:app|[_]ros2_daemon"
list() { ps -eo pid,args | grep -E "$P" | grep -v redeploy.sh | awk -v me=$$ '$1 != me {print $1}'; }
log() { echo "[$(date +%T)] $*"; }

log "停后端"
pkill -INT -f "[u]vicorn main:app"
for _ in $(seq 1 60); do pgrep -f "[u]vicorn main:app" >/dev/null || break; sleep 1; done
for sig in INT TERM KILL KILL; do
  pids=$(list); [ -z "$pids" ] && break
  log "残留 $(echo $pids | wc -w) 个进程，SIG$sig"; kill -$sig $pids 2>/dev/null; sleep 4
done
[ -n "$(list)" ] && log "还剩: $(list | xargs -r ps -o args= -p | cut -c1-60 | tr '\n' ';')"

if [ $# -gt 0 ]; then
  log "编译 $*（-j4，免得把板子压垮）"
  (cd "$WS" && MAKEFLAGS=-j4 bash make_remote.sh --packages-select "$@" --parallel-workers 1 2>&1 | grep -E "Summary|Failed|error")
fi

if [ -z "$(list)" ]; then
  log "清 FastDDS 共享内存（$(ls /dev/shm | grep -c fastrtps) 个）"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
fi

log "起后端"
cd "$ROOT" && setsid nohup bash "$HERE/start.sh" > /tmp/max_ui.log 2>&1 < /dev/null &
log "完成"
