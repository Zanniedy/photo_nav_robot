# MAX_NAV_ROBOT Web UI

浏览器版的 rviz2 + 进程管理：看机器人状态、建图、导航到点/航点、设初始位姿、遥控、跑 ros2 命令。
跑在机器人上，局域网任意机器打开 `http://<robot-ip>:8000`，用 `config.yaml` 里的账号登录。

## 部署（机器人上，一次）

```bash
sudo apt install ros-humble-rosbridge-server ros-humble-slam-toolbox ros-humble-nav2-bringup
pip install -r ~/max_robot/ros_ws/src/web/backend/requirements.txt
```

前端已构建到 `backend/static/`。改了前端后在开发机上 `cd frontend && npm install && npm run build`，再 rsync。

## 启动

```bash
~/max_robot/ros_ws/src/web/start.sh     # 一键：后端 → rosbridge → 底盘 → (默认地图)导航
```

开机自启：

```bash
sudo cp ~/max_robot/ros_ws/src/web/max_nav_robot.service /etc/systemd/system/max_nav_robot@.service
sudo systemctl daemon-reload
sudo systemctl enable --now max_nav_robot@$(whoami)
journalctl -u max_nav_robot@$(whoami) -f
```

## 配置 `backend/config.yaml`

| 字段 | 说明 |
|---|---|
| `auth.username / password` | 登录账号，**部署前改密码**；password 置空则关闭登录 |
| `map_dirs` | 地图搜索目录，递归 3 层找 map_server 格式 yaml；新建地图存到第一个 |
| `autostart` | 后端启动时拉起的进程，默认 `[rosbridge, base]` |
| `state_file` | 默认地图、各地图初始位姿，网页上改动写回这里 |
| `processes` | 各进程的启动命令 |

## 用法

- **建图**：顶栏「建图」→ 遥控走一圈 → 控制页填名字保存
- **导航**：顶栏选地图点「导航」，或控制页地图列表点「导航/切换」（导航中热切换，不重启 nav2）
- **默认地图 ★**：设了以后后端启动会自动进导航
- **初始位姿**：地图上「📍 初始位姿」拖拽，或控制页输入坐标发送；「保存为该地图初始位姿」后，下次进导航自动发送
- **导航目标**：「🎯 导航目标」点一下/拖朝向；或输坐标；多个航点用「➕ 航点」后执行
- **重启**：状态页每个进程有重启/停止按钮
- **命令**：常用检查一键执行；也可输入 `ros2 topic|node|service|param|action|interface|lifecycle|pkg|doctor ...`（不开放 run/launch）

视图：滚轮缩放，右键/中键拖动平移，空格急停，可勾选键盘 WASD 遥控。

## 安全

- 网页、REST、rosbridge 都要登录。rosbridge 只监听 127.0.0.1，浏览器经后端 `/ros` 代理访问
- 会话 cookie 签名密钥每次启动随机生成，后端重启需重新登录
- 走的是明文 HTTP，只适合在可信内网使用；不要把 8000 端口映射到公网

## 开发

```bash
cd frontend && npm install
ROBOT=192.168.10.42 npm run dev      # http://localhost:5173，/api 和 /ros 代理到机器人
```
