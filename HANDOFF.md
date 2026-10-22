# 交接文档（2026-10-15）

平面移动机器人：2D 激光（思岚 C1）+ 底盘 MCU（串口上报 yaw），无轮速计。
链路：`robot_bridge` → `lio_slam_node`（帧间 ICP）→ `lio_ekf_node` → slam_toolbox 建图 / map_localizer + Nav2（MPPI）导航，外加网页控制台 **MAX_NAV_ROBOT**（`ros_ws/src/web`）。

- git 根：`/home/lsy/cs_project/robot`，远端 `github.com/Zanniedy/photo_nav_robot`（main）
- 实机：`sunrise@192.168.10.42`（ARM64），代码在 `~/max_robot/ros_ws`，地图在 `~/max_robot/maps`
- 同步：`bash rsync.sh`（本机执行）；远端编译：`bash make_remote.sh [--packages-select ...]`（机器人上执行）

## 2026-10-21：网页相机窗口 + 真机在线标定

目标：做相机真机标定（人员定位第 1 步）。机器人上没有显示器，所以把标定流程搬进网页，在相机浮窗里操作。

### 已完成

- 相机设备按 by-id 固定：`/dev/v4l/by-id/usb-AX_USB2.0_AX_USB2.0-video-index0`（index1 是元数据节点，打不开图像）。实测 MJPG 1280x720 约 26fps。串口原本就是 by-id，没改
- `person_locator/camera_node.py`：MJPG 原始码流直接转发，不解码
  - `/camera/image_raw/compressed`：全分辨率
  - `/camera/preview/compressed`：1/2 分辨率、10Hz，有订阅者才编码，给网页用
  - `/camera/camera_info`：`~/max_robot/calib/camera_info.yaml` 存在才发，文件变了自动重载
  - 读帧失败约 1s 会自动重开相机
- `person_locator/calib_node.py`：在线标定，复用 `calibrate.py` 的算法
  - `/calib/preview/compressed`：叠加角点和已覆盖区域
  - `/calib/status`：JSON，2Hz
  - 服务 `/calib/{capture,calibrate,save,reset}`（Trigger）和 `/calib/auto`（SetBool）
  - 标定在后台线程跑；保存时先写 tmp 再 rename，旧文件留 `.bak`，原图存到 `~/max_robot/calib/images/`
- `person_locator/config/camera.yaml`：两个节点的参数，棋盘 8x11 内角点、25mm
- `calibrate.py`：`check_pose_diversity` 改为返回 `(ok, msgs)`，命令行用法不变
- web 后端
  - `config.yaml` 新增进程 `camera`、`calib`，`camera` 加入 autostart
  - 自检里加了相机设备
  - `stack.py` 用 `STANDALONE` 统一管理可单独启动和重启的进程
- web 前端
  - 新增 `components/CameraWindow.vue`：可拖动、可缩放的浮窗，有「预览 / 标定」两种模式，标定模式下有采集、自动、标定、保存、清空按钮，结果显示 RMS、fx/fy/cx/cy、D、视场和警告
  - 顶栏加「相机」按钮；`lib/ros.js` 加 `callService`；状态页能重启 camera/calib
  - 已 `npm run build`
- 已 rsync 到机器人，`make_remote.sh --packages-select person_locator robot_interfaces` 编译通过
- 后端已在机器人上用 `setsid nohup bash ros_ws/src/web/start.sh > /tmp/max_ui.log` 起来，uvicorn 和 camera_node 进程都在

### 未验证 / 待做

1. ~~网页相机预览、标定、保存~~：用户已在实机上标完；结果已拷回 `person_locator/config/camera_info.yaml`，**但 fx=2770、RMS 1.07 可疑**，详见 `ros_ws/src/person_locator/MODEL_HANDOFF.md` 第四节
2. 之后新增：网页相机「拍照 / 连拍 / 下载 zip」，存到 `~/max_robot/captures/`（camera_node `/camera/snapshot` + 后端 `routers/captures.py`），实机测过
3. **模型部分（YOLO26-seg 转 X5 BPU）单独写在 `ros_ws/src/person_locator/MODEL_HANDOFF.md`**，接手模型先看那份。现状：第一版 `.bin` 已在 docker 容器 `rdk_x5_mapper` 里生成（`~/rdk_x5_mapper/yolo26/model_output/`），但校准用的是棋盘图，精度偏低，还没上板
5. calib_node 的 CPU 占用没测过（每帧都要全尺寸解码加找角点）。卡的话降低处理帧率，或者只在 1/2 图上检测
6. 相机外参还是 `robot.launch.py` 里的假值 `base_link→camera_link 0.25 0 0.6`，人员定位第 3 步前要实测（相机和雷达的相对 yaw 最关键）
7. 本轮和上一轮的改动都**还没 git commit**
8. 接下来：人员定位第 2 步，选 X5 BPU 上能跑的人脸或人体检测模型；第 3 步用相机算方位角，激光在这个角度窗口里取距离

### 远程操作的坑

- 本机 ssh/scp 会加载 miniconda 的 OpenSSL 报 version mismatch，要用 `env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin /usr/bin/ssh sunrise@192.168.10.42 ...`（rsync.sh 里已经这样处理了）
- ssh 里后台起服务要 `setsid nohup ... < /dev/null &`，否则 ssh 会一直挂着不退出

## 当前状态

| 项 | 状态 |
|---|---|
| Web UI 第一轮（登录、一键启动、默认地图、初始位姿、进程重启、ros2 命令） | 已在 GitHub（commit `待测`），已同步到机器人，**未在实机上跑过** |
| 建图保存失败修复 | 已改，本机未提交，未同步 |
| 导航左右摇摆修复（RotationShim） | 已改，本机未提交，未同步，**需实机验证效果** |
| 绘图标注 + 命名点位 + `/points/*` ROS 服务 | 已完成，本机测试通过，未提交，未同步 |

最后一次尝试同步时机器人 ping 不通、SSH 被断开，所以第二轮改动全部只在本机。

## 接手后第一件事

```bash
# 本机
cd /home/lsy/cs_project/robot
git status                          # 确认第二轮改动还在
git add ros_ws/src/web ros_ws/src/robot_interfaces ros_ws/src/robot_nav/config/nav2_params.yaml HANDOFF.md
git commit -m "web: 绘图标注与点位, 修复建图保存; nav2: RotationShim 包 MPPI"
git push
bash rsync.sh

# 机器人
cd ~/max_robot/ros_ws
bash make_remote.sh --packages-select robot_interfaces robot_nav
~/max_robot/ros_ws/src/web/start.sh          # 浏览器开 http://192.168.10.42:8000
```

登录：用户名 `max_robot`，密码默认也是 `max_robot`（`web/backend/config.yaml` 的 `auth.password`，部署前要改）。

## 实机待验证清单

1. **建图保存**：建图 → 保存，应提示成功，`~/max_robot/maps` 下要有 `.yaml/.pgm/.posegraph/.data`
2. **导航摇摆**：大角度转弯时应先原地转正再走，不再左右来回。不理想就调 `nav2_params.yaml` → `controller_server.FollowPath` 的 `angular_dist_threshold`（0.6）和 `rotate_to_heading_angular_vel`（0.8）
3. **点位**：网页上标几个点 → 🎯 导航；以及 `ros2 service call /points/goto robot_interfaces/srv/GotoPoint "{names: [点名]}"`
4. **rosbridge 只听本机**：`address:=127.0.0.1` 后浏览器经后端 `/ros` 能正常连上
5. **性能**：`/map`、`/scan` 走后端 WebSocket 代理在 ARM 上卡不卡
6. **自动初始位姿**：地图存了初始位姿后，进导航时 map_localizer 能收到（日志里 `ui` 进程会打印"初始位姿…已发送"）

## 这两轮改了什么

### 建图保存失败
`web/backend/routers/stack.py`：原来判断 `data=0`，SaveMap 实际返回 `result=0`，导致成功也报失败，posegraph 也没存。已改为 `result=0`，超时 20s → 45s，serialize_map 失败只写日志。

### 导航时"左转一下右转一下"
原因：大角度转向时 MPPI 采样少（600 条、单次迭代），叠加 bridge 转向死区（小角速度被抬到约 0.45rad/s）和 0.1~0.35s 指令滞后，wz 正负来回跳。
改法：`robot_nav/config/nav2_params.yaml` 中 `FollowPath` 改为 `RotationShimController`，`primary_controller` 为 MPPI，偏差超过 0.6rad 先以 0.8rad/s 原地转正。MPPI 参数未动。

### 绘图标注与点位
- 存储：`~/max_robot/annotations/<地图名>_<hash>.json`，每张地图一个文件，含 `shapes` 和 `points`。`web/backend/annot_store.py` 负责读写（fcntl 文件锁，网页和 ROS 节点共用）
- 形状：直线、箭头、折线、多边形、矩形、圆、手绘、文字；只做标注，**不影响导航**
- 前端：`lib/annot.js`（状态、自动保存、撤销）、`lib/draw.js`（渲染/命中测试）、`components/AnnotPanel.vue`（标注页）、`MapView.vue`（绘图/标点/选择/擦除工具）
- REST：`GET/POST /api/points`、`GET/PATCH/DELETE /api/points/{名字或id}`、`POST /api/points_goto {"names": [...]}`、`GET /api/annotations`、`PUT /api/annotations/shapes`
- ROS：新包 `ros_ws/src/robot_interfaces`（`NamedPoint.msg` + 5 个 srv），节点 `web/ros_nodes/points_server.py`，服务 `/points/{list,get,set,delete,goto}`。`map` 留空 = 当前导航地图。由后端 `autostart` 里的 `points` 进程拉起
- 详细用法见 `ros_ws/src/web/README.md`

## Web UI 结构速览

```
web/
  start.sh                 一键启动（只起后端，其余由后端按 config.yaml autostart 拉起）
  max_nav_robot.service    systemd 模板
  backend/
    config.yaml            端口、登录、地图目录、进程命令、autostart
    main.py                FastAPI 入口 + 登录中间件
    auth.py                HMAC cookie 登录（密钥每次启动随机）
    process_mgr.py         进程组管理、日志缓冲、一次性 ros2 命令
    maps.py / state.py     地图搜索、默认地图/初始位姿持久化
    annot_store.py         标注与点位存储
    routers/               stack(模式/地图/导航) annotations commands diag rosproxy(/ros 代理)
    static/                前端构建产物（npm run build 输出）
  frontend/src/            Vue 3 源码
  ros_nodes/points_server.py
```

改前端后：`cd ros_ws/src/web/frontend && npm install && npm run build`，再 rsync。

## 已知坑

- **本机 colcon 会被 miniconda 的 Python 3.14 劫持**，生成的 ROS 接口 import 报 `rosidl_typesupport_c` 错。编译接口包要用干净环境：
  `env -i HOME=$HOME PATH=/usr/bin:/bin bash -c 'source /opt/ros/humble/setup.bash && colcon build --packages-select robot_interfaces'`
- ssh 里 `pkill -f xxx` 会把 ssh 命令自己杀掉，用 PID 或脚本文件
- `timeout ros2 launch` 超时后子节点会变孤儿继续占串口（雷达报 `SL_RESULT_OPERATION_TIMEOUT`）。网页后端按进程组停止，不会有这个问题
- 串口用 by-id 路径，ttyUSB0/1 上电会对调
- 雷达 x 轴朝车体左侧，`base_link→laser` yaw=+90°；MCU `Wz>0` 是顺时针，bridge `cmd_wz_sign=-1`
- 安全：走明文 HTTP，只适合内网，不要把 8000 映射到公网；密码记得改

## 遗留 / 以后可做

- 雷达点云稀少时 ICP 退化、里程计漂移（老问题，未处理）
- 形状目前纯标注；以后要"禁行区"可以把多边形写进 costmap（keepout filter）
- 网页没有 HTTPS；机器人后端重启需重新登录
