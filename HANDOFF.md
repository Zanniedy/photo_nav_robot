# 交接文档（2026-10-22 晚更新）

平面移动机器人：2D 激光（思岚 C1）+ 底盘 MCU（串口上报 yaw），无轮速计。
链路：`robot_bridge` → `lio_slam_node`（帧间 ICP）→ `lio_ekf_node` → slam_toolbox 建图 / map_localizer + Nav2（MPPI）导航，外加网页控制台 **MAX_NAV_ROBOT**（`ros_ws/src/web`）。

- git 根：`/home/lsy/cs_project/robot`，远端 `github.com/Zanniedy/photo_nav_robot`（main）
- 实机：`sunrise@192.168.10.42`（有线）/ `192.168.1.124`（Wi-Fi），ARM64，代码在 `~/max_robot/ros_ws`，地图在 `~/max_robot/maps`
- 同步：`bash rsync.sh`（本机执行）；远端编译：`bash make_remote.sh [--packages-select ...]`（机器人上执行）

## 一览（10-22 晚）

| 模块 | 状态 | 提交 |
|---|---|---|
| 底盘控制闭环、速度估计、lio 去畸变、Nav2 调参、网页速度曲线等 | 已上机器人，大部分没实车验证 | `745b9f1` |
| BPU 检测模型 v2（真实照片校准，调试用） | 已上板，能用但慢（117ms/帧） | 模型文件不在 git 里 |
| 人员跟踪 + 走到人面前（`person_tracker`、`/person/approach`） | 已上机器人，节点能稳定跑，**没在有人的场景实测** | 核心代码在 `ce97711`；网页调试、`/person/state`、approach_client、单测、相机外参**未提交** |
| Win / 平板交互接口 | 只定了范围 | — |

接下来按顺序：
1. 提交未提交的改动（见「接手后第一件事」）
2. 实测人员跟随：网页状态页启动 `person`，控制页「人员跟随」点开始（详见 10-22 第二节「实测步骤」）
3. 跑 v3 模型转换（Softmax 放到 BPU），目标把单帧压到 50ms 以内
4. 到实际工作环境拍照，重新校准出正式版模型
5. 重标相机内参（fx=2770 可疑），标定相机和雷达的相对 yaw

## 2026-10-22：底盘控制闭环、速度估计、lio 去畸变、网页功能

> 本节改动已在 `745b9f1 等待模型ing` 提交，已 rsync 到机器人并编译、重启生效。

### 网络（先看这个）

- 机器人自带 Wi-Fi，已连路由器 `TP-LINK_BF09_5G`，**无线 IP 192.168.1.124**；有线 eth0 是 192.168.10.42
- 网页后端监听 0.0.0.0:8000，所以 Win 主机、平板连同一个 Wi-Fi 后直接开 `http://192.168.1.124:8000` 就行，**不用再拖网线**
- 拖着网线跑会被线拽住：10-22 实测同一指令速度忽快忽慢、后退测出正速度，都是线的问题
- 路由器 DHCP 最好给机器人绑定固定 IP；ssh/rsync 走无线时把 `rsync.sh` 和下面命令里的 IP 换成 192.168.1.124

### 改了什么

**底盘控制（`robot_bridge`）**
- `src/cmd_control.hpp`（新，纯函数，有单测）：
  - 前馈从直线改成实测 (u, 稳态速度) 查表：`cmd_wz_table_u/v`、`cmd_vx_table_u/v`。转向表是 10-14 阶跃实测点，前进表还是旧线性模型取的点，**待稳态实测替换**
  - 转向前馈分两种：原地转保留起步死区补偿；前进中（|vx|≥`cmd_moving_v_full`=0.1）不抬死区，小转向按比例输出，边走边转连续
  - 角速度 PI 闭环：陀螺 yaw_speed 反馈（扣 bias，轻 EMA），叠加在前馈上，`cmd_wz_closed_loop` 开关，陀螺 0.2s 无数据自动退回纯前馈
  - 线速度没有快反馈：用 `/lio_ekf_odom` 速度慢慢修正前馈增益 k，前进/后退各一个（底盘前后不对称：0.2m/s 指令前进 0.17、后退 0.23），k∈[0.8,1.3]；实测速度 < 指令×30% 视为失速（遥控接管/卡住），不调 k
  - 速度平滑改为 S 曲线（加速度 + 加加速度限制）：`cmd_vx_acc 0.4 / dec 0.6 / jerk 1.5`，`cmd_wz_acc 1.2 / dec 1.8 / jerk 5`
- 所有控制参数可 `ros2 param set /robot_bridge_node ...` 在线改
- 调参话题 `/robot/cmd_debug`（Float32MultiArray，50Hz，有订阅才发）：`[target_vx, cur_vx, meas_vx, u_vx, k_vx, target_wz, cur_wz, meas_wz, u_wz, i_wz]`

**速度估计（`lio_ekf_node`）**
- **之前 `/lio_ekf_odom` 的 twist 全是 0**，而 Nav2 controller 的 odom 就是它，MPPI 一直以为车是静止的
- 新增 `VelocityKf`（`lio_ekf.hpp`）：odom 系匀速 KF，只吃 lio 位置，dt 用扫描时间戳；输出车体系 twist.linear，twist.angular.z 用陀螺
- `q_vel_acc=0.05`：仿真里速度噪声 0.04m/s、阶跃滞后约 0.2s；稳态速度标准差约 0.1（bridge 用 0.15 做门限）
- lio 被拒帧（只转 yaw、平移没测）用 `pose.covariance[0]=1e6` 标记，速度估计跳过

**lio 高速跟踪（`lio_slam_node`）**
- 扫描去畸变 `Deskew`（`lio_math.hpp`）：雷达一圈 0.1s，1rad/s 时一帧首尾差 5.7°，4m 处错开 0.4m。按点的采集时刻用匀速模型补偿。合成测试（0.35m/s + 1rad/s）：去畸变前 ICP 误差 3cm/2.4°，去畸变后≈0
- **`deskew_reversed: true` 是按 sllidar 驱动反转数组推断的，没实测**。原地快转时墙反而更糊就改 false 对比
- 运动中（>0.05m/s）预测加匀速平移；静止不加（会把 ICP 噪声正反馈成漂移）；被拒帧速度每帧衰减 0.7
- 关键帧点集只算一次缓存

**Nav2（`nav2_params.yaml`）**
- 慢但顺：`vx_max 0.25`、`wz_max 0.8`，`ax_max/ax_min/az_max` 与 bridge 一致
- RotationShim `angular_dist_threshold` 0.6→1.0rad：只有大偏差才原地转，其余边走边转；原地转速 0.6
- MPPI 采样噪声 `vx_std 0.15 / wz_std 0.3`，加 `TwirlingCritic`（weight 10）抑制左右摆

**网页**
- 顶栏「速度」：目标/实际速度曲线（线速度、角速度两张图，蓝=目标、橙=实际），时间窗 10/30/60s、暂停、导出 CSV
- 相机和速度曲线改为地图与右侧面板之间的停靠栏，左边缘拖宽度
- 地图工具栏加 ⟲ / ⟳ 旋转 90°、点角度复位、Alt+滚轮 5° 细调；只改显示，发出的坐标仍是 map 系
- 控制页「导航」加点名搜索框：回车（同名或唯一匹配）或点列表项直接导航
- **标注/点位改存地图旁 `<地图名>.annot.json`**，地图目录拷走标注跟着走；旧 `~/max_robot/annotations/` 文件仍能读，下次保存写到新位置；地图目录不可写时退回旧目录

### 测试情况

- 本机单测：`robot_bridge/test_cmd_control` 8 个、`robot_nav` 的 `test_velocity_kf` / `test_lio_deskew` / `test_scan_matcher_2d` 共 10 个，全过
- 实车阶跃（脚本在机器人 `/tmp/motion_test.py`，重启会丢）：只有第一轮可信（之后网线缠上）：转向 0.3/0.6/1.0 开环 0.39/0.59/1.05，闭环 0.32/0.59/1.02
- **没实车验证的**：S 曲线平滑和边走边转的手感、去畸变方向、新 Nav2 参数、点名导航、标注存到地图旁
- 低速原地转（0.3rad/s）贴着死区，有黏滑，闭环追不满，可能要加大 `cmd_wz_ki` 或把原地转最低速度抬到能稳定转的值

### 已知问题

- controller_server 一直报 `Extrapolation Error ... map → base_footprint`（TF 比请求时刻晚约 57ms），lio TF 用的是扫描时间戳。`transform_tolerance` 0.5 兜住了，没影响控制，以后可以处理
- 遥控器优先级最高，没关时 MCU 不执行 PC 指令：表现为 bridge 日志里指令正常、实测速度 0

### 下一步：Win 主机 / 平板交互接口（只定了范围，还没写）

已定：
- 传输：复用机器人 FastAPI 后端，加一组 `/api/v1` 接口。HTTP 请求/应答，WebSocket 推送状态和导航进度。网页、Win、平板共用
- 网络：机器人 Wi-Fi，Win 和平板连同一路由器
- 第一版功能：
  1. 点位查询 + 按名导航：列出/搜索点位；单点或多点依次导航；返回结果（到达/失败/取消）和进度（剩余距离、预计时间）
  2. 状态与控制：位姿、模式和地图、是否在导航、定位是否可靠（`/localizer/score`）；取消导航、急停
  3. 点位管理：新增/修改/删除，含"把当前位置存成点"
  4. 人员定位结果：先定格式（人的方位角、距离、置信度、时间戳），等 person_locator 第 3 步做完再接数据

还要定：
- 认证：网页现在是 cookie 登录；给 Win/平板建议加固定 API token（请求头带，配在 `config.yaml`），内网够用
- Win 端用什么语言、要不要写客户端示例（用户说先不写）
- 已有可复用的：`GET/POST/PATCH/DELETE /api/points`、`POST /api/points_goto`、`GET /api/status`、`POST /api/nav/cancel`；ROS 侧 `/points/{list,get,set,delete,goto}`。缺导航结果/进度回传（现在 points_goto 发出就返回）、急停、统一的错误码

### 再往后：上云（只是方案，没写代码）

需求：云端管理地图数据，下发 xy 目标点或控制指令。

**连接：机器人主动连云。** 机器人在路由器 NAT 后面，云连不进来。机器人上起一个常驻 `cloud_agent` 进程，开机主动连云并保持长连接。推荐 MQTT over TLS（每台机器人一套证书或密钥），可以用阿里云 IoT / 华为 IoTDA / AWS IoT，也可以自建 EMQX / Mosquitto：断线重连、QoS1 不丢、遗嘱消息报掉线，4G 弱网也能用。如果云端是自己写的后端，也可以改成机器人主动连 WebSocket，道理一样。

**指令和数据分开走：**

| 内容 | 通道 | 要点 |
|---|---|---|
| 目标点、按点名导航、取消、暂停 | MQTT 下行 | 带 `cmd_id`（去重）和有效期（过期不执行）；机器人先回 ack，导航结束再回结果 |
| 位姿、状态、导航进度、电量 | MQTT 上行 | 约 1Hz；不要把 `/scan`、`/tf` 推上云 |
| 地图（pgm / yaml / posegraph / annot.json） | HTTPS 对象存储，预签名 URL | 几 MB 不走 MQTT；MQTT 里只发"有新地图 + 版本 + 下载地址" |
| 遥控 | 尽量不走云 | 延迟 100~500ms 还会抖。必须做的话，保留 bridge 的 0.5s cmd_vel 超时停车 |

**坐标要和地图版本绑定（最容易踩的坑）。** xy 只在某一张地图里有意义，重新建图后原点和朝向都会变。
- 每张地图有 `map_id` + 版本（文件哈希）
- 下发 xy 必须带 `map_id@version`，和机器人当前地图对不上就拒绝并报错
- 优先按点名下发（"去 A 点"），由机器人本地查坐标。点位已经和地图存在一起（`<地图名>.annot.json`），正好配合

**安全判断留在本地，云只能"请求"：**
- 本地急停和遥控器优先级永远最高
- 定位不可靠（`/localizer/score` 低）时拒绝执行导航
- 云下发的速度上限只能比本地配置低
- 断网时，正在执行的任务按配置继续或原地停；恢复后补报结果

**和 Win / 平板共用一层。** `cloud_agent` 不直接操作 ROS，只调用本地 `/api/v1`。网页、Win、平板、云四个入口用同一套指令和状态逻辑。所以顺序是：**先做本地 `/api/v1`，云只是多一个 MQTT↔HTTP 转发器**。

开发阶段要从外网调试，可以先用 Tailscale / ZeroTier 组网直接访问网页和 ssh，只用于调试，不要当正式方案。

还要用户确认：
1. 云平台自建，还是用阿里云 / 华为云这类现成物联网平台
2. 机器人数量（一台的话云端可以简单很多）
3. 云上要不要实时视频 / 远程遥控（要的话得另走 WebRTC，工作量大不少）

## 2026-10-22：BPU 模型 v2 + 人员跟踪 / 走到人面前

> 核心代码（detector/geometry/tracker/person_tracker_node、ApproachPerson.action）在 `ce97711 人脸识别`；之后加的网页调试、`/person/state`、approach_client、单测、相机外参**还没提交**。都已 rsync 到机器人，`robot_interfaces`、`person_locator` 已编译。模型细节见 `ros_ws/src/person_locator/MODEL_HANDOFF.md`

### 模型（调试版）

- 用网页连拍的 122 张照片重新校准，出了 v2：`~/rdk_x5_mapper/yolo26/model_output/`，第一版（棋盘图校准）留在 `model_output_v1_chessboard/`
- 板上对比浮点 ONNX（122 张，score≥0.35）：

  | | 召回 | 多检 | 框中心水平偏差 均值/P95 |
  |---|---|---|---|
  | v1 棋盘图 | 95/97 | 132 | 2.4 / 6.1px |
  | v2 真实照片 | 92/97 | 1 | 3.4 / 8.6px |

  v2 漏的都是原模型就只有 0.4~0.55 分的糊图、画面边缘的人。**这一版只用来调通流程，正式版要在实际工作环境拍照重新校准**（照片只有一个人、一种光照）
- 已装到机器人：`~/max_robot/models/yolo26s_seg_bpu.bin` + `yolo26s_seg_bpu_outputs.json`（v1/v2 对比用的副本在 `models/v1`、`models/v2`）
- 速度（`hrt_model_exec perf` profile）：整模型 117ms，其中 BPU 只有 24ms；两个注意力层的 Softmax 在 CPU 上算（15+15ms），前后量化/反量化约 43ms，子图间搬数据约 13ms
- v3：`~/rdk_x5_mapper/yolo26/config_v3.yaml` 用 `node_info` 把两个 Softmax 放到 BPU（int16），输出到 `model_output_v3/`。`node_info` 放在 `calibration_parameters` 下报 `Wrong key`，已挪到 `model_parameters` 下，**还没跑**。命令：
  ```bash
  sudo docker run --rm -v ~/rdk_x5_mapper:/workspace openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8-py310 bash -c \
    'cd /workspace/yolo26 && hb_mapper makertbin --model-type onnx --config config_v3.yaml > /dev/null 2>&1; grep -iE "Wrong key|SchemaError|Softmax" hb_mapper_makertbin.log | tail -4; tail -1 hb_mapper_makertbin.log' \
    && sudo chown -R $USER:$USER ~/rdk_x5_mapper/yolo26
  ```
  还报错就去容器里 horizon_nn 源码 grep `node_info` 看 schema 写法
- 原来的 `rdk_x5_mapper` 容器已经没了，现在都用 `docker run --rm` 临时起（镜像还在）

### 人员跟踪节点 `person_tracker`

`ros2 run person_locator person_tracker --ros-args --params-file .../config/person_tracker.yaml`，网页后端进程名 `person`（不在 autostart，状态页手动启动）。要 camera、base、导航都在跑（跟踪系是 `map`）

- 定位：检测框左右边缘去畸变 → 方位扇形（框宽 ×0.6）→ 激光扇形内距离取 20% 分位 → 所在激光簇的质心往后推 0.12m（激光只打到前表面）→ 人在 map 下的位置
- 跟踪（`tracker.py`）：匀速卡尔曼。轨迹只由相机检测创建；之后激光簇（宽 0.1~1.0m）也能更新，所以人走远检测不到时还能跟。丢失条件：2s 没任何更新；或者人在相机视野内 5m 以内，相机却 3s 没看到（防止轨迹漂到柱子上）
- 锁定：开始时锁离机器人最近的人，中途有更近的人也不换；目标丢了再锁当前最近的
- 相机外参：相机在雷达正上方 0.72m，`base_link→camera_link` 改为 `0.25 0 0.62`（robot/rviz launch）。**相机和雷达的相对 yaw 默认 0，没标定过**
- 检测只取 person 通道、不算 mask（`detector.py`），1280x720 的 jpeg 按 1/2 直接解码成 640x360，省掉缩放。板上和批量测试结果一致（21 张框数全对，偏差中位 6.6px）
- 实测板子负载很高（load 16，导航全开），检测约 180ms/帧、4Hz

### 走到人面前 `/person/approach`

- action `robot_interfaces/action/ApproachPerson`：goal `distance`（默认 3m）、`timeout`、`search_timeout`（默认 5s 内要看到人）；feedback `state`（searching / approaching / arrived / lost）、目标 id、人的位置和距离
- 目标点在「机器人→人」连线上离人 distance 处，车头朝人；已经在 distance 以内就原地转向人，不后退
- 参数 `face_front`（运行中可改，每次开始时读）：
  - `false`：导航中人移动超过 0.5m 就更新目标点
  - `true`：锁定后用前 3 次相机观测平均出人的位置并**记住**，直到这次导航结束都按记住的位置走。人的朝向现在假设为面朝机器人（只有人体框，没有人脸/朝向）；以后换人脸或 pose 模型只改 `_facing()`
- 调用方式：
  - 命令行：`ros2 run person_locator approach_person --distance 3`
  - Python：`from person_locator.approach_client import approach_person; ok, msg = approach_person(node, 3.0)`（node 要在 executor 里 spin）
  - ROS：`ros2 action send_goal /person/approach robot_interfaces/action/ApproachPerson "{distance: 3.0}" --feedback`
  - 网页调试用服务：`/person/approach_start`、`/person/approach_cancel`（std_srvs/Trigger，按当前参数）
- 中途目标丢了会继续开往最后的目标点，结果里注明

### 网页调试

- 控制页「人员跟随」：face_front 开关、停在离人多远、开始/取消、状态、记住的位置、检测频率/耗时/测到距离的框数、轨迹列表
- 地图图层「人员跟踪」：橙圈=轨迹、绿圈=锁定的目标、**红点=记住的位置**（导航结束后变淡保留，方便对比）、粉箭头=导航目标
- 数据来源 `/person/state`（std_msgs/String JSON，5Hz）；参数通过 `/person_tracker/set_parameters` 改。RViz 可看 `/person/markers`

### 实测步骤（还没做过）

1. 网页状态页启动 `person`，控制页「人员跟随」显示「在线」
2. 人站在机器人前 2~4m：轨迹列表应出现一个人，「测到距离」不为 0，地图上橙圈和激光点里的人重合
3. 「测到距离」一直是 0：多半是方位不对，激光扇形没扫到人。先查相机内参（fx=2770 对应只有 26° 视场，不像真的）和相机相对雷达的 yaw
4. 橙圈和激光里的人左右错开：调 `robot.launch.py` 里相机的 yaw
5. 点「开始」，看能不能停在人前 3m、车头朝人；再分别试 face_front 开/关、人走动、人走出画面

### 测试

- 纯逻辑单测 23 个（几何、跟踪、解码），全过：`cd ros_ws/src/person_locator && PYTHONPATH=. /usr/bin/python3 -m pytest test/`（用系统 python，不要用 conda 的）
- 本机和机器人都编译通过，节点在机器人上跑了几分钟没出错（期间修了 Marker 字段类型导致的崩溃）

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
3. **模型部分（YOLO26-seg 转 X5 BPU）单独写在 `ros_ws/src/person_locator/MODEL_HANDOFF.md`**，接手模型先看那份。~~第一版用棋盘图校准~~：10-22 已用真实照片出了 v2 并上板，见上面 10-22 第二节
5. calib_node 的 CPU 占用没测过（每帧都要全尺寸解码加找角点）。卡的话降低处理帧率，或者只在 1/2 图上检测
6. ~~相机外参是假值~~：10-22 按实测改为雷达正上方 0.72m（`0.25 0 0.62`）；**相对 yaw 还没标**
7. ~~本轮和上一轮的改动都还没 git commit~~：已在 `30915ed camera` 提交
8. ~~人员定位第 2、3 步~~：10-22 完成第一版（YOLO26s-seg 人体检测 + 激光取距离 + 跟踪 + 走到人面前），见上面

### 远程操作的坑

- 本机 ssh/scp 会加载 miniconda 的 OpenSSL 报 version mismatch，要用 `env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin /usr/bin/ssh sunrise@192.168.10.42 ...`（rsync.sh 里已经这样处理了）
- ssh 里后台起服务要 `setsid nohup ... < /dev/null &`，否则 ssh 会一直挂着不退出

## 接手后第一件事

```bash
# 本机：提交 10-22 人员跟踪的剩余改动（核心代码已在 ce97711）
cd /home/lsy/cs_project/robot
git status
git add HANDOFF.md ros_ws/src/person_locator ros_ws/src/robot_nav/launch \
        ros_ws/src/web/backend/config.yaml ros_ws/src/web/backend/routers/stack.py ros_ws/src/web/backend/static \
        ros_ws/src/web/frontend/src
git commit -m "person: 网页调试面板与红点标记, approach_client, 单测; 相机外参 0.72m"
git push
# ros_ws/src/3rdparty/ 下的 libpointmatcher、pangolin、spdlog 不是这次的改动，别一起加

# 改了代码再同步、编译（本机执行 rsync，机器人上编译）
bash rsync.sh
# 机器人：cd ~/max_robot/ros_ws && bash make_remote.sh --packages-select robot_interfaces person_locator
```

后端已在机器人上跑着（`http://192.168.1.124:8000` 或 `192.168.10.42:8000`）。改了 `config.yaml` 要重启后端才能看到新进程 `person`：
`setsid nohup bash ~/max_robot/ros_ws/src/web/start.sh > /tmp/max_ui.log 2>&1 < /dev/null &`（先按 PID 停掉旧的 uvicorn）

登录：用户名 `max_robot`，密码默认也是 `max_robot`（`web/backend/config.yaml` 的 `auth.password`，部署前要改）。

## 实机待验证清单（10-15 那批）

1. ~~建图保存~~、~~点位~~、~~rosbridge 只听本机~~、~~自动初始位姿~~：后来实际用过，没再报问题
2. **导航摇摆**：参数 10-22 又改过（RotationShim 阈值 1.0rad、TwirlingCritic），见 10-22 第一节
3. **性能**：`/map`、`/scan` 走后端 WebSocket 代理在 ARM 上卡不卡，没专门测过

## 10-15 那两轮改了什么（存档）

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
  frontend/src/            Vue 3 源码（lib/person.js + components/PersonPanel.vue：人员跟随调试）
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
- 安全：走明文 HTTP，只适合内网，不要把 8000 映射到公网；密码记得改。`/person/approach_start` 也走这个代理，能让机器人动，同样只能内网用
- 本机跑 Python 单测用 `/usr/bin/python3`：conda 环境里 cv2 和 numpy 2 不兼容
- 机器人上 `pkill -f person_tracker` 会把 ssh 自己杀掉，用 `pgrep -f lib/person_locator/person_tracker` 拿 PID 再 kill

## 遗留 / 以后可做

- 雷达点云稀少时 ICP 退化、里程计漂移（老问题，未处理）
- 形状目前纯标注；以后要"禁行区"可以把多边形写进 costmap（keepout filter）
- 网页没有 HTTPS；机器人后端重启需重新登录
