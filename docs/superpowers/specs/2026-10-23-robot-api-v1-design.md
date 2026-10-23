# 机器人本地 /api/v1 与任务层 设计（2026-10-23）

上云的第 1 块。目标：机器人有自己的身份（robot_id），对外提供一套统一接口（网页、平板、cloud_agent、语音团队共用），能下发 goto / patrol / guide 任务并拿到进度和结果。云端（ECS + EMQX + cloud_agent）是第 2 块，另写 spec。

## 1. 机器人身份

- 每台机器人一份 `~/max_robot/robot.yaml`，**不放在仓库里**：`rsync.sh` 用 `--delete` 同步 `ros_ws/src`，仓库里的 `config.yaml` 每次都会被覆盖，放进去的 id 会丢
  ```yaml
  robot_id: "max-001"        # 全局唯一，只用 [a-z0-9-]，上云后 MQTT 账号、topic 都用它，定下来不要再改
  name: "一号机"              # 显示名，可以随时改
  api_tokens:                # 给平板 / 上层调用方，每个带名字，日志里记是谁调的
    - {name: "tablet", token: "<随机 32 字节>"}
  ```
- 文件不存在：后端照常启动，`robot_id` 用 `unset-<hostname>`，`/api/v1/robot` 和状态里带 `configured: false`，网页状态页「机器人身份」显示未配置
- 配置方式：网页状态页填 id 和名字（只在 `configured: false` 时允许改 id）；或 ssh 上去手写文件。token 由后端生成，只在生成时显示一次
- 后续按 robot_id 区分的配置（云端地址、MQTT 账号）也写在这份文件里

## 2. 结构

```
网页 / 平板 / cloud_agent / 语音团队
            │  HTTP + WebSocket
     backend /api/v1          token 或 cookie 认证，统一错误码
            │
     task_runner.py           纯 Python 任务状态机，不依赖 ROS，可单测
            │
     ros_link.py              后端进程内 rclpy 节点（后台线程）：
            │                 NavigateToPose action 客户端、订阅位姿 / /localizer/score / /person/state、发 /robot/estop
          Nav2
```

- 现在 `stack.py` 用子进程 `ros2 action send_goal` 发导航，拿不到结果，由 `ros_link` 取代
- 后端重启时正在跑的任务中断（导航进程本来就归后端管，可接受）
- 旧接口（`/api/points_goto`、`/api/nav/waypoints`、`/api/nav/cancel`）保留，内部改调 task_runner

## 3. 任务

同一时间只有一个「前台任务」；巡航可以被挂起，在后台等着。

| 类型 | 参数 | 说明 |
|---|---|---|
| `goto` | `points`（一个或多个点 id / 名字） | 依次走到每个点 |
| `guide` | `point`，`max_speed`（默认 0.15 m/s） | 带路：限速的单点 goto，不接人员跟踪 |
| `patrol` | `route`（路线 id），`loop`（默认按路线设置） | 按路线巡航 |

- 状态：`pending → running → succeeded / failed / canceled / preempted`；巡航多一个 `waiting`
- 进度：当前第几个点、剩余距离、预计时间（剩余距离 / 限速，粗估）
- 新的 goto / guide：巡航在跑或在等 → 巡航挂起成 `waiting`，新任务执行，结束后巡航仍在 `waiting`；没有巡航 → 顶掉当前 goto / guide（记 `preempted`）
- 新的 patrol 顶掉旧的巡航和它的子任务
- 限速实现：优先用 controller_server 自带的限速话题（`nav2_msgs/SpeedLimit`，默认话题 `speed_limit`），任务开始发限速、结束发 0（取消限速）；实现时先在机器人上确认 Humble 的 MPPI 认这个话题，不认再退回 `ros2 param set` 改 `vx_max`

### 巡航的 waiting

```
moving ──发现人 / 上层调 wait──▶ waiting ──上层调 resume / 超时──▶ moving（从当前位置去下一个没到的点）
                                  └─ 期间可以下发 goto / guide 子任务，做完仍停在 waiting
```

- 自动进入：订阅 `/person/state`，巡航中 `patrol.person_distance`（默认 3m）内出现人 → 取消当前导航目标、进入 waiting、推送 `person_found`。开关 `patrol.wait_on_person`，**默认关**（人员检测还没实测）
- 手动进入：`POST /api/v1/tasks/{id}/wait`
- 超时 `patrol.wait_timeout`（默认 120s，0 = 一直等）后自动 resume，推送 `wait_timeout`
- resume 后 `patrol.person_cooldown`（默认 30s）内不再因为发现人自动停
- 取消巡航连同子任务一起结束；取消子任务巡航仍在 waiting

### 安全检查（下发和 resume 前；导航中途停了由 tick 结束任务，急停直接取消任务）

- 导航没启动 → `NAV_NOT_RUNNING`；建图模式 → `BUSY`
- 定位分数 < `localization.min_score` → `LOCALIZATION_UNRELIABLE`。**阈值上车看过正常值后再定**，先默认 0（不检查）
- 请求带 `map_id` 且和当前地图不同 → `MAP_MISMATCH`
- 急停中 → `ESTOPPED`

## 4. 地图标识

- `map_id` = 地图名（yaml 文件名去扩展名，同名冲突时报错让用户改名）；`version` = yaml + pgm 内容的 sha1 前 12 位
- 内部仍用 yaml 绝对路径找文件，对外只出 `map_id` / `version`
- 巡航路线存在 `<地图名>.annot.json` 新增的 `routes` 字段：`[{id, name, points: [点 id...], loop}]`；点被删时路线里的这一项一起删

## 5. 接口

- 成功 `{"ok": true, "data": ...}`；失败 `{"ok": false, "code": "...", "msg": "..."}`，HTTP 状态码照常，调用方以 `code` 为准
- 认证：`Authorization: Bearer <token>`（`robot.yaml` 的 `api_tokens`）或网页 cookie
- 所有返回和事件都带 `robot_id`

| 接口 | 用途 |
|---|---|
| `GET /api/v1/robot` | `robot_id`、`name`、`configured`、接口版本 `api_version`（机器人上是 rsync 过去的代码，没有 .git） |
| `PUT /api/v1/robot` | 改 `name`；`configured: false` 时可设 `robot_id` |
| `POST /api/v1/robot/tokens`、`DELETE /api/v1/robot/tokens/{name}` | 生成 / 删除调用 token |
| `GET /api/v1/status` | 位姿、模式、当前地图 `map_id`/`version`、定位分数和是否可靠、前台任务和巡航摘要、急停 |
| `GET /api/v1/maps` | 地图列表 |
| `GET /api/v1/points?q=` | 当前地图点位，可搜索 |
| `POST /api/v1/points` | 新增；`from_current_pose: true` 时用当前位姿 |
| `PATCH/DELETE /api/v1/points/{id或名字}` | 改 / 删 |
| `GET/POST /api/v1/routes`、`PUT/DELETE /api/v1/routes/{id}` | 巡航路线 |
| `POST /api/v1/tasks` | 下发任务，返回 `task_id` |
| `POST /api/v1/guide` | 带路快捷接口：`{"point": "A会议室", "max_speed": 0.15, "map_id": 可选}`，等价于 `tasks` 的 guide |
| `GET /api/v1/tasks/current`、`GET /api/v1/tasks/{id}` | 状态和进度；只保留最近 50 个 |
| `POST /api/v1/tasks/{id}/cancel`、`/wait`、`/resume` | 取消；巡航等待 / 恢复 |
| `POST /api/v1/estop`、`/estop/release` | 软件急停 / 解除 |
| `WS /api/v1/events` | 1Hz 状态 + 任务事件（`task_state`、`progress`、`person_found`、`waiting`、`resumed`、`wait_timeout`、`estop`） |

错误码：`UNAUTHORIZED`、`BAD_REQUEST`、`NOT_FOUND`、`AMBIGUOUS`（名字匹配到多个点）、`NAV_NOT_RUNNING`、`LOCALIZATION_UNRELIABLE`、`MAP_MISMATCH`、`ESTOPPED`、`BUSY`、`NAV_FAILED`、`ROBOT_NOT_CONFIGURED`（`robot_id` 已设后再改）、`INTERNAL`

上云后云端用同一套格式，路径前加 `/api/v1/robots/{robot_id}`。

## 6. 急停

- `robot_bridge` 订阅 `/robot/estop`（std_msgs/Bool，transient local）。为 true 时不管 cmd_vel 输出 0，速度平滑状态清零（解除后从 0 起步）
- 判断逻辑放进 `cmd_control.hpp` 的纯函数，加 gtest
- 后端急停：取消所有任务、发 `/robot/estop true`、拒绝新任务；要手动 release
- 只是软件保护，不替代遥控器和硬件急停

## 7. 测试

本机：
- `task_runner`：`FakeNav`（手动控制到达 / 失败）+ 可注入时钟。覆盖多点 goto、失败、取消、preempt、巡航循环 / 单次、wait/resume、子任务后仍 waiting、超时、冷却期、急停、各安全检查
- 接口：FastAPI `TestClient`，覆盖认证、错误码格式、robot 配置、点位 / 路线增删改、旧 `points_goto`
- `robot_bridge` 急停 gtest
- 本机系统 python 没有 fastapi：按 `requirements.txt` 在仓库根建 `.venv_web/` 跑测试（不放 `ros_ws/src` 下，免得被 rsync 到机器人），不动系统和 conda

机器人上（要在线）：
1. 写 `robot.yaml`，`GET /api/v1/robot` 返回正确 id；旧网页功能照常
2. curl + token 下发 goto / guide，`/api/v1/events` 能看到进度和到达；guide 实际速度不超过限速
3. 两三个点的循环巡航，手动 wait → 插 guide → resume
4. 急停立即停车、拒绝新任务，解除后恢复
5. 看 `/localizer/score` 正常值，定 `min_score`

## 不在这一块

- 网页的路线编辑和巡航控制界面（下一小块，接口稳定后做）
- 云端、cloud_agent、平板（第 2 块；用户已有一台 ECS，第 2 块从「机器人连上这台 ECS 能互相调试」开始）
- 发现人自动触发：代码写好、默认关
