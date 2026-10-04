# 机器人项目交接文档

## 项目概览

平面移动机器人，目标：多传感器融合定位导航。

**传感器配置：**
- 单目 USB 摄像头
- 思岚 C1 2D LiDAR
- 底盘 IMU
- 无轮速里程计

**最终目标架构：**
```
单目+IMU ──→ ORB-SLAM3 VIO ──→ /odom_vo
思岚C1   ──→ slam_toolbox  ──→ /odom_lidar
底盘IMU  ──────────────────→ /imu/data
                 ↓
         robot_localization EKF
                 ↓
            /odom_fused → Nav2
```

---

## 当前状态

### 已完成

#### 1. 自研 vslam 模块
路径：`/home/lsy/cs_project/robot/ros_ws/src/vslam/`

核心文件：
- `src/camera/tracker.hpp` — LK 光流 + GFTT 特征点 + 关键帧策略
- `src/camera/pose_estimator.hpp` — findEssentialMat + recoverPose + Ceres BA 精化
- `src/camera/trajectory_viewer.hpp` — Pangolin 3D 轨迹可视化（独立渲染线程）
- `src/camera/vslam.hpp` — 主类，`PoseCallback` 回调
- `test/vslam_test.cpp` — 测试入口，信号处理，Pangolin 窗口

关键参数（`tracker.hpp`）：
```cpp
min_tracked_good  = 50    // 低于此切 TRACKING_BAD
min_tracked_bad   = 20    // 低于此切 LOST
min_kf_features   = 80    // 关键帧重检测阈值
min_kf_disp_px    = 15.0  // 关键帧位移阈值（像素）
```

位姿接受门限（`pose_estimator.hpp`）：
```cpp
min_inliers       = 30
min_inlier_ratio  = 0.5
max_rotation_deg  = 5.0
```

**已知问题：**
- 单目无尺度（recoverPose 输出单位尺度，位移数字无物理意义）
- 无回环检测，长时间漂移
- 这是根本性缺陷，不建议继续投入优化

#### 2. 通用线程池
路径：`/home/lsy/cs_project/robot/ros_ws/src/utils/threadpool/thread_pool.hpp`

```cpp
namespace aurora {
  class ThreadPool { ... };
  inline ThreadPool& global_thread_pool(std::size_t n = 0);
}
```

`submit()` 返回 `std::future`，全局单例。

#### 3. 第三方库
- `src/3rdparty/spdlog/` — v1.13.0，header-only，用 `FMT_HEADER_ONLY` 宏
- `src/3rdparty/pangolin/` — v0.9，**静态链接**（`BUILD_SHARED_LIBS=OFF`），组件化构建

### 未完成 / 下一步

**核心任务：换用 ORB-SLAM3**

自研 VO 效果差，根本原因是单目无尺度 + 无回环。
下一步直接用 ORB-SLAM3 单目+IMU 模式（VIO），IMU 提供尺度。

---

## 下一步详细计划

### Step 1：前置条件确认

需要用户提供：
1. IMU topic 名称（常见：`/imu/data`、`/imu/raw`）
2. 相机内参标定文件（`camera_info` 或 `.yaml`），没有的话先做标定
3. 相机-IMU 外参（空间变换 T_cam_imu + 时间偏移）

相机标定工具：`ros2 run camera_calibration cameracalibrator`

相机-IMU 联合标定工具：**Kalibr**
```bash
# Kalibr 输出 camchain.yaml，包含外参和时延
```

### Step 2：安装 ORB-SLAM3

```bash
# 依赖
sudo apt install -y libglew-dev libpangolin-dev libeigen3-dev

# 克隆
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3 && chmod +x build.sh && ./build.sh

# ROS2 wrapper（推荐）
git clone https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git
# 或
git clone https://github.com/zang09/ORB-SLAM3-ROS2
```

### Step 3：配置文件

ORB-SLAM3 需要 `yaml` 配置文件，关键字段：
```yaml
Camera.type: "PinHole"
Camera.fx: ???   # 标定得到
Camera.fy: ???
Camera.cx: ???
Camera.cy: ???
Camera.k1: ???   # 畸变系数
# ...
IMU.NoiseGyro: ???    # IMU 噪声参数（查数据手册或标定）
IMU.NoiseAcc: ???
IMU.T_b_c1: [...]     # 相机到IMU的变换矩阵（Kalibr输出）
```

### Step 4：EKF 融合

`robot_localization` 包，配置 `ekf.yaml`：
```yaml
odom0: /orb_slam3/odom          # VO 输出
odom0_config: [true, true, false,  # x y z
               false, false, true,  # roll pitch yaw
               ...]
imu0: /imu/data
# LiDAR odom 后续加入
odom1: /slam_toolbox/odom
```

---

## 工作空间结构

```
/home/lsy/cs_project/robot/ros_ws/
├── src/
│   ├── vslam/                    # 自研VO（当前在用，待替换）
│   │   ├── CMakeLists.txt
│   │   ├── src/camera/
│   │   │   ├── tracker.hpp
│   │   │   ├── pose_estimator.hpp
│   │   │   ├── trajectory_viewer.hpp
│   │   │   └── vslam.hpp
│   │   ├── test/vslam_test.cpp
│   │   └── 3rdparty/
│   │       ├── spdlog/
│   │       └── pangolin/
│   └── utils/
│       └── threadpool/
│           └── thread_pool.hpp
├── build/
└── install/
```

## 构建命令

```bash
cd /home/lsy/cs_project/robot/ros_ws
colcon build --packages-select vslam --cmake-args \
  -DCMAKE_PREFIX_PATH="/home/lsy/cs_project/robot/ros_ws/install" \
  -DCMAKE_BUILD_TYPE=Release
```

运行测试（需要加 conda lib 路径）：
```bash
LD_LIBRARY_PATH=/home/lsy/miniconda3/lib:$LD_LIBRARY_PATH \
  ./install/vslam/lib/vslam/vslam_test
```

---

## 技术决策记录

| 决策 | 原因 |
|------|------|
| LK 光流替换 ORB 描述子匹配 | 连续帧间 ORB 匹配不稳定，LK 更适合帧间追踪（slambook2 ch13） |
| GFTT 替换 ORB 检测 | 平面/低纹理场景 ORB 特征点质量差 |
| kf_disp 相对关键帧位移 | 帧间位移（~0.3px）太小无法触发位姿估计，改为相对关键帧累积位移 |
| Pangolin 静态链接 | 动态库运行时找不到 libpango_display.so，改为 BUILD_SHARED_LIBS=OFF |
| Ceres BA 精化 | recoverPose 初值噪声大，10次迭代 Huber loss 精化 R,t |
| 放弃自研转 ORB-SLAM3 | 单目无尺度是根本缺陷，成熟系统更可靠 |

---

## 环境信息

- OS：Ubuntu（x86_64）
- ROS：ROS2
- Python：conda base 环境，miniconda3
- 编译器：GCC，C++17
- 关键系统库：
  - OpenCV（ROS 提供）
  - Eigen3
  - Ceres Solver
  - libepoxy-dev（Pangolin 依赖）
  - yaml-cpp 0.9（在 `/home/lsy/miniconda3/lib/`）
