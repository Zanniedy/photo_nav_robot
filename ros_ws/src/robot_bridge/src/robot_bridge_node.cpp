#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>

#include <algorithm>
#include <deque>
#include <stdexcept>

#include "serial_driver.hpp"
#include "common.hpp"
#include "cmd_control.hpp"

namespace robot_bridge {

// TX 任务队列：cmd_vel -> 串口帧
struct TxTask {
    uint8_t en;
    float   vy;
    float   wz;
};

// 发送过的速度指令（限幅后），线速度自适应按时间回查
struct CmdSample {
    double t;    // ROS 时间 s
    double vx;
    double wz;
};

// IMU worker 队列元素：解包后的原始帧数据
struct ImuSample {
    float  yaw;
    float  yaw_speed;
    double stamp;  // seconds, from node clock
};

class RobotBridgeNode : public rclcpp::Node {
public:
    explicit RobotBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("robot_bridge_node", options)
        , running_(false)
        , yaw_offset_(0.0f)
        , yaw_initialized_(false)
        , yaw_speed_filtered_(0.0f)
    {
        // 初始化参数：低通滤波系数 α（越小越平滑）和归零所需的稳定帧数
        alpha_              = this->declare_parameter<double>("imu_lpf_alpha",          0.2);
        yaw_speed_deadzone_ = this->declare_parameter<double>("imu_yaw_speed_deadzone", 0.3);
        // MCU yaw 正方向：实测原地左转(逆时针) MCU yaw 减小，即 MCU 顺时针为正，取 -1
        imu_yaw_sign_       = static_cast<float>(this->declare_parameter<double>("imu_yaw_sign", -1.0));
        // cmd_vel(m/s, rad/s) → MCU 摇杆量 [-1, 1]（2026-10-14 阶跃测试）：
        //   MCU 是比例控制，有起步死区，过死区后速度随 u 上升但不是直线（转向斜率 3.0→4.1 rad/s/u）。
        //   前馈改为实测 (u, 稳态速度) 表分段线性反查，表首点 = 刚过死区的 u；|v| 低于 *_min 视为 0
        //   默认表：转向是阶跃实测点；前进是旧线性模型 v=0.8(u-0.1) 取的点，待稳态实测替换
        vx_min_cmd_  = this->declare_parameter<double>("cmd_vx_min",      0.02);  // m/s
        wz_min_cmd_  = this->declare_parameter<double>("cmd_wz_min",      0.12);  // rad/s，原地转时低于此当 0（MPPI 小幅噪声）
        wz_min_moving_ = this->declare_parameter<double>("cmd_wz_min_moving", 0.01);  // rad/s，行进中的转向归零门限
        moving_v_full_ = this->declare_parameter<double>("cmd_moving_v_full", 0.10);  // m/s，前进速度到此值完全按"行进中"映射转向
        u_max_       = this->declare_parameter<double>("cmd_u_max",       0.60);  // 限幅：~0.4m/s / ~2rad/s
        set_table(vx_table_, "vx",
            this->declare_parameter<std::vector<double>>("cmd_vx_table_u", {0.28, 0.60, 1.00}),
            this->declare_parameter<std::vector<double>>("cmd_vx_table_v", {0.144, 0.40, 0.72}), true);
        set_table(wz_table_, "wz",
            this->declare_parameter<std::vector<double>>("cmd_wz_table_u", {0.17, 0.20, 0.30, 0.50}),
            this->declare_parameter<std::vector<double>>("cmd_wz_table_v", {0.45, 0.558, 0.855, 1.676}), true);
        // 角速度闭环：陀螺 yaw_speed 反馈 PI，叠加在前馈上。陀螺超时（MCU 不报）自动退回纯前馈
        //   默认增益按 增益≈3.7rad/s/u、纯滞后≈0.15s 估算：穿越频率约 2rad/s，相角裕度约 60°
        wz_closed_loop_ = this->declare_parameter<bool>  ("cmd_wz_closed_loop", true);
        wz_pi_.kp       = this->declare_parameter<double>("cmd_wz_kp",          0.05);  // u per rad/s
        wz_pi_.ki       = this->declare_parameter<double>("cmd_wz_ki",          0.5);   // u per rad
        wz_pi_.i_max    = this->declare_parameter<double>("cmd_wz_i_max",       0.15);  // u
        wz_u_floor_     = this->declare_parameter<double>("cmd_wz_u_floor",     0.12);  // 原地转时闭环允许压到的最小 |u|（行进中按 moving 线性降到 0）
        gyro_alpha_     = this->declare_parameter<double>("cmd_gyro_lpf_alpha", 0.5);   // 100Hz 下 EMA，只滤量化噪声，不要加滞后
        gyro_timeout_   = this->declare_parameter<double>("cmd_gyro_timeout",   0.2);   // s
        // 线速度：没有轮速，EKF 速度 10Hz、滞后约 0.3s，不做快环，只慢慢修正前馈增益 k（u = table(k·|v|)）
        vx_adapt_          = this->declare_parameter<bool>  ("cmd_vx_adapt",          true);
        // 前进、后退各一个 k：实测底盘不对称（同样 0.2m/s 指令，前进 0.17、后退 0.23），共用会互相抵消
        vx_gain_fwd_.rate  = this->declare_parameter<double>("cmd_vx_adapt_rate",     0.3);   // 1/s
        vx_gain_fwd_.k_min = this->declare_parameter<double>("cmd_vx_adapt_k_min",    0.8);
        vx_gain_fwd_.k_max = this->declare_parameter<double>("cmd_vx_adapt_k_max",    1.3);
        vx_gain_rev_       = vx_gain_fwd_;
        vx_adapt_delay_    = this->declare_parameter<double>("cmd_vx_adapt_delay",    0.4);   // s，MCU 滞后 + 速度估计滞后
        vx_adapt_min_v_    = this->declare_parameter<double>("cmd_vx_adapt_min_v",    0.10);  // m/s，低于此不更新
        vx_adapt_max_wz_   = this->declare_parameter<double>("cmd_vx_adapt_max_wz",   0.3);   // rad/s，转弯时不更新
        vx_adapt_max_std_  = this->declare_parameter<double>("cmd_vx_adapt_max_std",  0.15);  // m/s，EKF 速度标准差上限：稳态约 0.1（q_vel_acc=0.05），初始化/重置时 0.5，只用来排除后者
        vx_stall_ratio_    = this->declare_parameter<double>("cmd_vx_stall_ratio",    0.3);   // 实测 < 指令×此值视为失速
        odom_timeout_      = this->declare_parameter<double>("cmd_odom_timeout",      0.3);   // s
        const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/lio_ekf_odom");
        cmd_timeout_ = this->declare_parameter<double>("cmd_timeout",     0.5);   // s
        // 平滑：nav2 控制器 10Hz 输出是阶梯状的，这里以 tx_rate 做加速度 + 加加速度限制（S 曲线）后再换算摇杆量
        // （先限幅再过死区映射：起步直接跳过死区，不会因为慢慢爬坡而起步更迟）
        tx_rate_     = this->declare_parameter<double>("cmd_tx_rate",     50.0);  // Hz
        vx_acc_      = this->declare_parameter<double>("cmd_vx_acc",      0.4);   // m/s²
        vx_dec_      = this->declare_parameter<double>("cmd_vx_dec",      0.6);   // m/s²，减速给大些
        wz_acc_      = this->declare_parameter<double>("cmd_wz_acc",      1.2);   // rad/s²
        wz_dec_      = this->declare_parameter<double>("cmd_wz_dec",      1.8);   // rad/s²
        // 加加速度限制：加速度从 0 到满需要 acc/jerk 秒（0.4/1.5≈0.27s），起停不顿
        vx_jerk_     = this->declare_parameter<double>("cmd_vx_jerk",     1.5);   // m/s³
        wz_jerk_     = this->declare_parameter<double>("cmd_wz_jerk",     5.0);   // rad/s³
        // ROS angular.z>0 = 左转(逆时针)。实测 MCU Wz>0 → 顺时针（MCU yaw 增大、scan 匹配 -13°），取 -1
        cmd_wz_sign_ = static_cast<float>(this->declare_parameter<double>("cmd_wz_sign", -1.0));
        // 静止判定 / 漂移估计
        static_thresh_      = this->declare_parameter<double>("imu_static_thresh",  0.5);   // °/s
        static_frames_      = this->declare_parameter<int>   ("imu_static_frames",  20);
        bias_window_s_      = this->declare_parameter<double>("imu_bias_window_s",  1.0);
        bias_alpha_         = this->declare_parameter<double>("imu_bias_alpha",     0.2);
        std::string config_path = this->declare_parameter<std::string>(
            "config_path",
            ament_index_cpp::get_package_share_directory("robot_bridge") + "/config/serial.yaml"
        );

        try {
            YAML::Node config = YAML::LoadFile(config_path);
            driver_ = std::make_unique<serial::SerialDriver>(config["serial"]);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load config: %s", e.what());
            throw;
        }

        driver_->set_error_callback([this](const std::string& msg) {
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        });

        // 串口收到数据 -> 放入 rx 队列，由 rx_worker 处理
        driver_->set_data_callback([this](const uint8_t* data, std::size_t len) {
            std::vector<uint8_t> buf(data, data + len);
            {
                std::lock_guard<std::mutex> lk(rx_mutex_);
                rx_queue_.push(std::move(buf));
            }
            rx_cv_.notify_one();
        });

        // 发布话题
        yaw_pub_       = this->create_publisher<std_msgs::msg::Float32>("/robot/yaw",       10);
        yaw_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot/yaw_speed", 10);

        // 订阅 cmd_vel：只记录目标速度，由下面的定时器平滑后发送
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(tx_mutex_);
                target_vx_     = msg->linear.x;
                target_wz_     = msg->angular.z;
                last_cmd_time_ = std::chrono::steady_clock::now();
            });

        // EKF 速度：只用于线速度前馈增益自适应
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });

        // 软件急停（web 后端 /api/v1/estop 发）：transient local，bridge 重启也能拿到当前状态
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/estop", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (estop_.exchange(msg->data) != msg->data)
                    RCLCPP_WARN(this->get_logger(), msg->data ? "[ESTOP] 软件急停" : "[ESTOP] 解除急停");
            });

        // 调参用：[target_vx, cur_vx, meas_vx, u_vx, k_vx, target_wz, cur_wz, meas_wz, u_wz, i_wz]
        debug_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/cmd_debug", 10);

        // 增益、开关、表都可以运行时 ros2 param set
        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& ps) { return on_params(ps); });

        running_ = true;
        driver_->start();
        RCLCPP_INFO(this->get_logger(), "Serial driver started");

        // 发送定时器（兼心跳，MCU 需要 PC 持续发才上报）：tx_rate 下对目标速度做加减速限幅，
        // 换算摇杆量后发送。cmd_vel 超过 cmd_timeout 没更新（上层停了/崩了）目标归零
        const double dt = 1.0 / tx_rate_;
        tx_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            [this, dt]() { control_step(dt); });

        // TX worker
        tx_thread_  = std::thread(&RobotBridgeNode::tx_worker,  this);
        // RX worker
        rx_thread_  = std::thread(&RobotBridgeNode::rx_worker,  this);
        // IMU worker：滤波 + 归零 + 发布
        imu_thread_ = std::thread(&RobotBridgeNode::imu_worker, this);
    }

    ~RobotBridgeNode() {
        running_ = false;

        tx_cv_.notify_all();
        rx_cv_.notify_all();
        imu_cv_.notify_all();

        if (tx_thread_.joinable())  tx_thread_.join();
        if (rx_thread_.joinable())  rx_thread_.join();
        if (imu_thread_.joinable()) imu_thread_.join();

        if (driver_) driver_->stop();
    }

private:
    bool set_table(SpeedTable& t, const char* name, const std::vector<double>& u,
                   const std::vector<double>& v, bool fatal) {
        std::string err;
        if (t.set(u, v, &err)) return true;
        RCLCPP_ERROR(this->get_logger(), "cmd_%s_table 非法：%s", name, err.c_str());
        if (fatal) throw std::invalid_argument("bad cmd table");
        return false;
    }

    // 陀螺角速度（rad/s，ROS 逆时针为正，已扣 bias）；超时返回 false
    bool gyro_rate(double& w) {
        std::lock_guard<std::mutex> lk(gyro_mutex_);
        if (!gyro_valid_) return false;
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - gyro_time_).count();
        if (age > gyro_timeout_) return false;
        w = gyro_rate_;
        return true;
    }

    // ── 发送定时器：限幅 → 前馈 → 角速度 PI → 打包 ─────────────────────────
    void control_step(double dt) {
        double meas_wz = 0.0;
        const bool gyro_ok = gyro_rate(meas_wz);
        float dbg[10];
        {
            std::lock_guard<std::mutex> lk(tx_mutex_);
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_cmd_time_).count();
            if (age > cmd_timeout_) target_vx_ = target_wz_ = 0.0;
            if (estop_) {               // 急停期间不管 cmd_vel 是什么都输出 0
                apply_estop(target_vx_, target_wz_, vx_s_, wz_s_);
                wz_pi_.reset();
            }
            cur_vx_ = vx_s_.step(target_vx_, dt, vx_acc_, vx_dec_, vx_jerk_);
            cur_wz_ = wz_s_.step(target_wz_, dt, wz_acc_, wz_dec_, wz_jerk_);

            // 线速度：min 判定用原始指令，增益 k 只放大查表输入
            const double u_vx = std::abs(cur_vx_) < vx_min_cmd_ ? 0.0
                : feedforward_u(vx_gain(cur_vx_).k * cur_vx_, vx_table_, 0.0, u_max_);

            // 角速度：前馈为 0（指令低于门限）时停转并清积分；换向清积分。
            // moving：前进越快越按"行进中"映射，没有起步死区补偿，保证边走边转是连续的
            const double moving = std::clamp(std::abs(cur_vx_) / moving_v_full_, 0.0, 1.0);
            double u_wz = feedforward_wz(cur_wz_, wz_table_, wz_min_cmd_, wz_min_moving_, u_max_, moving);
            wz_pi_.u_floor = (1.0 - moving) * wz_u_floor_;
            const int sgn = u_wz > 0.0 ? 1 : (u_wz < 0.0 ? -1 : 0);
            if (sgn != wz_sgn_) { wz_pi_.reset(); wz_sgn_ = sgn; }
            if (sgn != 0 && wz_closed_loop_ && gyro_ok) {
                wz_pi_.u_max = u_max_;
                u_wz = wz_pi_.step(sgn, cur_wz_ - meas_wz, u_wz, dt);
            } else {
                wz_pi_.reset();
            }

            // 记录指令历史，线速度自适应按 odom 时间戳回查（对齐 MCU 和估计的滞后）
            cmd_hist_.push_back({this->now().seconds(), cur_vx_, cur_wz_});
            while (cmd_hist_.size() > 200) cmd_hist_.pop_front();

            // 队列只留最新一帧，串口慢时不积压旧指令
            while (!tx_queue_.empty()) tx_queue_.pop();
            tx_queue_.push(TxTask{1, static_cast<float>(u_vx), static_cast<float>(cmd_wz_sign_ * u_wz)});

            const double v[10] = {target_vx_, cur_vx_, meas_vx_, u_vx, vx_gain(cur_vx_).k,
                                  target_wz_, cur_wz_, gyro_ok ? meas_wz : NAN, u_wz, wz_pi_.integ};
            for (int i = 0; i < 10; ++i) dbg[i] = static_cast<float>(v[i]);
        }
        tx_cv_.notify_one();

        if (debug_pub_->get_subscription_count() > 0) {
            std_msgs::msg::Float32MultiArray m;
            m.data.assign(dbg, dbg + 10);
            debug_pub_->publish(m);
        }
    }

    // ── EKF 速度 → 线速度前馈增益自适应 ────────────────────────────────────
    // 只在"稳定直行"时更新：指令（回查到 delay 之前）够快、近似匀速、不怎么转、估计不确定度够小
    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double t     = rclcpp::Time(msg.header.stamp).seconds();
        const double v     = msg.twist.twist.linear.x;
        const double var_v = msg.twist.covariance[0];
        std::lock_guard<std::mutex> lk(tx_mutex_);
        meas_vx_ = v;
        if (!vx_adapt_ || cmd_hist_.empty()) return;

        CmdSample a{}, b{};
        if (!cmd_at(t - vx_adapt_delay_, a) || !cmd_at(t - vx_adapt_delay_ - 0.3, b)) return;
        const bool steady = std::abs(a.vx) >= vx_adapt_min_v_ && std::abs(a.vx - b.vx) < 0.03 &&
                            std::abs(a.wz) < vx_adapt_max_wz_ && std::abs(b.wz) < vx_adapt_max_wz_;
        const bool fresh  = std::abs(this->now().seconds() - t) < odom_timeout_;
        const bool sure   = var_v > 0.0 && std::sqrt(var_v) < vx_adapt_max_std_;
        const double dt   = last_adapt_t_ > 0.0 ? std::clamp(t - last_adapt_t_, 0.0, 0.2) : 0.0;
        last_adapt_t_ = t;
        if (!(steady && fresh && sure) || dt <= 0.0) return;
        // 失速：指令够大但几乎没动（遥控接管、急停、被线/障碍卡住），不是增益问题，别把 k 推到上限
        if (std::abs(v) < vx_stall_ratio_ * std::abs(a.vx)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[CMD] 底盘没响应：指令 vx=%.2f，实测 %.3f（遥控接管/急停/卡住？），暂停增益自适应", a.vx, v);
            return;
        }

        GainAdapter& g = vx_gain(a.vx);
        g.update(a.vx, v, dt);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[CMD] vx adapt: cmd=%.3f meas=%.3f k_%s=%.3f", a.vx, v, a.vx > 0 ? "fwd" : "rev", g.k);
    }

    // 按时间回查指令历史（调用方持 tx_mutex_）
    bool cmd_at(double t, CmdSample& out) const {
        if (cmd_hist_.empty() || t < cmd_hist_.front().t || t > cmd_hist_.back().t) return false;
        auto it = std::lower_bound(cmd_hist_.begin(), cmd_hist_.end(), t,
            [](const CmdSample& s, double v) { return s.t < v; });
        out = *it;
        return true;
    }

    rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter>& ps) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        std::lock_guard<std::mutex> lk(tx_mutex_);
        // 表的 u、v 是两个参数，单独改一个时拿另一个的当前值配对校验
        auto table = [&](SpeedTable& t, const char* name, const rclcpp::Parameter& p, bool is_u) {
            const std::string other = std::string("cmd_") + name + (is_u ? "_table_v" : "_table_u");
            std::vector<double> o = this->get_parameter(other).as_double_array();
            for (const auto& q : ps) if (q.get_name() == other) o = q.as_double_array();
            const auto& mine = p.as_double_array();
            if (!set_table(t, name, is_u ? mine : o, is_u ? o : mine, false)) {
                res.successful = false;
                res.reason = std::string("cmd_") + name + "_table 非法";
            }
        };
        for (const auto& p : ps) {
            const auto& n = p.get_name();
            if      (n == "cmd_wz_closed_loop")   { wz_closed_loop_ = p.as_bool(); wz_pi_.reset(); }
            else if (n == "cmd_wz_kp")            wz_pi_.kp = p.as_double();
            else if (n == "cmd_wz_ki")            wz_pi_.ki = p.as_double();
            else if (n == "cmd_wz_i_max")         wz_pi_.i_max = p.as_double();
            else if (n == "cmd_wz_u_floor")       wz_u_floor_ = p.as_double();
            else if (n == "cmd_wz_min_moving")    wz_min_moving_ = p.as_double();
            else if (n == "cmd_moving_v_full")    moving_v_full_ = std::max(1e-3, p.as_double());
            else if (n == "cmd_vx_adapt")         vx_adapt_ = p.as_bool();
            else if (n == "cmd_vx_adapt_rate")    vx_gain_fwd_.rate  = vx_gain_rev_.rate  = p.as_double();
            else if (n == "cmd_vx_adapt_k_min")   vx_gain_fwd_.k_min = vx_gain_rev_.k_min = p.as_double();
            else if (n == "cmd_vx_adapt_k_max")   vx_gain_fwd_.k_max = vx_gain_rev_.k_max = p.as_double();
            else if (n == "cmd_vx_adapt_delay")   vx_adapt_delay_ = p.as_double();
            else if (n == "cmd_vx_min")           vx_min_cmd_ = p.as_double();
            else if (n == "cmd_wz_min")           wz_min_cmd_ = p.as_double();
            else if (n == "cmd_u_max")            u_max_ = p.as_double();
            else if (n == "cmd_vx_acc")           vx_acc_ = p.as_double();
            else if (n == "cmd_vx_dec")           vx_dec_ = p.as_double();
            else if (n == "cmd_wz_acc")           wz_acc_ = p.as_double();
            else if (n == "cmd_wz_dec")           wz_dec_ = p.as_double();
            else if (n == "cmd_vx_jerk")          vx_jerk_ = p.as_double();
            else if (n == "cmd_wz_jerk")          wz_jerk_ = p.as_double();
            else if (n == "cmd_vx_table_u")       table(vx_table_, "vx", p, true);
            else if (n == "cmd_vx_table_v")       table(vx_table_, "vx", p, false);
            else if (n == "cmd_wz_table_u")       table(wz_table_, "wz", p, true);
            else if (n == "cmd_wz_table_v")       table(wz_table_, "wz", p, false);
        }
        return res;
    }

    // ── TX worker: 从队列取任务，打包 CRC16 帧，写串口 ──────────────────────
    void tx_worker() {
        while (running_) {
            TxTask task{};
            {
                std::unique_lock<std::mutex> lk(tx_mutex_);
                tx_cv_.wait(lk, [this] { return !tx_queue_.empty() || !running_; });
                if (!running_) break;
                task = tx_queue_.front();
                tx_queue_.pop();
            }

            PcDownFrame f{};
            f.EN        = task.en;
            f.Vy        = task.vy;
            f.Wz        = task.wz;
            f.Reserve_1 = 0.0f;
            f.Reserve_2 = 0.0f;

            std::vector<uint8_t> buf(sizeof(PcDownFrame));
            std::memcpy(buf.data(), &f, sizeof(PcDownFrame));
            crc16_append(buf.data(), buf.size());

            RCLCPP_DEBUG(this->get_logger(),
                "[TX] raw frame (%zu bytes): EN=%d Vy=%.2f Wz=%.2f CRC=0x%04X",
                buf.size(), f.EN, f.Vy, f.Wz,
                (uint16_t)(buf[17] | (buf[18] << 8)));

            driver_->write(buf);
        }
    }

    // ── RX worker: 从队列取原始字节，解析 PcUpFrame，发布话题 ───────────────
    void rx_worker() {
        std::vector<uint8_t> stash;
        std::size_t total_bytes = 0;
        std::size_t total_frames = 0;

        while (running_) {
            std::vector<uint8_t> chunk;
            {
                std::unique_lock<std::mutex> lk(rx_mutex_);
                rx_cv_.wait(lk, [this] { return !rx_queue_.empty() || !running_; });
                if (!running_) break;
                chunk = std::move(rx_queue_.front());
                rx_queue_.pop();
            }

            total_bytes += chunk.size();
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //     "[RX] raw bytes received total=%zu stash=%zu",
            //     total_bytes, stash.size());

            stash.insert(stash.end(), chunk.begin(), chunk.end());

            while (stash.size() >= sizeof(PcUpFrame)) {
                if (!crc16_verify(stash.data(), sizeof(PcUpFrame))) {
                    RCLCPP_WARN(this->get_logger(),
                        "[RX] CRC16 mismatch, re-aligning. stash[0]=0x%02X stash[1]=0x%02X",
                        stash[0], stash[1]);
                    stash.erase(stash.begin());
                    continue;
                }

                PcUpFrame f{};
                std::memcpy(&f, stash.data(), sizeof(PcUpFrame));
                stash.erase(stash.begin(), stash.begin() + sizeof(PcUpFrame));
                total_frames++;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[RX] frame #%zu Yaw=%.3f° YawSpeed=%.3f°/s",
                    total_frames, f.Yaw, f.Yaw_Speed);

                // 解包后交给 imu_worker 处理，不在此发布
                {
                    std::lock_guard<std::mutex> lk(imu_mutex_);
                    if (imu_queue_.size() < 200) {
                        // MCU 以顺时针为正，乘 imu_yaw_sign_ 转成 ROS 约定（逆时针为正）
                    imu_queue_.push({imu_yaw_sign_ * f.Yaw, imu_yaw_sign_ * f.Yaw_Speed,
                            this->now().seconds()});
                    }
                }
                imu_cv_.notify_one();
            }
        }
    }

    // ── IMU worker: 归零 + 静止零速更新(ZUPT) + bias 补偿 + 发布 ────────────
    //
    // MCU 上报的 yaw 静止时也会持续增长（gyro bias 积分）。处理方式：
    //   静止：连续 static_frames_ 帧 |yaw_speed| < static_thresh_ → 输出 yaw 冻结，
    //         该段 yaw 增量全部视为漂移扣掉，同时累计 (Σdyaw, Σdt) 估计 bias
    //   运动：输出 yaw += 原始增量 - bias * dt
    // bias 用整段静止窗口的 Σdyaw/Σdt 求，不用逐帧 dyaw/dt（主机时间戳受串口批量到达影响，逐帧 dt 很抖）
    static double wrap_deg(double a) {
        while (a >  180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    void imu_worker() {
        double bias_deg_per_s = 0.0;   // 估计的漂移速率 (°/s)
        double yaw_out        = 0.0;   // 补偿后的 yaw (°)
        double last_yaw_raw   = 0.0;
        double last_stamp     = -1.0;
        int    static_count   = 0;     // 连续低角速度帧数
        double win_dyaw       = 0.0;   // 当前静止窗口累计漂移 (°)
        double win_dt         = 0.0;   // 当前静止窗口累计时长 (s)

        while (running_) {
            ImuSample s{};
            {
                std::unique_lock<std::mutex> lk(imu_mutex_);
                imu_cv_.wait(lk, [this] { return !imu_queue_.empty() || !running_; });
                if (!running_) break;
                s = imu_queue_.front();
                imu_queue_.pop();
            }

            // ── 第一帧归零 ──────────────────────────────────────────────────
            if (!yaw_initialized_) {
                yaw_offset_      = s.yaw;
                yaw_initialized_ = true;
                last_yaw_raw     = s.yaw;
                last_stamp       = s.stamp;
                RCLCPP_INFO(this->get_logger(),
                    "[IMU] initialized: yaw_offset=%.3f°", yaw_offset_);
                continue;
            }

            double dt = s.stamp - last_stamp;
            last_stamp = s.stamp;
            if (dt <= 0.0 || dt > 0.5) dt = 0.0;   // 异常间隔不参与 bias 计算

            const double dyaw = wrap_deg(s.yaw - last_yaw_raw);   // 本帧原始增量
            last_yaw_raw = s.yaw;

            // ── 静止判定 ────────────────────────────────────────────────────
            if (std::abs(s.yaw_speed) < static_thresh_) {
                static_count = std::min(static_count + 1, static_frames_);
            } else {
                static_count = 0;
            }
            const bool is_static = (static_count >= static_frames_);

            if (is_static) {
                // ZUPT：yaw_out 不动，增量全部计入漂移
                win_dyaw += dyaw;
                win_dt   += dt;
                // 窗口够长就更新一次 bias，然后开新窗口
                if (win_dt >= bias_window_s_) {
                    const double sample = win_dyaw / win_dt;
                    bias_deg_per_s = (1.0 - bias_alpha_) * bias_deg_per_s + bias_alpha_ * sample;
                    win_dyaw = 0.0;
                    win_dt   = 0.0;
                }
            } else {
                win_dyaw = 0.0;
                win_dt   = 0.0;
                yaw_out  = wrap_deg(yaw_out + dyaw - bias_deg_per_s * dt);
            }

            // ── 角速度闭环反馈：原始陀螺扣 bias，轻 EMA（不用下面带死区的发布值）──
            {
                const double w = (s.yaw_speed - bias_deg_per_s) * (M_PI / 180.0);
                std::lock_guard<std::mutex> lk(gyro_mutex_);
                gyro_rate_  = gyro_valid_ ? gyro_alpha_ * w + (1.0 - gyro_alpha_) * gyro_rate_ : w;
                gyro_valid_ = true;
                gyro_time_  = std::chrono::steady_clock::now();
            }

            // ── yaw_speed: EMA + 死区 ────────────────────────────────────────
            float yaw_speed_ema = static_cast<float>(alpha_) * s.yaw_speed
                                + (1.0f - static_cast<float>(alpha_)) * yaw_speed_filtered_;
            yaw_speed_filtered_ = (std::abs(yaw_speed_ema) < static_cast<float>(yaw_speed_deadzone_))
                                  ? 0.0f : yaw_speed_ema;

            auto yaw_msg       = std_msgs::msg::Float32();
            auto yaw_speed_msg = std_msgs::msg::Float32();
            yaw_msg.data       = static_cast<float>(yaw_out);
            yaw_speed_msg.data = yaw_speed_filtered_;

            yaw_pub_->publish(yaw_msg);
            yaw_speed_pub_->publish(yaw_speed_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[IMU] yaw=%.3f° bias=%.4f°/s static=%d yaw_speed=%.3f°/s (raw: yaw=%.3f° speed=%.3f°/s)",
                yaw_out, bias_deg_per_s, is_static, static_cast<double>(yaw_speed_filtered_),
                wrap_deg(s.yaw - yaw_offset_), static_cast<double>(s.yaw_speed));
        }
    }

    std::unique_ptr<serial::SerialDriver> driver_;

    // TX
    std::queue<TxTask>      tx_queue_;
    std::mutex              tx_mutex_;
    std::condition_variable tx_cv_;
    std::thread             tx_thread_;
    double                  target_vx_{0.0}, target_wz_{0.0};         // 受 tx_mutex_ 保护
    double                  cur_vx_{0.0},    cur_wz_{0.0};            // 受 tx_mutex_ 保护
    std::chrono::steady_clock::time_point last_cmd_time_{};          // 受 tx_mutex_ 保护
    double                  tx_rate_, vx_acc_, vx_dec_, wz_acc_, wz_dec_, vx_jerk_, wz_jerk_;
    SmoothAxis              vx_s_, wz_s_;                            // 受 tx_mutex_ 保护
    double                  vx_min_cmd_, wz_min_cmd_, u_max_;      // 以下控制参数/状态受 tx_mutex_ 保护
    double                  wz_min_moving_, moving_v_full_, wz_u_floor_;
    SpeedTable              vx_table_, wz_table_;
    bool                    wz_closed_loop_{true};
    RatePi                  wz_pi_;
    int                     wz_sgn_{0};
    bool                    vx_adapt_{true};
    GainAdapter             vx_gain_fwd_, vx_gain_rev_;
    GainAdapter& vx_gain(double v) { return v >= 0.0 ? vx_gain_fwd_ : vx_gain_rev_; }
    double                  vx_adapt_delay_, vx_adapt_min_v_, vx_adapt_max_wz_, vx_adapt_max_std_;
    double                  odom_timeout_, vx_stall_ratio_;
    double                  meas_vx_{0.0};
    double                  last_adapt_t_{-1.0};
    std::deque<CmdSample>   cmd_hist_;

    // 陀螺反馈（imu_worker 写，发送定时器读）
    std::mutex              gyro_mutex_;
    double                  gyro_rate_{0.0};
    bool                    gyro_valid_{false};
    std::chrono::steady_clock::time_point gyro_time_{};
    double                  gyro_alpha_, gyro_timeout_;
    double                  cmd_timeout_;
    float                   cmd_wz_sign_;

    // RX
    std::queue<std::vector<uint8_t>> rx_queue_;
    std::mutex                       rx_mutex_;
    std::condition_variable          rx_cv_;
    std::thread                      rx_thread_;

    // IMU worker
    std::queue<ImuSample>   imu_queue_;
    std::mutex              imu_mutex_;
    std::condition_variable imu_cv_;
    std::thread             imu_thread_;

    // IMU 滤波 / 归零状态
    double      alpha_;
    double      yaw_speed_deadzone_;
    float       imu_yaw_sign_;
    double      static_thresh_;
    int         static_frames_;
    double      bias_window_s_;
    double      bias_alpha_;
    float       yaw_offset_;
    bool        yaw_initialized_;
    float       yaw_speed_filtered_;

    std::atomic<bool> running_;
    std::atomic<bool> estop_{false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_speed_pub_;
    rclcpp::TimerBase::SharedPtr                                tx_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        estop_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
    OnSetParametersCallbackHandle::SharedPtr                    param_cb_;
};

} // namespace robot_bridge

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_bridge::RobotBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
