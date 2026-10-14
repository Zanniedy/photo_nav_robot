#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
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

#include "serial_driver.hpp"
#include "common.hpp"

namespace robot_bridge {

// TX 任务队列：cmd_vel -> 串口帧
struct TxTask {
    uint8_t en;
    float   vy;
    float   wz;
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
        // cmd_vel(m/s, rad/s) → MCU 摇杆量 [-1, 1] 的标定（2026-10-14 阶跃测试）：
        //   MCU 是比例控制，不是只看正负号。摇杆量 u 有死区，过死区后速度近似线性：
        //   前进 v ≈ vx_gain*(u - vx_offset)，u<0.25 基本不动；u=1 约 0.63~0.7m/s
        //   转向 ω ≈ wz_gain*(u - wz_offset)，u=0.2/0.3/0.5 → 32/49/96 °/s，u<0.15 基本不动
        // 映射：u = sgn * (offset + |v|/gain)，再夹到 [u_min, 1]；|v| 低于 *_min_cmd 视为 0
        vx_gain_     = this->declare_parameter<double>("cmd_vx_gain",     0.80);  // m/s per u
        vx_offset_   = this->declare_parameter<double>("cmd_vx_offset",   0.10);
        vx_u_min_    = this->declare_parameter<double>("cmd_vx_u_min",    0.28);  // 刚过死区，~0.15m/s
        vx_min_cmd_  = this->declare_parameter<double>("cmd_vx_min",      0.02);  // m/s
        wz_gain_     = this->declare_parameter<double>("cmd_wz_gain",     3.73);  // rad/s per u
        wz_offset_   = this->declare_parameter<double>("cmd_wz_offset",   0.05);
        wz_u_min_    = this->declare_parameter<double>("cmd_wz_u_min",    0.17);  // 刚过死区，~0.45rad/s
        wz_min_cmd_  = this->declare_parameter<double>("cmd_wz_min",      0.12);  // rad/s，MPPI 输出有小幅噪声，低于此当 0，免得被抬到死区 0.45rad/s 左右抖
        u_max_       = this->declare_parameter<double>("cmd_u_max",       0.60);  // 限幅：~0.4m/s / ~2rad/s
        cmd_timeout_ = this->declare_parameter<double>("cmd_timeout",     0.5);   // s
        // 平滑：nav2 控制器 10Hz 输出是阶梯状的，这里以 tx_rate 对 m/s、rad/s 做加减速限幅后再换算摇杆量
        // （先限幅再过死区映射：起步直接跳过死区，不会因为慢慢爬坡而起步更迟）
        tx_rate_     = this->declare_parameter<double>("cmd_tx_rate",     50.0);  // Hz
        vx_acc_      = this->declare_parameter<double>("cmd_vx_acc",      0.6);   // m/s²
        vx_dec_      = this->declare_parameter<double>("cmd_vx_dec",      1.0);   // m/s²，减速给大些
        wz_acc_      = this->declare_parameter<double>("cmd_wz_acc",      2.0);   // rad/s²
        wz_dec_      = this->declare_parameter<double>("cmd_wz_dec",      3.0);   // rad/s²
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

        running_ = true;
        driver_->start();
        RCLCPP_INFO(this->get_logger(), "Serial driver started");

        // 发送定时器（兼心跳，MCU 需要 PC 持续发才上报）：tx_rate 下对目标速度做加减速限幅，
        // 换算摇杆量后发送。cmd_vel 超过 cmd_timeout 没更新（上层停了/崩了）目标归零
        const double dt = 1.0 / tx_rate_;
        tx_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            [this, dt]() {
                {
                    std::lock_guard<std::mutex> lk(tx_mutex_);
                    const double age = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - last_cmd_time_).count();
                    if (age > cmd_timeout_) target_vx_ = target_wz_ = 0.0;
                    cur_vx_ = slew(cur_vx_, target_vx_, vx_acc_ * dt, vx_dec_ * dt);
                    cur_wz_ = slew(cur_wz_, target_wz_, wz_acc_ * dt, wz_dec_ * dt);
                    // 队列只留最新一帧，串口慢时不积压旧指令
                    while (!tx_queue_.empty()) tx_queue_.pop();
                    tx_queue_.push(to_mcu(cur_vx_, cur_wz_));
                }
                tx_cv_.notify_one();
            });

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
    // ── cmd_vel → MCU 摇杆量（连续比例，前进和转向可同时给）─────────────────
    static float to_u(double v, double gain, double offset, double u_min, double v_min, double u_max) {
        if (std::abs(v) < v_min) return 0.0f;
        const double u = std::clamp(offset + std::abs(v) / gain, u_min, u_max);
        return static_cast<float>(v > 0.0 ? u : -u);
    }

    // 加减速限幅：|v| 增大用 acc，减小（含过零）用 dec
    static double slew(double cur, double target, double acc_step, double dec_step) {
        const double step = std::abs(target) > std::abs(cur) && cur * target >= 0.0 ? acc_step : dec_step;
        return cur + std::clamp(target - cur, -step, step);
    }

    TxTask to_mcu(double vx, double wz) const {
        return TxTask{1,
            to_u(vx, vx_gain_, vx_offset_, vx_u_min_, vx_min_cmd_, u_max_),
            cmd_wz_sign_ * to_u(wz, wz_gain_, wz_offset_, wz_u_min_, wz_min_cmd_, u_max_)};
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
    double                  tx_rate_, vx_acc_, vx_dec_, wz_acc_, wz_dec_;
    double                  vx_gain_, vx_offset_, vx_u_min_, vx_min_cmd_;
    double                  wz_gain_, wz_offset_, wz_u_min_, wz_min_cmd_;
    double                  u_max_;
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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_speed_pub_;
    rclcpp::TimerBase::SharedPtr                                tx_timer_;
};

} // namespace robot_bridge

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_bridge::RobotBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
