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

#include "serial_driver.hpp"
#include "common.hpp"

namespace robot_bridge {

// TX 任务队列：cmd_vel -> 串口帧
struct TxTask {
    uint8_t en;
    float   vy;
    float   wz;
};

class RobotBridgeNode : public rclcpp::Node {
public:
    explicit RobotBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("robot_bridge_node", options)
        , running_(false)
    {
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

        // 订阅 cmd_vel -> 放入 tx 队列
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                TxTask task{1,
                    static_cast<float>(msg->linear.y),
                    static_cast<float>(msg->angular.z)};
                {
                    std::lock_guard<std::mutex> lk(tx_mutex_);
                    tx_queue_.push(task);
                }
                tx_cv_.notify_one();
            });

        running_ = true;
        driver_->start();
        RCLCPP_INFO(this->get_logger(), "Serial driver started");

        // 心跳定时器：10Hz，保持 PC 在线状态
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                TxTask task{1, 0.0f, 0.0f};
                {
                    std::lock_guard<std::mutex> lk(tx_mutex_);
                    if (tx_queue_.empty()) tx_queue_.push(task);
                }
                tx_cv_.notify_one();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[TX] heartbeat sent EN=1 Vy=0 Wz=0");
            });

        // TX worker
        tx_thread_ = std::thread(&RobotBridgeNode::tx_worker, this);
        // RX worker
        rx_thread_ = std::thread(&RobotBridgeNode::rx_worker, this);
    }

    ~RobotBridgeNode() {
        running_ = false;

        tx_cv_.notify_all();
        rx_cv_.notify_all();

        if (tx_thread_.joinable()) tx_thread_.join();
        if (rx_thread_.joinable()) rx_thread_.join();

        if (driver_) driver_->stop();
    }

private:
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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[RX] raw bytes received total=%zu stash=%zu",
                total_bytes, stash.size());

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

                RCLCPP_INFO(this->get_logger(),
                    "[RX] frame #%zu Yaw=%.3f° YawSpeed=%.3f°/s",
                    total_frames, f.Yaw, f.Yaw_Speed);

                auto yaw_msg       = std_msgs::msg::Float32();
                auto yaw_speed_msg = std_msgs::msg::Float32();
                yaw_msg.data       = f.Yaw;
                yaw_speed_msg.data = f.Yaw_Speed;

                yaw_pub_->publish(yaw_msg);
                yaw_speed_pub_->publish(yaw_speed_msg);
            }
        }
    }

    std::unique_ptr<serial::SerialDriver> driver_;

    // TX
    std::queue<TxTask>      tx_queue_;
    std::mutex              tx_mutex_;
    std::condition_variable tx_cv_;
    std::thread             tx_thread_;

    // RX
    std::queue<std::vector<uint8_t>> rx_queue_;
    std::mutex                       rx_mutex_;
    std::condition_variable          rx_cv_;
    std::thread                      rx_thread_;

    std::atomic<bool> running_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        yaw_speed_pub_;
    rclcpp::TimerBase::SharedPtr                                heartbeat_timer_;
};

} // namespace robot_bridge

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_bridge::RobotBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
