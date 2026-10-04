#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <atomic>
#include <deque>
#include <mutex>

namespace pcdmap {

struct ScanStamped {
    sensor_msgs::msg::LaserScan msg;
};

// Node 职责：订阅话题，把消息原样搬入队列，不做任何计算
class PcdMapReceiver : public rclcpp::Node
{
public:
    explicit PcdMapReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
    : Node("pcd_map_node", options)
    {
        declare_parameter("scan_topic", "/scan");
        declare_parameter("buf_max",    200);

        buf_max_ = static_cast<std::size_t>(get_parameter("buf_max").as_int());

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            get_parameter("scan_topic").as_string(),
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(scan_mtx_);
                scan_buf_.push_back({*msg});
                if (scan_buf_.size() > buf_max_) scan_buf_.pop_front();
            });
    }

    // 计算线程调用：取出并清空缓冲
    std::deque<ScanStamped> drain_scans()
    {
        std::lock_guard<std::mutex> lk(scan_mtx_);
        return std::exchange(scan_buf_, {});
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    std::mutex scan_mtx_;
    std::deque<ScanStamped> scan_buf_;
    std::size_t buf_max_{200};
};

} // namespace pcdmap
