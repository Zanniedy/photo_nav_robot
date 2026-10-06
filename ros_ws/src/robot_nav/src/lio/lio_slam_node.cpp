#include "lio_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <cmath>

namespace lioslam {

class LioSlamNode : public rclcpp::Node {
public:

    explicit LioSlamNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("lio_slam_node", opts)
    {
        declare_parameter("scan_topic",   "/scan");
        declare_parameter("pub_topic",    "/lio_odom");
        declare_parameter("odom_frame",   "odom");
        declare_parameter("base_frame",   "base_footprint");
        declare_parameter("publish_tf",   true);
        declare_parameter("buf_max",      200);
        declare_parameter("loop_rate_hz", 10.0);

        scan_topic_   = get_parameter("scan_topic").as_string();
        pub_topic_    = get_parameter("pub_topic").as_string();
        odom_frame_   = get_parameter("odom_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        publish_tf_   = get_parameter("publish_tf").as_bool();
        buf_max_      = static_cast<size_t>(get_parameter("buf_max").as_int());
        loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_, 10);

        // 订阅激光扫描，收到后放入待处理容器
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                {
                    std::lock_guard<std::mutex> lk(scan_mtx_will_process_);
                    if (scan_dequq_will_process_.size() >= buf_max_) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "scan_dequq_will_process_ full, dropping oldest");
                        scan_dequq_will_process_.pop_front();
                    }
                    scan_dequq_will_process_.push_back(msg);
                }
                scan_cv_.notify_one();
            });

        running_ = true;

        // 线程1：从待处理容器取扫描帧，跑 ICP，结果放入待发布容器
        proc_thread_ = std::thread(&LioSlamNode::process_loop, this);

        // 线程2：从待发布容器取里程计，对外发布
        pub_thread_ = std::thread(&LioSlamNode::publish_loop, this);
    }

    ~LioSlamNode() {
        running_ = false;
        scan_cv_.notify_all();
        odom_cv_.notify_all();
        if (proc_thread_.joinable()) proc_thread_.join();
        if (pub_thread_.joinable())  pub_thread_.join();
    }

private:

    // 线程1：处理扫描帧
    void process_loop()
    {
        while (running_) {
            sensor_msgs::msg::LaserScan::SharedPtr scan;
            {
                std::unique_lock<std::mutex> lk(scan_mtx_will_process_);
                scan_cv_.wait(lk, [this] {
                    return !scan_dequq_will_process_.empty() || !running_;
                });
                if (!running_) break;
                scan = scan_dequq_will_process_.front();
                scan_dequq_will_process_.pop_front();
            }

            if (!prev_scan_) {
                prev_scan_ = scan;
                continue;
            }

            try {
                drobotpose delta = matcher_.match(*prev_scan_, *scan);

                // 单帧位移/转角超过阈值说明 ICP 错配，直接丢弃
                constexpr double MAX_TRANS = 0.30;  // m，根据实际最高速度调整
                constexpr double MAX_ROT   = 0.50;  // rad (~28°)
                const double dist = std::hypot(delta.dx, delta.dy);
                if (dist > MAX_TRANS || std::abs(delta.dyaw) > MAX_ROT) {
                    RCLCPP_WARN(get_logger(),
                        "ICP outlier rejected: dx=%.4f dy=%.4f dyaw=%.4f",
                        delta.dx, delta.dy, delta.dyaw);
                } else {
                    RCLCPP_DEBUG(get_logger(),
                        "ICP delta: dx=%.4f dy=%.4f dyaw=%.4f  pose: x=%.3f y=%.3f yaw=%.3f",
                        delta.dx, delta.dy, delta.dyaw, x_, y_, yaw_);
                    integrate(delta);
                    enqueue_odom(scan->header.stamp);
                }
            } catch (const std::exception & e) {
                RCLCPP_WARN(get_logger(), "ICP failed: %s", e.what());
            }
            prev_scan_ = scan;
        }
    }

    // 线程2：发布里程计
    void publish_loop()
    {
        while (running_) {
            nav_msgs::msg::Odometry msg;
            {
                std::unique_lock<std::mutex> lk(odom_mtx_will_publish_);
                odom_cv_.wait(lk, [this] {
                    return !odom_dequq_will_publish_.empty() || !running_;
                });
                if (!running_ && odom_dequq_will_publish_.empty()) break;
                msg = odom_dequq_will_publish_.front();
                odom_dequq_will_publish_.pop_front();
            }

            odom_pub_->publish(msg);

            if (publish_tf_ && tf_broadcaster_) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header         = msg.header;
                tf.child_frame_id = base_frame_;
                tf.transform.translation.x = msg.pose.pose.position.x;
                tf.transform.translation.y = msg.pose.pose.position.y;
                tf.transform.rotation      = msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(tf);
            }
        }
    }

    void integrate(const drobotpose & d)
    {
        double cos_yaw = std::cos(yaw_);
        double sin_yaw = std::sin(yaw_);
        x_   += cos_yaw * d.dx - sin_yaw * d.dy;
        y_   += sin_yaw * d.dx + cos_yaw * d.dy;
        yaw_ += d.dyaw;
        yaw_  = std::atan2(std::sin(yaw_), std::cos(yaw_));
    }

    // 把当前位姿打包成 Odometry 放入待发布容器
    void enqueue_odom(const rclcpp::Time & stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id  = base_frame_;
        msg.pose.pose.position.x    = x_;
        msg.pose.pose.position.y    = y_;
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        {
            std::lock_guard<std::mutex> lk(odom_mtx_will_publish_);
            if (odom_dequq_will_publish_.size() >= buf_max_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "odom_dequq_will_publish_ full, dropping oldest");
                odom_dequq_will_publish_.pop_front();
            }
            odom_dequq_will_publish_.push_back(msg);
        }
        odom_cv_.notify_one();
    }

    // 参数
    std::string scan_topic_, pub_topic_, odom_frame_, base_frame_;
    bool   publish_tf_{true};
    size_t buf_max_{200};
    double loop_rate_hz_{10.0};

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;

    // 待处理的扫描帧容器
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_dequq_will_process_;
    std::mutex                                         scan_mtx_will_process_;
    std::condition_variable                            scan_cv_;

    // 待发布的里程计容器
    std::deque<nav_msgs::msg::Odometry> odom_dequq_will_publish_;
    std::mutex                          odom_mtx_will_publish_;
    std::condition_variable             odom_cv_;

    // ICP
    scan_match matcher_;
    sensor_msgs::msg::LaserScan::SharedPtr prev_scan_;

    // 累积位姿（仅 proc_thread_ 写，无需额外锁）
    double x_{0.0}, y_{0.0}, yaw_{0.0};

    // 线程
    std::thread       proc_thread_;
    std::thread       pub_thread_;
    std::atomic<bool> running_{false};
};

} // namespace lioslam

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lioslam::LioSlamNode>());
    rclcpp::shutdown();
    return 0;
}
