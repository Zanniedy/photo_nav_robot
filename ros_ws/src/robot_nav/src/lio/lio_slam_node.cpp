#include "lio_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <deque>
#include <mutex>
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

        scan_topic_    = get_parameter("scan_topic").as_string();
        pub_topic_     = get_parameter("pub_topic").as_string();
        odom_frame_    = get_parameter("odom_frame").as_string();
        base_frame_    = get_parameter("base_frame").as_string();
        publish_tf_    = get_parameter("publish_tf").as_bool();
        buf_max_       = static_cast<size_t>(get_parameter("buf_max").as_int());
        loop_rate_hz_  = get_parameter("loop_rate_hz").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(scan_mtx_);
                scan_buf_.push_back(msg);
                if (scan_buf_.size() > buf_max_) scan_buf_.pop_front();
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_, 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        running_ = true;
        proc_thread_ = std::thread(&LioSlamNode::process_loop, this);
    }

    ~LioSlamNode() {
        running_ = false;
        if (proc_thread_.joinable()) proc_thread_.join();
    }

private:
    void process_loop()
    {
        const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);

        while (running_) {
            auto t0 = std::chrono::steady_clock::now();

            sensor_msgs::msg::LaserScan::SharedPtr scan;
            {
                std::lock_guard<std::mutex> lk(scan_mtx_);
                if (!scan_buf_.empty()) {
                    scan = scan_buf_.back();
                    scan_buf_.clear();
                }
            }

            if (scan) {
                if (!prev_scan_) {
                    // 第一帧，仅初始化
                    prev_scan_ = scan;
                } else {
                    try {
                        drobotpose delta = matcher_.match(*prev_scan_, *scan);
                        integrate(delta);
                        publish(scan->header.stamp);
                    } catch (const std::exception & e) {
                        RCLCPP_WARN(get_logger(), "ICP failed: %s", e.what());
                    }
                    prev_scan_ = scan;
                }
            }

            std::this_thread::sleep_until(t0 + period);
        }
    }

    void integrate(const drobotpose & d)
    {
        // 把局部坐标系下的位移转换到世界坐标系再累加
        double cos_yaw = std::cos(yaw_);
        double sin_yaw = std::sin(yaw_);
        x_   += cos_yaw * d.dx - sin_yaw * d.dy;
        y_   += sin_yaw * d.dx + cos_yaw * d.dy;
        yaw_ += d.dyaw;
        // 归一化到 [-pi, pi]
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
    }

    void publish(const rclcpp::Time & stamp)
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

        odom_pub_->publish(msg);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header         = msg.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.rotation      = msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
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

    // 数据缓冲
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_buf_;
    std::mutex scan_mtx_;

    // ICP
    scan_match matcher_;
    sensor_msgs::msg::LaserScan::SharedPtr prev_scan_;

    // 累积位姿
    double x_{0.0}, y_{0.0}, yaw_{0.0};

    // 处理线程
    std::thread       proc_thread_;
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
