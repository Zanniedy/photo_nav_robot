#include "ekf.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>

namespace ekf {

// ── 量测数据结构 ────────────────────────────────────────────────────────────

struct OdomMeas {
    double stamp;
    double vx, vy, yaw_rate;
    double var_vx, var_vy, var_yr;
};

struct ImuMeas {
    double stamp;
    double yaw_rate;
    double var_yr;
};

// ── Node ───────────────────────────────────────────────────────────────────

class EkfCombinedNode : public rclcpp::Node
{
public:
    explicit EkfCombinedNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("ekf_combined_node", opts)
    {
        // 参数
        declare_parameter("odom_topic",    "/odom");
        declare_parameter("imu_topic",     "/imu");
        declare_parameter("pub_topic",     "/ekf_odom");
        declare_parameter("pub_frame",     "odom");
        declare_parameter("child_frame",   "base_footprint");
        declare_parameter("publish_tf",    true);
        declare_parameter("loop_rate_hz",  50.0);

        const auto odom_topic  = get_parameter("odom_topic").as_string();
        const auto imu_topic   = get_parameter("imu_topic").as_string();
        const auto pub_topic   = get_parameter("pub_topic").as_string();
        pub_frame_             = get_parameter("pub_frame").as_string();
        child_frame_           = get_parameter("child_frame").as_string();
        publish_tf_            = get_parameter("publish_tf").as_bool();
        loop_rate_hz_          = get_parameter("loop_rate_hz").as_double();

        // 订阅 — 只搬运数据
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                OdomMeas m;
                m.stamp    = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.vx       = msg->twist.twist.linear.x;
                m.vy       = msg->twist.twist.linear.y;
                m.yaw_rate = msg->twist.twist.angular.z;
                m.var_vx   = msg->twist.covariance[0];
                m.var_vy   = msg->twist.covariance[7];
                m.var_yr   = msg->twist.covariance[35];
                // 协方差为 0 说明没有填，给默认值
                if (m.var_vx  <= 0.0) m.var_vx  = 0.01;
                if (m.var_vy  <= 0.0) m.var_vy  = 0.01;
                if (m.var_yr  <= 0.0) m.var_yr  = 0.01;

                std::lock_guard<std::mutex> lk(odom_mtx_);
                odom_buf_.push_back(m);
                if (odom_buf_.size() > BUF_MAX) odom_buf_.pop_front();
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                ImuMeas m;
                m.stamp    = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.yaw_rate = msg->angular_velocity.z;
                m.var_yr   = msg->angular_velocity_covariance[8];
                if (m.var_yr <= 0.0) m.var_yr = 0.005;

                std::lock_guard<std::mutex> lk(imu_mtx_);
                imu_buf_.push_back(m);
                if (imu_buf_.size() > BUF_MAX) imu_buf_.pop_front();
            });

        // 发布
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic, 10);
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // EKF 线程
        running_ = true;
        ekf_thread_ = std::thread(&EkfCombinedNode::ekf_loop, this);

        RCLCPP_INFO(get_logger(), "ekf_combined_node started (%.0f Hz)", loop_rate_hz_);
    }

    ~EkfCombinedNode()
    {
        running_ = false;
        if (ekf_thread_.joinable()) ekf_thread_.join();
    }

private:
    void ekf_loop()
    {
        const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / loop_rate_hz_));
        auto next = std::chrono::steady_clock::now();
        double last_stamp = -1.0;

        while (running_) {
            next += period;
            std::this_thread::sleep_until(next);

            // 取当前时间戳（用 node clock 保持与 ROS 时间一致）
            const double now = get_clock()->now().seconds();

            // ── predict ──
            if (last_stamp > 0.0) {
                ekf_.predict(now - last_stamp);
            }
            last_stamp = now;

            // ── odom update ──
            {
                std::lock_guard<std::mutex> lk(odom_mtx_);
                while (!odom_buf_.empty()) {
                    const auto & m = odom_buf_.front();
                    ekf_.update_odom(m.vx, m.vy, m.yaw_rate,
                                     m.var_vx, m.var_vy, m.var_yr);
                    odom_buf_.pop_front();
                }
            }

            // ── imu update ──
            {
                std::lock_guard<std::mutex> lk(imu_mtx_);
                while (!imu_buf_.empty()) {
                    const auto & m = imu_buf_.front();
                    ekf_.update_imu(m.yaw_rate, m.var_yr);
                    imu_buf_.pop_front();
                }
            }

            publish(now);
        }
    }

    void publish(double stamp_sec)
    {
        const auto & x = ekf_.state();
        const double yaw = x(2);

        // yaw → quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        rclcpp::Time stamp(static_cast<int32_t>(stamp_sec),
                           static_cast<uint32_t>((stamp_sec - std::floor(stamp_sec)) * 1e9),
                           get_clock()->get_clock_type());

        // Odometry msg
        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = pub_frame_;
        msg.child_frame_id  = child_frame_;

        msg.pose.pose.position.x  = x(0);
        msg.pose.pose.position.y  = x(1);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        msg.twist.twist.linear.x  = x(3);
        msg.twist.twist.linear.y  = x(4);
        msg.twist.twist.angular.z = x(5);

        odom_pub_->publish(msg);

        // TF
        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header          = msg.header;
            tf.child_frame_id  = child_frame_;
            tf.transform.translation.x = x(0);
            tf.transform.translation.y = x(1);
            tf.transform.rotation      = msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    // ── 订阅 / 发布
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    // ── 数据缓冲
    static constexpr size_t BUF_MAX = 200;
    std::deque<OdomMeas> odom_buf_;
    std::deque<ImuMeas>  imu_buf_;
    std::mutex odom_mtx_;
    std::mutex imu_mtx_;

    // ── EKF & 线程
    Ekf ekf_;
    std::thread   ekf_thread_;
    std::atomic<bool> running_{false};

    // ── 参数
    std::string pub_frame_;
    std::string child_frame_;
    bool   publish_tf_{true};
    double loop_rate_hz_{50.0};
};

} // namespace ekf

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf::EkfCombinedNode>());
    rclcpp::shutdown();
    return 0;
}
