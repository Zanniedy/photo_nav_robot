#include "lio_ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace lio_ekf {

struct ImuMeas {
    double stamp;
    double yaw_rate;
};

struct LioMeas {
    double stamp;
    double px, py, yaw;
};

class LioEkfNode : public rclcpp::Node {
public:
    explicit LioEkfNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("lio_ekf_node", opts)
    {
        declare_parameter("lio_odom_topic", "/lio_odom");
        declare_parameter("imu_topic",      "/imu");
        declare_parameter("pub_topic",      "/lio_ekf_odom");
        declare_parameter("odom_frame",     "odom");
        declare_parameter("base_frame",     "base_footprint");
        declare_parameter("publish_tf",     false);
        declare_parameter("loop_rate_hz",   50.0);

        odom_frame_   = get_parameter("odom_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        publish_tf_   = get_parameter("publish_tf").as_bool();
        loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();

        // /lio_odom は自前ノードが出す BEST_EFFORT
        lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("lio_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                LioMeas m;
                m.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.px    = msg->pose.pose.position.x;
                m.py    = msg->pose.pose.position.y;
                // quaternion → yaw
                tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                m.yaw = tf2::getYaw(q);
                std::lock_guard<std::mutex> lk(lio_mtx_);
                lio_buf_.push_back(m);
                if (lio_buf_.size() > BUF_MAX) lio_buf_.pop_front();
            });

        // /imu は bridge から来る RELIABLE
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("imu_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                ImuMeas m;
                m.stamp    = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.yaw_rate = msg->angular_velocity.z;
                std::lock_guard<std::mutex> lk(imu_mtx_);
                imu_buf_.push_back(m);
                if (imu_buf_.size() > BUF_MAX) imu_buf_.pop_front();
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            get_parameter("pub_topic").as_string(), 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        running_ = true;
        loop_thread_ = std::thread(&LioEkfNode::loop, this);
    }

    ~LioEkfNode() {
        running_ = false;
        if (loop_thread_.joinable()) loop_thread_.join();
    }

private:
    void loop() {
        const auto period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / loop_rate_hz_));
        auto next    = std::chrono::steady_clock::now();
        double last_t = -1.0;
        bool   initialized = false;

        while (running_) {
            next += period;
            std::this_thread::sleep_until(next);

            const double now = get_clock()->now().seconds();

            // ── IMU predict ──
            {
                std::lock_guard<std::mutex> lk(imu_mtx_);
                while (!imu_buf_.empty()) {
                    const auto & m = imu_buf_.front();
                    if (initialized && last_t > 0.0) {
                        double dt = m.stamp - last_t;
                        if (dt > 0.0) ekf_.predict(dt, m.yaw_rate);
                    }
                    last_t = m.stamp;
                    imu_buf_.pop_front();
                }
            }

            // ── LIO update ──
            {
                std::lock_guard<std::mutex> lk(lio_mtx_);
                while (!lio_buf_.empty()) {
                    const auto & m = lio_buf_.front();
                    if (!initialized) {
                        ekf_.set_state(m.px, m.py, m.yaw);
                        initialized = true;
                    } else {
                        ekf_.update(m.px, m.py, m.yaw);
                    }
                    lio_buf_.pop_front();
                }
            }

            if (initialized) publish(now);
        }
    }

    void publish(double stamp_sec) {
        const auto & x = ekf_.state();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x(2));

        rclcpp::Time stamp(
            static_cast<int32_t>(stamp_sec),
            static_cast<uint32_t>((stamp_sec - std::floor(stamp_sec)) * 1e9),
            get_clock()->get_clock_type());

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id  = base_frame_;
        msg.pose.pose.position.x    = x(0);
        msg.pose.pose.position.y    = x(1);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        odom_pub_->publish(msg);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header         = msg.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x(0);
            tf.transform.translation.y = x(1);
            tf.transform.rotation      = msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    static constexpr size_t BUF_MAX = 500;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    std::deque<ImuMeas> imu_buf_;
    std::deque<LioMeas> lio_buf_;
    std::mutex imu_mtx_, lio_mtx_;

    LioEkf ekf_;
    std::thread       loop_thread_;
    std::atomic<bool> running_{false};

    std::string odom_frame_, base_frame_;
    bool   publish_tf_{false};
    double loop_rate_hz_{50.0};
};

} // namespace lio_ekf

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lio_ekf::LioEkfNode>());
    rclcpp::shutdown();
    return 0;
}
