#include "lio_ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>

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
        declare_parameter("loop_rate_hz",   50.0);  // 保留参数，兼容 launch 文件

        odom_frame_   = get_parameter("odom_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        publish_tf_   = get_parameter("publish_tf").as_bool();

        // 订阅 LIO 里程计，放入待处理容器
        lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("lio_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                LioMeas m;
                m.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.px    = msg->pose.pose.position.x;
                m.py    = msg->pose.pose.position.y;
                tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                m.yaw = tf2::getYaw(q);
                {
                    std::lock_guard<std::mutex> lk(lio_mtx_);
                    if (lio_buf_.size() >= BUF_MAX) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "lio_buf_ full, dropping oldest");
                        lio_buf_.pop_front();
                    }
                    lio_buf_.push_back(m);
                }
                proc_cv_.notify_one();
            });

        // 订阅 IMU，放入待处理容器
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("imu_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                ImuMeas m;
                m.stamp    = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                m.yaw_rate = msg->angular_velocity.z;
                {
                    std::lock_guard<std::mutex> lk(imu_mtx_);
                    if (imu_buf_.size() >= BUF_MAX) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "imu_buf_ full, dropping oldest");
                        imu_buf_.pop_front();
                    }
                    imu_buf_.push_back(m);
                }
                proc_cv_.notify_one();
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            get_parameter("pub_topic").as_string(), 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        running_ = true;

        // 线程1：消费 imu_buf_ + lio_buf_，跑 EKF，结果放入 odom_dequq_will_publish_
        proc_thread_ = std::thread(&LioEkfNode::process_loop, this);

        // 线程2：消费 odom_dequq_will_publish_，对外发布
        pub_thread_ = std::thread(&LioEkfNode::publish_loop, this);
    }

    ~LioEkfNode() {
        running_ = false;
        proc_cv_.notify_all();
        odom_cv_.notify_all();
        if (proc_thread_.joinable()) proc_thread_.join();
        if (pub_thread_.joinable())  pub_thread_.join();
    }

private:

    // 线程1：EKF predict + update
    void process_loop()
    {
        double last_imu_t = -1.0;
        bool   initialized = false;

        while (running_) {
            // 等待任意一个缓冲区有数据
            {
                std::unique_lock<std::mutex> lk(proc_wait_mtx_);
                proc_cv_.wait(lk, [this] {
                    std::lock_guard<std::mutex> li(imu_mtx_);
                    std::lock_guard<std::mutex> ll(lio_mtx_);
                    return (!imu_buf_.empty() || !lio_buf_.empty()) || !running_;
                });
            }
            if (!running_) break;

            // IMU predict：把当前积累的所有 IMU 帧都处理完
            {
                std::lock_guard<std::mutex> lk(imu_mtx_);
                while (!imu_buf_.empty()) {
                    const ImuMeas & m = imu_buf_.front();
                    if (initialized && last_imu_t > 0.0) {
                        double dt = m.stamp - last_imu_t;
                        if (dt > 0.0) ekf_.predict(dt, m.yaw_rate);
                    }
                    last_imu_t = m.stamp;
                    imu_buf_.pop_front();
                }
            }

            // LIO update
            bool updated = false;
            double latest_stamp = 0.0;
            {
                std::lock_guard<std::mutex> lk(lio_mtx_);
                while (!lio_buf_.empty()) {
                    const LioMeas & m = lio_buf_.front();
                    if (!initialized) {
                        ekf_.set_state(m.px, m.py, m.yaw);
                        initialized  = true;
                        last_imu_t   = m.stamp;
                    } else {
                        ekf_.update(m.px, m.py, m.yaw);
                    }
                    latest_stamp = m.stamp;
                    updated = true;
                    lio_buf_.pop_front();
                }
            }

            // 只有 LIO 观测到来时才产生一帧输出（避免 IMU-only 时重复发布）
            if (initialized && updated) {
                enqueue_odom(latest_stamp);
            }
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

    // 把当前 EKF 状态打包放入待发布容器
    void enqueue_odom(double stamp_sec)
    {
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

        {
            std::lock_guard<std::mutex> lk(odom_mtx_will_publish_);
            if (odom_dequq_will_publish_.size() >= BUF_MAX) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "odom_dequq_will_publish_ full, dropping oldest");
                odom_dequq_will_publish_.pop_front();
            }
            odom_dequq_will_publish_.push_back(msg);
        }
        odom_cv_.notify_one();
    }

    static constexpr size_t BUF_MAX = 500;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    // 待处理的输入容器
    std::deque<ImuMeas> imu_buf_;
    std::deque<LioMeas> lio_buf_;
    std::mutex          imu_mtx_, lio_mtx_;

    // proc_thread_ 的等待条件（imu 或 lio 任意一个有数据即唤醒）
    std::mutex              proc_wait_mtx_;
    std::condition_variable proc_cv_;

    // 待发布的里程计容器
    std::deque<nav_msgs::msg::Odometry> odom_dequq_will_publish_;
    std::mutex                          odom_mtx_will_publish_;
    std::condition_variable             odom_cv_;

    // EKF（仅 proc_thread_ 访问，无需额外锁）
    LioEkf ekf_;

    std::thread       proc_thread_;
    std::thread       pub_thread_;
    std::atomic<bool> running_{false};

    std::string odom_frame_, base_frame_;
    bool        publish_tf_{false};
};

} // namespace lio_ekf

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lio_ekf::LioEkfNode>());
    rclcpp::shutdown();
    return 0;
}
