#include "lio_ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
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
    explicit LioEkfNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{})
        : Node("lio_ekf_node", opts)
    {
        declare_parameter("lio_odom_topic",   "/lio_odom");
        declare_parameter("yaw_speed_topic",  "/robot/yaw_speed");
        declare_parameter("pub_topic",        "/lio_ekf_odom");
        declare_parameter("odom_frame",       "odom");
        declare_parameter("base_frame",       "base_footprint");
        declare_parameter("publish_tf",       false);

        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();

        // LIO 里程计 → lio_buf_
        lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("lio_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                LioMeas m;
                m.stamp = stamp_to_sec(msg->header.stamp);
                m.px    = msg->pose.pose.position.x;
                m.py    = msg->pose.pose.position.y;
                tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                m.yaw = tf2::getYaw(q);
                push_lio(m);
            });

        // robot_bridge 发布的 yaw_speed (°/s) → imu_buf_（转 rad/s）
        yaw_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("yaw_speed_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                ImuMeas m;
                m.stamp    = now().seconds();
                m.yaw_rate = msg->data * (M_PI / 180.0);  // °/s → rad/s
                push_imu(m);
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            get_parameter("pub_topic").as_string(), 10);

        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        running_ = true;
        proc_thread_ = std::thread(&LioEkfNode::process_loop, this);
        pub_thread_  = std::thread(&LioEkfNode::publish_loop,  this);
    }

    ~LioEkfNode() {
        running_ = false;
        proc_cv_.notify_all();
        odom_cv_.notify_all();
        if (proc_thread_.joinable()) proc_thread_.join();
        if (pub_thread_.joinable())  pub_thread_.join();
    }

private:
    // ── 数据入队 ────────────────────────────────────────────────────────────
    void push_imu(const ImuMeas& m) {
        {
            std::lock_guard<std::mutex> lk(buf_mtx_);
            if (imu_buf_.size() >= BUF_MAX) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "imu_buf full, dropping oldest");
                imu_buf_.pop_front();
            }
            imu_buf_.push_back(m);
        }
        proc_cv_.notify_one();
    }

    void push_lio(const LioMeas& m) {
        {
            std::lock_guard<std::mutex> lk(buf_mtx_);
            if (lio_buf_.size() >= BUF_MAX) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "lio_buf full, dropping oldest");
                lio_buf_.pop_front();
            }
            lio_buf_.push_back(m);
        }
        proc_cv_.notify_one();
    }

    // ── 线程1：EKF predict + update ────────────────────────────────────────
    void process_loop() {
        double last_imu_t  = -1.0;
        bool   initialized = false;

        while (running_) {
            std::deque<ImuMeas> imu_work;
            std::deque<LioMeas> lio_work;
            {
                std::unique_lock<std::mutex> lk(buf_mtx_);
                proc_cv_.wait(lk, [this] {
                    return (!imu_buf_.empty() || !lio_buf_.empty()) || !running_;
                });
                if (!running_) break;
                imu_work.swap(imu_buf_);
                lio_work.swap(lio_buf_);
            }

            // IMU predict
            for (const auto& m : imu_work) {
                if (initialized && last_imu_t > 0.0) {
                    double dt = m.stamp - last_imu_t;
                    if (dt > 0.0) ekf_.predict(dt, m.yaw_rate);
                }
                last_imu_t = m.stamp;
            }

            // LIO update
            bool   updated      = false;
            double latest_stamp = 0.0;
            for (const auto& m : lio_work) {
                if (!initialized) {
                    ekf_.set_state(m.px, m.py, m.yaw);
                    initialized = true;
                    last_imu_t  = m.stamp;
                } else {
                    ekf_.update(m.px, m.py, m.yaw);
                }
                latest_stamp = m.stamp;
                updated = true;
            }

            if (initialized && updated)
                enqueue_odom(latest_stamp);
        }
    }

    // ── 线程2：发布里程计 / TF ──────────────────────────────────────────────
    void publish_loop() {
        while (running_) {
            nav_msgs::msg::Odometry msg;
            {
                std::unique_lock<std::mutex> lk(odom_mtx_);
                odom_cv_.wait(lk, [this] {
                    return !odom_buf_.empty() || !running_;
                });
                if (!running_ && odom_buf_.empty()) break;
                msg = std::move(odom_buf_.front());
                odom_buf_.pop_front();
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

    // ── 打包当前 EKF 状态 ────────────────────────────────────────────────────
    void enqueue_odom(double stamp_sec) {
        const auto& x = ekf_.state();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x(2));

        auto sec  = static_cast<int32_t>(stamp_sec);
        auto nsec = static_cast<uint32_t>((stamp_sec - sec) * 1e9);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = rclcpp::Time(sec, nsec, get_clock()->get_clock_type());
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id  = base_frame_;
        msg.pose.pose.position.x    = x(0);
        msg.pose.pose.position.y    = x(1);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        {
            std::lock_guard<std::mutex> lk(odom_mtx_);
            if (odom_buf_.size() >= BUF_MAX) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "odom_buf full, dropping oldest");
                odom_buf_.pop_front();
            }
            odom_buf_.push_back(std::move(msg));
        }
        odom_cv_.notify_one();
    }

    static double stamp_to_sec(const builtin_interfaces::msg::Time& t) {
        return t.sec + t.nanosec * 1e-9;
    }

    static constexpr size_t BUF_MAX = 500;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  yaw_speed_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    // 输入缓冲（共用一把锁，消除 process_loop 里的嵌套加锁死锁风险）
    std::deque<ImuMeas> imu_buf_;
    std::deque<LioMeas> lio_buf_;
    std::mutex          buf_mtx_;
    std::condition_variable proc_cv_;

    // 输出缓冲
    std::deque<nav_msgs::msg::Odometry> odom_buf_;
    std::mutex                          odom_mtx_;
    std::condition_variable             odom_cv_;

    LioEkf            ekf_;
    std::thread       proc_thread_;
    std::thread       pub_thread_;
    std::atomic<bool> running_{false};

    std::string odom_frame_, base_frame_;
    bool        publish_tf_{false};
};

} // namespace lio_ekf

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lio_ekf::LioEkfNode>());
    rclcpp::shutdown();
    return 0;
}
