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
    double yaw_rate;  // rad/s
};

struct YawMeas {
    double stamp;
    double yaw;       // rad
};

struct LioMeas {
    double stamp;
    double px, py, yaw;
    bool   valid;     // false：lio 本帧 ICP 被拒（只转 yaw、平移没测），速度估计跳过
};

class LioEkfNode : public rclcpp::Node {
public:
    explicit LioEkfNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{})
        : Node("lio_ekf_node", opts)
    {
        declare_parameter("lio_odom_topic",  "/lio_odom");
        declare_parameter("yaw_speed_topic", "/robot/yaw_speed");
        declare_parameter("yaw_topic",       "/robot/yaw");
        declare_parameter("pub_topic",       "/lio_ekf_odom");
        declare_parameter("odom_frame",      "lio_odom");
        declare_parameter("base_frame",      "base_footprint");
        declare_parameter("publish_tf",      false);

        // EKF 噪声参数，可通过 yaml 覆盖
        declare_parameter("q_xy",      0.025);
        declare_parameter("q_yaw",     1e-3);
        declare_parameter("r_lio_xy",  0.0025);
        declare_parameter("r_lio_yaw", 0.0012);
        declare_parameter("r_yaw",     0.0003);
        // 速度估计（twist.linear）：q_vel_acc 越大越跟手、越噪；r_vel_xy 是 lio 帧间位置噪声
        declare_parameter("q_vel_acc",      0.05);     // (m/s²)²·s
        declare_parameter("r_vel_xy",       0.0004);   // (0.02 m)²
        declare_parameter("vel_gate_chi2",  13.8);     // 2 自由度 99.9%
        declare_parameter("vel_max_outliers", 3);

        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();

        ekf_.set_Q    (get_parameter("q_xy").as_double(),
                       get_parameter("q_yaw").as_double());
        ekf_.set_R_lio(get_parameter("r_lio_xy").as_double(),
                       get_parameter("r_lio_yaw").as_double());
        ekf_.set_r_yaw(get_parameter("r_yaw").as_double());
        vel_kf_.set_params(get_parameter("q_vel_acc").as_double(),
                           get_parameter("r_vel_xy").as_double(),
                           get_parameter("vel_gate_chi2").as_double(),
                           static_cast<int>(get_parameter("vel_max_outliers").as_int()));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            get_parameter("pub_topic").as_string(), 10);

        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // /robot/yaw_speed (°/s) → predict 队列
        yaw_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("yaw_speed_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                ImuMeas m;
                m.stamp    = now().seconds();
                m.yaw_rate = msg->data * (M_PI / 180.0);
                push_imu(m);
            });

        // /robot/yaw (°) → yaw update 队列
        yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("yaw_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                YawMeas m;
                m.stamp = now().seconds();
                m.yaw   = msg->data * (M_PI / 180.0);
                push_yaw(m);
            });

        // lio_odom → lio update 队列
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
                // lio_slam_node 对被拒帧把位置方差设成很大；旧版全 0 也当有效
                m.valid = msg->pose.covariance[0] < 1.0;
                push_lio(m);
            });

        running_     = true;
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
    void push_imu(const ImuMeas& m) {
        { std::lock_guard<std::mutex> lk(buf_mtx_); imu_buf_.push_back(m); trim(imu_buf_); }
        proc_cv_.notify_one();
    }
    void push_yaw(const YawMeas& m) {
        { std::lock_guard<std::mutex> lk(buf_mtx_); yaw_buf_.push_back(m); trim(yaw_buf_); }
        proc_cv_.notify_one();
    }
    void push_lio(const LioMeas& m) {
        { std::lock_guard<std::mutex> lk(buf_mtx_); lio_buf_.push_back(m); trim(lio_buf_); }
        proc_cv_.notify_one();
    }

    template<typename T>
    void trim(std::deque<T>& q) {
        while (q.size() > BUF_MAX) q.pop_front();
    }

    // ── 线程1：EKF predict / update ────────────────────────────────────────
    void process_loop() {
        double last_imu_t  = -1.0;
        bool   initialized = false;

        while (running_) {
            // 每次只取一帧 IMU，其余测量批量取
            ImuMeas  imu{};
            bool     has_imu = false;
            std::deque<YawMeas> yaw_work;
            std::deque<LioMeas> lio_work;
            {
                std::unique_lock<std::mutex> lk(buf_mtx_);
                proc_cv_.wait(lk, [this] {
                    return !imu_buf_.empty() || !yaw_buf_.empty()
                        || !lio_buf_.empty() || !running_;
                });
                if (!running_) break;

                // 每次只消费一帧 IMU，保证 dt 是真实相邻帧间隔
                if (!imu_buf_.empty()) {
                    imu     = imu_buf_.front();
                    imu_buf_.pop_front();
                    has_imu = true;
                }
                yaw_work.swap(yaw_buf_);
                lio_work.swap(lio_buf_);
            }

            // predict（单帧，dt 是真实相邻帧间隔）
            if (has_imu) {
                if (initialized && last_imu_t > 0.0) {
                    double dt = imu.stamp - last_imu_t;
                    if (dt > 0.0) ekf_.predict(dt, imu.yaw_rate);
                }
                last_imu_t = imu.stamp;
                yaw_rate_  = imu.yaw_rate;
            }

            // yaw update
            for (const auto& m : yaw_work) {
                if (initialized)
                    ekf_.update_yaw(m.yaw);
            }

            // lio update
            bool   updated      = false;
            double latest_stamp = 0.0;
            for (const auto& m : lio_work) {
                if (!initialized) {
                    ekf_.set_state(m.px, m.py, m.yaw);
                    initialized = true;
                    last_imu_t  = m.stamp;
                } else {
                    ekf_.update_lio(m.px, m.py, m.yaw);
                }
                if (!vel_kf_.step(m.stamp, m.px, m.py, m.valid))
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "velocity KF: lio position outlier rejected");
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

        // twist 在 child_frame（车体系）下：odom 系速度按 yaw 转回车体
        if (vel_kf_.initialized()) {
            const double c = std::cos(x(2)), s = std::sin(x(2));
            const Eigen::Vector2d v = vel_kf_.velocity();
            Eigen::Matrix2d R;
            R << c, s, -s, c;   // R(yaw)^T
            const Eigen::Matrix2d Pv = R * vel_kf_.velocity_cov() * R.transpose();
            msg.twist.twist.linear.x = R.row(0).dot(v);
            msg.twist.twist.linear.y = R.row(1).dot(v);
            msg.twist.covariance[0]  = Pv(0, 0);
            msg.twist.covariance[1]  = msg.twist.covariance[6] = Pv(0, 1);
            msg.twist.covariance[7]  = Pv(1, 1);
        }
        msg.twist.twist.angular.z = yaw_rate_;   // 陀螺直接测量
        msg.twist.covariance[35]  = 1e-3;

        {
            std::lock_guard<std::mutex> lk(odom_mtx_);
            if (odom_buf_.size() >= BUF_MAX) odom_buf_.pop_front();
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
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  yaw_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    std::deque<ImuMeas> imu_buf_;
    std::deque<YawMeas> yaw_buf_;
    std::deque<LioMeas> lio_buf_;
    std::mutex          buf_mtx_;
    std::condition_variable proc_cv_;

    std::deque<nav_msgs::msg::Odometry> odom_buf_;
    std::mutex                          odom_mtx_;
    std::condition_variable             odom_cv_;

    LioEkf            ekf_;
    VelocityKf        vel_kf_;    // 仅 proc_thread_ 读写
    double            yaw_rate_{0.0};
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
