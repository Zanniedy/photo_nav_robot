#include "lio_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
        declare_parameter("yaw_topic",    "/robot/yaw");
        declare_parameter("odom_frame",   "odom");
        declare_parameter("base_frame",   "base_footprint");
        declare_parameter("publish_tf",   true);
        declare_parameter("buf_max",      200);
        declare_parameter("loop_rate_hz", 10.0);
        declare_parameter("max_trans",    0.10);   // m/帧，按最高速度调整
        declare_parameter("max_rot",      0.30);   // rad/帧 (~17°)
        declare_parameter("kf_trans",     0.30);   // 相对关键帧平移超过此值换关键帧 (m)
        declare_parameter("kf_rot",       0.35);   // 相对关键帧旋转超过此值换关键帧 (rad)
        declare_parameter("max_rejects",  10);     // 连续拒绝超过此数强制重置关键帧

        scan_topic_   = get_parameter("scan_topic").as_string();
        pub_topic_    = get_parameter("pub_topic").as_string();
        odom_frame_   = get_parameter("odom_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        publish_tf_   = get_parameter("publish_tf").as_bool();
        buf_max_      = static_cast<size_t>(get_parameter("buf_max").as_int());
        loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
        max_trans_    = get_parameter("max_trans").as_double();
        max_rot_      = get_parameter("max_rot").as_double();
        kf_trans_     = get_parameter("kf_trans").as_double();
        kf_rot_       = get_parameter("kf_rot").as_double();
        max_rejects_  = static_cast<int>(get_parameter("max_rejects").as_int());

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_, 10);

        // 订阅 /robot/yaw (°)，用于 ICP 初始对齐
        yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("yaw_topic").as_string(),
            rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                // Float32 没有时间戳，按接收时刻打戳存历史，处理扫描时按扫描时间戳查
                std::lock_guard<std::mutex> lk(yaw_mtx_);
                yaw_hist_.emplace_back(now().seconds(), msg->data * (M_PI / 180.0));
                while (yaw_hist_.size() > 2 &&
                       yaw_hist_.back().first - yaw_hist_.front().first > 5.0) {
                    yaw_hist_.pop_front();
                }
            });

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
    //
    // scan-to-keyframe：当前帧和关键帧匹配，而不是和上一帧。
    //   - 某帧 ICP 被拒时位姿先用 IMU yaw 顶上（平移不动），
    //     下一帧仍对同一关键帧匹配，被拒帧期间的真实位移会被找回，不会永久丢失
    //   - ICP 在激光系下求解，结果用外参 T_bl 换算到 base 系：
    //     T_kf_base = T_bl * T_kf_laser * T_bl^-1
    //     雷达不在旋转中心时，原地转也会让激光系产生平移，不换算会积分出假位移
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

            if (!have_extrinsic_ && !lookup_extrinsic(scan->header.frame_id)) continue;

            // 取扫描时刻的 IMU yaw（处理可能落后扫描很多，不能用最新值）
            double cur_yaw = 0.0;
            const bool imu_ok = yaw_at(rclcpp::Time(scan->header.stamp).seconds(), cur_yaw);

            if (!kf_scan_) {
                kf_scan_      = scan;
                T_odom_kf_    = pose_;
                prev_yaw_rad_ = cur_yaw;
                prev_imu_ok_  = imu_ok;
                enqueue_odom(scan->header.stamp);
                continue;
            }

            // IMU 第一次到达时没有上一帧可比，本帧先用上一帧的 yaw 增量，避免绝对 yaw 被当成增量
            double dyaw_pred = pose_yaw(last_delta_);
            if (imu_ok && prev_imu_ok_) {
                dyaw_pred = std::atan2(std::sin(cur_yaw - prev_yaw_rad_),
                                       std::cos(cur_yaw - prev_yaw_rad_));
            }
            prev_yaw_rad_ = cur_yaw;
            prev_imu_ok_  = imu_ok;

            // 预测：只用 IMU 旋转，平移不外推。
            // 匀速外推在静止时会正反馈：ICP 噪声被当成速度外推，ICP 又收敛到外推初值附近，
            // 实测静止 15s 漂 0.3m。帧间平移只有几 cm，ICP 从零初值就能收敛
            const Pose2 pred_delta = make_pose(0.0, 0.0, dyaw_pred);
            const Pose2 T_pred        = pose_ * pred_delta;
            const Pose2 T_kf_base_ini = T_odom_kf_.inverse() * T_pred;

            bool  accepted = false;
            Pose2 T_kf_base = T_kf_base_ini;
            try {
                const Pose2 init_laser = T_bl_.inverse() * T_kf_base_ini * T_bl_;
                bool enough = false;
                const Pose2 T_kf_laser = matcher_.match(*kf_scan_, *scan, init_laser, enough);
                if (enough) {
                    T_kf_base = T_bl_ * T_kf_laser * T_bl_.inverse();
                    // 门限：ICP 结果相对预测的偏差。连续被拒时预测误差会累积，门限适度放宽，
                    // 但封顶 2 倍：放得太宽会把匹配到坏关键帧的结果放进来
                    const Pose2  err   = T_kf_base_ini.inverse() * T_kf_base;
                    const double scale = std::min(1.0 + rejects_, 2.0);
                    accepted = err.translation().norm() < max_trans_ * scale &&
                               std::abs(pose_yaw(err))  < max_rot_   * scale;
                    if (!accepted) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                            "ICP outlier rejected: err_t=%.3f err_yaw=%.3f (rejects=%d)",
                            err.translation().norm(), pose_yaw(err), rejects_ + 1);
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ICP: too few points");
                }
            } catch (const std::exception & e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ICP failed: %s", e.what());
            }

            if (accepted) {
                const Pose2 new_pose = T_odom_kf_ * T_kf_base;
                last_delta_ = pose_.inverse() * new_pose;
                pose_       = new_pose;
                rejects_    = 0;

                // 相对关键帧走得够远就换关键帧
                if (T_kf_base.translation().norm() > kf_trans_ ||
                    std::abs(pose_yaw(T_kf_base))  > kf_rot_) {
                    kf_scan_   = scan;
                    T_odom_kf_ = pose_;
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "ICP ok: pose x=%.3f y=%.3f yaw=%.3f",
                    pose_.translation().x(), pose_.translation().y(), pose_yaw(pose_));
            } else {
                // 回退只用 IMU 旋转、不外推平移：匀速外推在静止或匹配持续失败时会越推越远。
                // 被拒期间漏掉的平移，下一次对同一关键帧匹配成功时会补回来
                pose_       = pose_ * make_pose(0.0, 0.0, dyaw_pred);
                last_delta_ = make_pose(0.0, 0.0, dyaw_pred);
                if (++rejects_ > max_rejects_) {
                    // 长时间匹配不上（退化/遮挡），以预测位姿重置关键帧，避免一直卡死
                    RCLCPP_WARN(get_logger(), "ICP rejected %d frames in a row, reset keyframe", rejects_);
                    kf_scan_    = scan;
                    T_odom_kf_  = pose_;
                    last_delta_ = make_pose(0.0, 0.0, 0.0);
                    rejects_    = 0;
                }
            }
            enqueue_odom(scan->header.stamp);
        }
    }

    // 按时间戳在 IMU 历史里线性插值 yaw；超出范围取端点
    bool yaw_at(double t, double & yaw)
    {
        std::lock_guard<std::mutex> lk(yaw_mtx_);
        if (yaw_hist_.empty()) return false;
        if (t <= yaw_hist_.front().first) { yaw = yaw_hist_.front().second; return true; }
        if (t >= yaw_hist_.back().first)  { yaw = yaw_hist_.back().second;  return true; }
        auto it = std::lower_bound(yaw_hist_.begin(), yaw_hist_.end(), t,
            [](const std::pair<double, double> & a, double v) { return a.first < v; });
        const auto & b = *it;
        const auto & a = *(it - 1);
        const double r = (t - a.first) / std::max(b.first - a.first, 1e-9);
        const double d = std::atan2(std::sin(b.second - a.second), std::cos(b.second - a.second));
        yaw = a.second + r * d;
        return true;
    }

    // 查询 base ← laser 外参，只取平面分量 (x, y, yaw)
    bool lookup_extrinsic(const std::string & laser_frame)
    {
        try {
            const auto tf = tf_buffer_->lookupTransform(
                base_frame_, laser_frame, tf2::TimePointZero);
            const auto & q = tf.transform.rotation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            T_bl_ = make_pose(tf.transform.translation.x, tf.transform.translation.y, yaw);
            have_extrinsic_ = true;
            RCLCPP_INFO(get_logger(), "extrinsic %s <- %s: x=%.3f y=%.3f yaw=%.3f",
                base_frame_.c_str(), laser_frame.c_str(),
                T_bl_.translation().x(), T_bl_.translation().y(), pose_yaw(T_bl_));
            return true;
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "waiting for TF %s <- %s: %s", base_frame_.c_str(), laser_frame.c_str(), e.what());
            return false;
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

    // 把当前位姿打包成 Odometry 放入待发布容器
    void enqueue_odom(const rclcpp::Time & stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose_yaw(pose_));

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id  = base_frame_;
        msg.pose.pose.position.x    = pose_.translation().x();
        msg.pose.pose.position.y    = pose_.translation().y();
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
    double max_trans_{0.10};
    double max_rot_{0.30};
    double kf_trans_{0.30};
    double kf_rot_{0.35};
    int    max_rejects_{10};

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr      yaw_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer>                             tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener>                  tf_listener_;

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

    // 关键帧与位姿（仅 proc_thread_ 读写，无需额外锁）
    sensor_msgs::msg::LaserScan::SharedPtr kf_scan_;
    Pose2 T_bl_        = Pose2::Identity();   // base <- laser 外参
    Pose2 T_odom_kf_   = Pose2::Identity();   // 关键帧时刻的 base 位姿
    Pose2 pose_        = Pose2::Identity();   // 当前 base 位姿
    Pose2 last_delta_  = Pose2::Identity();   // 上一帧 base 增量（无 IMU 时用它的 yaw 做预测）
    bool  have_extrinsic_{false};
    int   rejects_{0};

    // yaw 先验（来自 /robot/yaw），yaw_hist_ 受 yaw_mtx_ 保护
    std::mutex yaw_mtx_;
    std::deque<std::pair<double, double>> yaw_hist_;   // (接收时刻 s, yaw rad)
    double     prev_yaw_rad_{0.0};
    bool       prev_imu_ok_{false};   // 上一帧扫描时是否已有 IMU（仅 proc_thread_ 用）

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
