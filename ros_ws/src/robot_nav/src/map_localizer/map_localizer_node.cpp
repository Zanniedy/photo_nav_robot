// map_localizer_node — 2D scan-to-map 定位，替代 AMCL 发布 map→odom
//
// 每帧 scan（工作线程）：
//   1. 预测：T_map_base = T_map_odom · T_odom_base(t_scan)，odom 增量来自 lio TF
//   2. 匹配：scan 点（base 系）在地图距离场上 Gauss-Newton 精配（scan_matcher_2d.hpp）
//   3. 把关：内点率 >= min_inlier_ratio 且修正量不超限才接受，否则保持预测（只靠 odom）
//   4. 更新 T_map_odom = T_map_base · T_odom_base(t)^-1
// TF 由定时器按 tf_rate 发布，时间戳前移 tf_future 秒（同 AMCL transform_tolerance 的做法），
// 匹配线程卡顿不影响 nav2 查 TF。
//
// 初值：/initialpose（由 relocalize_node 精配后发布，或参数 initial_pose_*）
// 输出：/localizer/pose (PoseWithCovarianceStamped)、/localizer/score (Float32 内点率)

#include "scan_matcher_2d.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace map_localizer {

template <typename Q>
double yaw_of(const Q& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class MapLocalizerNode : public rclcpp::Node {
public:
    explicit MapLocalizerNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{})
        : Node("map_localizer_node", opts)
    {
        map_frame_  = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "lio_odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
        tf_rate_    = declare_parameter<double>("tf_rate", 20.0);
        tf_future_  = declare_parameter<double>("tf_future", 0.2);    // s
        max_points_ = declare_parameter<int>("max_points", 360);
        min_inlier_ = declare_parameter<double>("min_inlier_ratio", 0.5);
        max_corr_t_ = declare_parameter<double>("max_correction_trans", 0.3);   // m/帧
        max_corr_r_ = declare_parameter<double>("max_correction_rot", 0.17);  // rad/帧 (~10°)
        lost_after_ = declare_parameter<int>("lost_after_rejects", 20);    // 连续拒绝 ~2s 报丢失
        mp_.max_iters   = declare_parameter<int>("max_iters", 15);
        mp_.huber_delta = declare_parameter<double>("huber_delta", 0.10);
        mp_.prior_xy    = declare_parameter<double>("prior_xy", 2.0);
        mp_.prior_yaw   = declare_parameter<double>("prior_yaw", 2.0);
        mp_.inlier_dist = declare_parameter<double>("inlier_dist", 0.10);
        max_dist_       = declare_parameter<double>("field_max_dist", 0.5);
        {
            // 未收到 /initialpose 前假定从这里出发（与 AMCL set_initial_pose 同义）
            Pose2 init{declare_parameter<double>("initial_pose_x", 0.0),
                       declare_parameter<double>("initial_pose_y", 0.0),
                       declare_parameter<double>("initial_pose_yaw", 0.0)};
            pending_init_ = init;
            has_pending_init_ = true;
        }

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_pub_      = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto map_qos = rclcpp::QoS(1).transient_local().reliable();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { on_map(*m); });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr s) {
                { std::lock_guard<std::mutex> lk(scan_mtx_); latest_scan_ = s; }   // 只保留最新帧，处理不过来就跳帧
                scan_cv_.notify_one();
            });
        init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                if (!m->header.frame_id.empty() && m->header.frame_id != map_frame_) {
                    RCLCPP_WARN(get_logger(), "忽略 frame=%s 的初始位姿，需要 %s",
                                m->header.frame_id.c_str(), map_frame_.c_str());
                    return;
                }
                const auto& p = m->pose.pose;
                std::lock_guard<std::mutex> lk(state_mtx_);
                pending_init_ = {p.position.x, p.position.y, yaw_of(p.orientation)};
                has_pending_init_ = true;
                RCLCPP_INFO(get_logger(), "收到初始位姿 (%.2f, %.2f, %.1f°)",
                            pending_init_.x, pending_init_.y, pending_init_.yaw * 180 / M_PI);
            });

        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localizer/pose", 10);
        score_pub_ = create_publisher<std_msgs::msg::Float32>("/localizer/score", 10);

        tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_rate_), [this] { publish_tf(); });

        running_ = true;
        worker_  = std::thread(&MapLocalizerNode::worker_loop, this);
        RCLCPP_INFO(get_logger(), "等待 /map；map→%s 由本节点发布（不要同时启动 AMCL）", odom_frame_.c_str());
    }

    ~MapLocalizerNode() override {
        running_ = false;
        scan_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

private:
    void on_map(const nav_msgs::msg::OccupancyGrid& m) {
        auto df = std::make_shared<DistanceField>();
        df->build(m.data, m.info.width, m.info.height, m.info.resolution,
                  m.info.origin.position.x, m.info.origin.position.y, 65, max_dist_);
        std::atomic_store(&field_, std::shared_ptr<const DistanceField>(df));
        RCLCPP_INFO(get_logger(), "地图已加载 %ux%u @ %.3fm，距离场构建完成",
                    m.info.width, m.info.height, m.info.resolution);
    }

    bool lookup(const std::string& target, const std::string& source, const rclcpp::Time& t,
                Pose2& out, double timeout_s) {
        try {
            const auto tf = tf_buffer_->lookupTransform(target, source, t,
                                                       rclcpp::Duration::from_seconds(timeout_s));
            out = {tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)};
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF %s<-%s: %s",
                                 target.c_str(), source.c_str(), e.what());
            return false;
        }
    }

    void worker_loop() {
        while (running_) {
            sensor_msgs::msg::LaserScan::SharedPtr scan;
            {
                std::unique_lock<std::mutex> lk(scan_mtx_);
                scan_cv_.wait(lk, [this] { return latest_scan_ || !running_; });
                if (!running_) break;
                scan = std::move(latest_scan_);
            }
            process(*scan);
        }
    }

    void process(const sensor_msgs::msg::LaserScan& scan) {
        auto df = std::atomic_load(&field_);
        if (!df) return;
        const rclcpp::Time stamp(scan.header.stamp);

        // lio TF 比 scan 晚 0.1~0.3s 才到，最多等 0.5s
        Pose2 T_odom_base, T_base_laser;
        if (!lookup(odom_frame_, base_frame_, stamp, T_odom_base, 0.5)) return;
        if (!lookup(base_frame_, scan.header.frame_id, rclcpp::Time(0), T_base_laser, 0.1)) return;

        // scan → base 系点，均匀抽稀到 max_points
        std::vector<Eigen::Vector2d> pts;
        const int n = static_cast<int>(scan.ranges.size());
        int valid = 0;
        for (float r : scan.ranges) valid += std::isfinite(r) && r > scan.range_min && r < scan.range_max;
        const double step = std::max(1.0, double(valid) / std::max<int64_t>(1, max_points_));
        double acc = 0.0;
        pts.reserve(std::min<int64_t>(valid, max_points_) + 1);
        for (int i = 0; i < n; ++i) {
            const float r = scan.ranges[i];
            if (!(std::isfinite(r) && r > scan.range_min && r < scan.range_max)) continue;
            if ((acc += 1.0) < step) continue;
            acc -= step;
            const double a = scan.angle_min + i * scan.angle_increment;
            const Pose2 pl{r * std::cos(a), r * std::sin(a), 0.0};
            const Pose2 pb = compose(T_base_laser, pl);
            pts.emplace_back(pb.x, pb.y);
        }
        if (pts.size() < 30) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "有效激光点太少 (%zu)", pts.size());
            return;
        }

        Pose2 T_map_odom;
        bool from_init = false;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (has_pending_init_) {
                // 初值给的是 map 系下的 base 位姿
                T_map_odom = compose(pending_init_, inverse(T_odom_base));
                has_pending_init_ = false;
                from_init = true;
            } else {
                T_map_odom = T_map_odom_;
            }
        }

        const Pose2 pred = compose(T_map_odom, T_odom_base);
        const auto t0 = std::chrono::steady_clock::now();
        const MatchResult r = match_scan(*df, pts, pred, mp_);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        const double dt = std::hypot(r.pose.x - pred.x, r.pose.y - pred.y);
        const double dr = std::abs(wrap_angle(r.pose.yaw - pred.yaw));
        const bool accept = r.inlier_ratio >= min_inlier_ && dt <= max_corr_t_ && dr <= max_corr_r_;

        Pose2 T_map_base = pred;
        if (accept) {
            T_map_base = r.pose;
            T_map_odom = compose(T_map_base, inverse(T_odom_base));
            if (rejects_ >= lost_after_)
                RCLCPP_INFO(get_logger(), "定位恢复：内点率 %.2f", r.inlier_ratio);
            rejects_ = 0;
        } else {
            ++rejects_;
            if (rejects_ == lost_after_)
                RCLCPP_WARN(get_logger(), "连续 %d 帧匹配被拒（内点率 %.2f，修正 %.2fm/%.1f°），"
                            "只靠里程计；请在 RViz 重新给初始位姿", rejects_, r.inlier_ratio, dt, dr * 180 / M_PI);
        }
        RCLCPP_DEBUG(get_logger(), "inl=%.2f corr=%.3fm/%.2f° iters=%d used=%d %.2fms %s",
                     r.inlier_ratio, dt, dr * 180 / M_PI, r.iters, r.used_points, ms, accept ? "OK" : "REJECT");

        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (!has_pending_init_) {   // 处理期间来了新初值则以新初值为准
                T_map_odom_ = T_map_odom;
                tf_valid_ = true;
            }
            if (from_init && !accept)
                RCLCPP_WARN(get_logger(), "初始位姿处匹配失败（内点率 %.2f），先按初值走", r.inlier_ratio);
        }

        std_msgs::msg::Float32 score;
        score.data = static_cast<float>(r.inlier_ratio);
        score_pub_->publish(score);

        geometry_msgs::msg::PoseWithCovarianceStamped pm;
        pm.header.stamp = scan.header.stamp;
        pm.header.frame_id = map_frame_;
        pm.pose.pose.position.x = T_map_base.x;
        pm.pose.pose.position.y = T_map_base.y;
        pm.pose.pose.orientation.z = std::sin(T_map_base.yaw / 2);
        pm.pose.pose.orientation.w = std::cos(T_map_base.yaw / 2);
        // 粗略协方差：接受时取匹配精度量级，拒绝时随连续拒绝帧数放大
        const double sxy = accept ? 0.02 : 0.02 + 0.02 * rejects_;
        const double syaw = accept ? 0.01 : 0.01 + 0.01 * rejects_;
        pm.pose.covariance[0] = pm.pose.covariance[7] = sxy * sxy;
        pm.pose.covariance[35] = syaw * syaw;
        pose_pub_->publish(pm);
    }

    void publish_tf() {
        Pose2 T;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (!tf_valid_) return;
            T = T_map_odom_;
        }
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now() + rclcpp::Duration::from_seconds(tf_future_);
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        tf.transform.translation.x = T.x;
        tf.transform.translation.y = T.y;
        tf.transform.rotation.z = std::sin(T.yaw / 2);
        tf.transform.rotation.w = std::cos(T.yaw / 2);
        tf_pub_->sendTransform(tf);
    }

    std::string map_frame_, odom_frame_, base_frame_;
    double tf_rate_, tf_future_, min_inlier_, max_corr_t_, max_corr_r_, max_dist_;
    int64_t max_points_, lost_after_;
    MatchParams mp_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr score_pub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    std::shared_ptr<const DistanceField> field_;   // atomic_load/store 访问

    std::mutex scan_mtx_;
    std::condition_variable scan_cv_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    std::mutex state_mtx_;                         // 保护下面四个
    Pose2 T_map_odom_;
    bool  tf_valid_{false};
    Pose2 pending_init_;
    bool  has_pending_init_{false};

    int rejects_{0};                               // 仅 worker 线程
    std::atomic<bool> running_{false};
    std::thread worker_;
};

}  // namespace map_localizer

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_localizer::MapLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
