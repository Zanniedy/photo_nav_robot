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
// 输出：/localizer/pose (PoseWithCovarianceStamped)、/localizer/score (Float32 内点率)、
//       /localizer/state (String JSON，2Hz：state/score/rejects/dynamic/recover)
//
// 动态点：匹配前用 dynamic_filter.hpp 去掉帧间移动快的点（走动的人），不参与匹配和评分
//
// 状态机：
//   tracking ──连续 lost_after_rejects 帧被拒、且这串帧平均内点率 < lost_score──▶ lost ──▶ recovering
//   （只是修正量超限、内点率还行的被拒不算丢：那是 odom 和地图差了一点，靠 odom 走着等下一帧）
//   recovering：每帧先按预测做一次普通匹配，内点率 >= recover_score 直接回 tracking（短暂遮挡）；
//               否则每 recover_interval 秒从最后可信位姿一圈圈往外搜（global_search.hpp），
//               连续两次找到的位姿一致（recover_agree_*）且内点率 >= recover_score 才采用 → tracking
//   收到 /initialpose 任何状态都回 tracking
// 服务 /localizer/relocalize (std_srvs/Trigger)：手动进入 recovering

#include "../common/dynamic_filter.hpp"
#include "global_search.hpp"
#include "scan_matcher_2d.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
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
        // 10-26 现场正常跟踪内点率 0.42~0.58：0.5 太紧，动起来就连续被拒
        min_inlier_ = declare_parameter<double>("min_inlier_ratio", 0.4);
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
            dynfilter::Params dp;
            dp.enable    = declare_parameter<bool>("dyn_enable", true);
            dp.speed     = declare_parameter<double>("dyn_speed", 0.5);       // m/s
            dp.margin    = declare_parameter<double>("dyn_margin", 0.06);
            dp.ref_dt    = declare_parameter<double>("dyn_ref_dt", 0.3);
            dp.max_ratio = declare_parameter<double>("dyn_max_ratio", 0.3);
            dyn_.set_params(dp);
        }
        auto_recover_     = declare_parameter<bool>("auto_recover", true);
        recover_score_    = declare_parameter<double>("recover_score", 0.5);   // 实测正常跟踪 0.42~0.58
        lost_score_       = declare_parameter<double>("lost_score", 0.3);      // 被拒这串帧平均内点率低于它才算丢
        recover_interval_ = declare_parameter<double>("recover_interval", 1.0);
        agree_t_          = declare_parameter<double>("recover_agree_trans", 0.3);
        agree_r_          = declare_parameter<double>("recover_agree_rot", 0.17);
        sp_.accept_score  = recover_score_;
        sp_.xy_step       = declare_parameter<double>("search_xy_step", 0.15);
        sp_.yaw_step      = declare_parameter<double>("search_yaw_step_deg", 5.0) * M_PI / 180.0;
        {
            const auto radii = declare_parameter<std::vector<double>>("search_radii",
                std::vector<double>{0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 40.0});
            if (!radii.empty()) sp_.radii = radii;
        }
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
                force_recover_ = false;
                RCLCPP_INFO(get_logger(), "收到初始位姿 (%.2f, %.2f, %.1f°)",
                            pending_init_.x, pending_init_.y, pending_init_.yaw * 180 / M_PI);
            });

        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localizer/pose", 10);
        score_pub_ = create_publisher<std_msgs::msg::Float32>("/localizer/score", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/localizer/state", 10);
        reloc_srv_ = create_service<std_srvs::srv::Trigger>("/localizer/relocalize",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                force_recover_ = true;
                res->success = true;
                res->message = "进入 recovering：从当前位置往外搜";
            });
        state_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_state(); });

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

        // scan → base 系点（扫描顺序）→ 去动态点 → 均匀抽稀到 max_points
        std::vector<Eigen::Vector2d> all;
        const int n = static_cast<int>(scan.ranges.size());
        all.reserve(n);
        for (int i = 0; i < n; ++i) {
            const float r = scan.ranges[i];
            if (!(std::isfinite(r) && r > scan.range_min && r < scan.range_max)) continue;
            const double a = scan.angle_min + i * scan.angle_increment;
            const Pose2 pb = compose(T_base_laser, Pose2{r * std::cos(a), r * std::sin(a), 0.0});
            all.emplace_back(pb.x, pb.y);
        }
        const Pose2 d = has_prev_odom_ ? compose(inverse(prev_odom_base_), T_odom_base) : Pose2{};
        Eigen::Isometry2d T_prev_cur = Eigen::Isometry2d::Identity();
        T_prev_cur.translate(Eigen::Vector2d(d.x, d.y));
        T_prev_cur.rotate(d.yaw);
        prev_odom_base_ = T_odom_base;
        has_prev_odom_ = true;
        dynfilter::Stats dst;
        const auto dyn = dyn_.update(all, stamp.seconds(), T_prev_cur, &dst);
        dyn_ratio_ = dst.total ? double(dst.dynamic) / dst.total : 0.0;
        int valid = dst.total - dst.dynamic;
        std::vector<Eigen::Vector2d> pts;
        const double step = std::max(1.0, double(valid) / std::max<int64_t>(1, max_points_));
        double acc = 0.0;
        pts.reserve(std::min<int64_t>(valid, max_points_) + 1);
        for (size_t i = 0; i < all.size(); ++i) {
            if (dyn[i]) continue;
            if ((acc += 1.0) < step) continue;
            acc -= step;
            pts.push_back(all[i]);
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
        if (from_init) set_state("tracking", "收到初始位姿");
        if (force_recover_.exchange(false) && !from_init) {
            rejects_ = lost_after_;
            set_state("recovering", "手动重定位");
            last_search_ = 0.0;
        }

        const Pose2 pred = compose(T_map_odom, T_odom_base);
        const auto t0 = std::chrono::steady_clock::now();
        const MatchResult r = match_scan(*df, pts, pred, mp_);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        const double dt = std::hypot(r.pose.x - pred.x, r.pose.y - pred.y);
        const double dr = std::abs(wrap_angle(r.pose.yaw - pred.yaw));
        bool accept = r.inlier_ratio >= min_inlier_ && dt <= max_corr_t_ && dr <= max_corr_r_;
        // 恢复中：普通匹配要达到恢复门限才算回来；否则走外扩搜索
        Pose2 recovered;
        double rec_score = 0.0;
        if (state_ == "recovering") {
            if (accept && r.inlier_ratio >= recover_score_) {
                set_state("tracking", "局部匹配恢复，内点率 " + fmt(r.inlier_ratio));
            } else {
                accept = false;
                if (try_recover(*df, pts, pred, stamp.seconds(), recovered, rec_score)) {
                    T_map_odom = compose(recovered, inverse(T_odom_base));
                    rejects_ = 0;
                    set_state("tracking", "外扩搜索恢复，内点率 " + fmt(rec_score));
                    commit(T_map_odom, recovered, true, rec_score, scan);
                    return;
                }
            }
        }

        Pose2 T_map_base = pred;
        if (accept) {
            T_map_base = r.pose;
            T_map_odom = compose(T_map_base, inverse(T_odom_base));
            rejects_ = 0;
            reject_inl_sum_ = 0.0;
            last_good_ = T_map_base;
            has_good_ = true;
        } else {
            ++rejects_;
            reject_inl_sum_ += r.inlier_ratio;
            const double mean_inl = reject_inl_sum_ / rejects_;
            if (rejects_ >= lost_after_ && state_ == "tracking" && mean_inl >= lost_score_) {
                // 内点率还行，只是修正量超限：不算丢，重新数（避免一直累积到很大）
                rejects_ = 0;
                reject_inl_sum_ = 0.0;
            } else if (rejects_ >= lost_after_ && state_ == "tracking") {
                RCLCPP_WARN(get_logger(), "连续 %d 帧匹配被拒（内点率 %.2f，修正 %.2fm/%.1f°），只靠里程计",
                            rejects_.load(), r.inlier_ratio, dt, dr * 180 / M_PI);
                set_state("lost", "连续 " + std::to_string(rejects_.load()) + " 帧被拒，平均内点率 " + fmt(mean_inl));
                if (auto_recover_) {
                    set_state("recovering", "从最后可信位置往外搜");
                    last_search_ = 0.0;
                    agree_n_ = 0;
                }
            }
        }
        RCLCPP_DEBUG(get_logger(), "inl=%.2f corr=%.3fm/%.2f° iters=%d used=%d dyn=%d %.2fms %s",
                     r.inlier_ratio, dt, dr * 180 / M_PI, r.iters, r.used_points, dst.dynamic, ms,
                     accept ? "OK" : "REJECT");
        commit(T_map_odom, T_map_base, accept, r.inlier_ratio, scan, from_init);
    }

    void commit(const Pose2& T_map_odom, const Pose2& T_map_base, bool accept, double inlier,
                const sensor_msgs::msg::LaserScan& scan, bool from_init = false) {
        score_ = inlier;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (!has_pending_init_) {   // 处理期间来了新初值则以新初值为准
                T_map_odom_ = T_map_odom;
                tf_valid_ = true;
            }
            if (from_init && !accept)
                RCLCPP_WARN(get_logger(), "初始位姿处匹配失败（内点率 %.2f），先按初值走", inlier);
        }

        std_msgs::msg::Float32 score;
        score.data = static_cast<float>(inlier);
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

    // 恢复：每 recover_interval 秒做一次外扩搜索；连续两次结果一致才采用
    bool try_recover(const DistanceField& df, const std::vector<Eigen::Vector2d>& pts, const Pose2& pred,
                     double t, Pose2& out, double& score) {
        if (t - last_search_ < recover_interval_) return false;
        last_search_ = t;
        const Pose2 center = has_good_ ? last_good_ : pred;
        const auto t0 = std::chrono::steady_clock::now();
        const SearchResult sr = expanding_search(df, pts, agree_n_ ? cand_ : center, sp_, mp_);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        last_search_score_ = sr.score;
        last_search_radius_ = sr.radius;
        RCLCPP_INFO(get_logger(), "外扩搜索：%s 内点率 %.2f，搜到 %.1fm（%d 圈，%ld 个位姿，%.0fms）",
                    sr.found ? "找到" : "没找到", sr.score, sr.radius, sr.rings, sr.evals, ms);
        if (!sr.found) { agree_n_ = 0; return false; }
        if (agree_n_ > 0 && std::hypot(sr.pose.x - cand_.x, sr.pose.y - cand_.y) <= agree_t_ &&
            std::abs(wrap_angle(sr.pose.yaw - cand_.yaw)) <= agree_r_) {
            agree_n_ = 0;
            out = sr.pose;
            score = sr.score;
            last_good_ = sr.pose;
            has_good_ = true;
            return true;
        }
        cand_ = sr.pose;              // 第一次找到：下一次在它附近确认
        agree_n_ = 1;
        last_search_ = t - recover_interval_ * 0.7;   // 确认搜索早一点做
        return false;
    }

    void set_state(const std::string& s, const std::string& why) {
        std::lock_guard<std::mutex> lk(fsm_mtx_);
        if (s == state_) return;
        RCLCPP_INFO(get_logger(), "定位状态 %s → %s：%s", state_.c_str(), s.c_str(), why.c_str());
        state_ = s;
        state_why_ = why;
    }

    void publish_state() {
        std::string st, why;
        { std::lock_guard<std::mutex> lk(fsm_mtx_); st = state_; why = state_why_; }
        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "{\"state\":\"%s\",\"score\":%.3f,\"rejects\":%d,\"dynamic\":%.3f,"
            "\"recover\":{\"score\":%.3f,\"radius\":%.1f,\"confirming\":%s},\"why\":\"%s\"}",
            st.c_str(), score_.load(), rejects_.load(), dyn_ratio_.load(), last_search_score_.load(),
            last_search_radius_.load(), agree_n_ ? "true" : "false", why.c_str());
        std_msgs::msg::String m;
        m.data = buf;
        state_pub_->publish(m);
    }

    static std::string fmt(double v) { char b[16]; std::snprintf(b, sizeof(b), "%.2f", v); return b; }

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
    rclcpp::TimerBase::SharedPtr tf_timer_, state_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reloc_srv_;

    // 动态点、恢复状态机（worker 线程写；state_ 由 fsm_mtx_ 保护，数值用 atomic 给状态定时器读）
    dynfilter::DynamicFilter dyn_;
    Pose2 prev_odom_base_;
    bool  has_prev_odom_{false};
    SearchParams sp_;
    bool   auto_recover_{true};
    double recover_score_{0.5}, recover_interval_{1.0}, agree_t_{0.3}, agree_r_{0.17}, lost_score_{0.3};
    double reject_inl_sum_{0.0};                   // 仅 worker 线程
    std::mutex  fsm_mtx_;
    std::string state_{"tracking"}, state_why_{"启动"};
    std::atomic<bool> force_recover_{false};
    Pose2  last_good_, cand_;
    bool   has_good_{false};
    std::atomic<int> agree_n_{0};
    double last_search_{0.0};
    std::atomic<double> score_{0.0}, dyn_ratio_{0.0}, last_search_score_{0.0}, last_search_radius_{0.0};

    std::shared_ptr<const DistanceField> field_;   // atomic_load/store 访问

    std::mutex scan_mtx_;
    std::condition_variable scan_cv_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    std::mutex state_mtx_;                         // 保护下面四个
    Pose2 T_map_odom_;
    bool  tf_valid_{false};
    Pose2 pending_init_;
    bool  has_pending_init_{false};

    std::atomic<int> rejects_{0};                  // worker 线程写
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
