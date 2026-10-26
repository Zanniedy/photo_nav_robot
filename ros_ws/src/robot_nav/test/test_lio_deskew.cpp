// 扫描去畸变测试：合成房间里边转边走地采一帧（每个点在自己的时刻采），
// 和静止时采的参考帧做 ICP，比较去畸变前后的误差
#include "../src/lio/lio_math.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>

#include <limits>

using namespace lioslam;

namespace {

struct Seg { Eigen::Vector2d a, b; };

// 6m x 4m 房间 + 两个箱子 + 一段内墙，避免对称
std::vector<Seg> room() {
    std::vector<Seg> s;
    auto box = [&](double x0, double y0, double x1, double y1) {
        s.push_back({{x0, y0}, {x1, y0}}); s.push_back({{x1, y0}, {x1, y1}});
        s.push_back({{x1, y1}, {x0, y1}}); s.push_back({{x0, y1}, {x0, y0}});
    };
    box(-3, -2, 3, 2);
    box(1.0, 0.8, 1.5, 1.2);
    box(-2.0, -1.5, -1.4, -1.1);
    s.push_back({{-0.5, 2.0}, {-0.5, 0.9}});
    return s;
}

double cast(const std::vector<Seg>& w, const Eigen::Vector2d& o, double ang) {
    const Eigen::Vector2d d(std::cos(ang), std::sin(ang));
    double best = std::numeric_limits<double>::infinity();
    for (const auto& s : w) {
        const Eigen::Vector2d e = s.b - s.a;
        const double den = d.x() * e.y() - d.y() * e.x();
        if (std::abs(den) < 1e-12) continue;
        const Eigen::Vector2d q = s.a - o;
        const double t = (q.x() * e.y() - q.y() * e.x()) / den;
        const double u = (q.x() * d.y() - q.y() * d.x()) / den;
        if (t > 0.0 && u >= 0.0 && u <= 1.0) best = std::min(best, t);
    }
    return best;
}

// 世界系雷达位姿 T_w_l(t)：车体从 T_w_b0 出发按 (vx, wz) 匀速运动，雷达外参 T_bl
// 下标 i 的采集时刻按 sllidar 顺序（下标大先采）
sensor_msgs::msg::LaserScan make_scan(const Pose2& T_w_b0, const Pose2& T_bl,
                                      double vx, double wz, double period = 0.1, int n = 720) {
    sensor_msgs::msg::LaserScan sc;
    sc.angle_min = -M_PI; sc.angle_max = M_PI;
    sc.angle_increment = 2 * M_PI / (n - 1);
    sc.range_min = 0.15f; sc.range_max = 12.0f; sc.scan_time = period;
    sc.ranges.resize(n);
    Deskew mot; mot.vx = vx; mot.wz = wz; mot.period = period;
    const auto w = room();
    for (int i = 0; i < n; ++i) {
        const double t = mot.t_of(i, n);
        const Pose2 T_w_l = T_w_b0 * mot.base_motion(t) * T_bl;
        const double a = sc.angle_min + i * sc.angle_increment;
        sc.ranges[i] = static_cast<float>(cast(w, T_w_l.translation(), pose_yaw(T_w_l) + a));
    }
    return sc;
}

struct Err { double t, r; };

Err run(bool deskew) {
    const Pose2 T_bl = make_pose(0.25, 0.0, M_PI / 2);   // 与实机外参一致
    const Pose2 T_w_ref = make_pose(0.0, 0.0, 0.0);
    const Pose2 T_w_cur = make_pose(0.25, 0.05, 0.3);     // 当前帧首点时刻车体位姿
    const double vx = 0.35, wz = 1.0;

    scan_match m;
    Deskew none;
    Deskew dk; dk.enable = deskew; dk.T_bl = T_bl; dk.vx = vx; dk.wz = wz;
    const DP ref = m.make_points(make_scan(T_w_ref, T_bl, 0.0, 0.0), none);
    const DP cur = m.make_points(make_scan(T_w_cur, T_bl, vx, wz), dk);

    const Pose2 truth = (T_w_ref * T_bl).inverse() * (T_w_cur * T_bl);
    bool ok = false;
    const Pose2 init = truth * make_pose(0.03, -0.02, 0.02);   // 带点偏差的初值
    const Pose2 est = m.match(ref, cur, init, ok);
    EXPECT_TRUE(ok);
    const Pose2 e = truth.inverse() * est;
    return {e.translation().norm(), std::abs(pose_yaw(e))};
}

}  // namespace

TEST(Deskew, StaticIsIdentity) {
    Deskew d; d.enable = true; d.T_bl = make_pose(0.25, 0, M_PI / 2);
    const Eigen::Vector2d p(1.0, 2.0);
    EXPECT_LT((d.apply(p, 0.07) - p).norm(), 1e-12);
}

TEST(Deskew, PureRotationAboutBase) {
    // 车体原地转：雷达系里一个世界固定点的坐标应被还原到首点时刻
    Deskew d; d.enable = true; d.T_bl = make_pose(0.25, 0, M_PI / 2); d.wz = 1.0;
    const Eigen::Vector2d pw(2.0, 1.0);   // 首点时刻车体系 = 世界系
    const double t = 0.05;
    const Pose2 T_b0_lt = d.base_motion(t) * d.T_bl;
    const Eigen::Vector2d p_lt = T_b0_lt.inverse() * pw;     // t 时刻雷达看到的
    const Eigen::Vector2d p_l0 = d.T_bl.inverse() * pw;      // 首点时刻雷达系下应有的
    EXPECT_LT((d.apply(p_lt, t) - p_l0).norm(), 1e-9);
}

TEST(Deskew, ImprovesIcpWhileTurningAndDriving) {
    const Err raw = run(false), fix = run(true);
    std::printf("no deskew: %.3f m %.2f deg | deskew: %.3f m %.2f deg\n",
                raw.t, raw.r * 180 / M_PI, fix.t, fix.r * 180 / M_PI);
    EXPECT_LT(fix.t, 0.02);
    EXPECT_LT(fix.r, 0.5 * M_PI / 180);
    EXPECT_GT(raw.r, 2.0 * fix.r);
}

// 同一个关键帧连续匹配多帧：kd-tree 只建一次，结果和每次重建一样准；换关键帧才重建
TEST(ScanMatch, KeyframeTreeCached) {
    const Pose2 T_bl = make_pose(0.25, 0.0, M_PI / 2);
    scan_match m;
    Deskew none;
    const DP ref = m.make_points(make_scan(make_pose(0, 0, 0), T_bl, 0.0, 0.0), none);
    for (int k = 1; k <= 5; ++k) {
        const Pose2 T_w_cur = make_pose(0.05 * k, 0.01 * k, 0.03 * k);
        const DP cur = m.make_points(make_scan(T_w_cur, T_bl, 0.0, 0.0), none);
        const Pose2 truth = T_bl.inverse() * (T_w_cur * T_bl);
        bool ok = false;
        const Pose2 est = m.match(ref, cur, truth * make_pose(0.02, -0.02, 0.01), ok);
        ASSERT_TRUE(ok);
        const Pose2 e = truth.inverse() * est;
        EXPECT_LT(e.translation().norm(), 0.02);
        EXPECT_LT(std::abs(pose_yaw(e)), 0.01);
    }
    EXPECT_EQ(m.map_builds(), 1u);
    const DP ref2 = m.make_points(make_scan(make_pose(0.3, 0, 0.2), T_bl, 0.0, 0.0), none);
    bool ok = false;
    m.match(ref2, ref2, Pose2::Identity(), ok);
    EXPECT_EQ(m.map_builds(), 2u);
}

// 计时对比（打印结果）：同一个关键帧匹配 50 帧，旧写法每帧 ICP::operator() 重算 ref 法向量 + 建树，
// 新写法 ICPSequence 只建一次
TEST(ScanMatch, CachedTreeIsFaster) {
    const Pose2 T_bl = make_pose(0.25, 0.0, M_PI / 2);
    scan_match m;
    Deskew none;
    const DP ref = m.make_points(make_scan(make_pose(0, 0, 0), T_bl, 0.0, 0.0), none);
    std::vector<DP> cur;
    for (int k = 0; k < 50; ++k) cur.push_back(m.make_points(make_scan(make_pose(0.004 * k, 0.002 * k, 0.003 * k), T_bl, 0.0, 0.0), none));
    // 同一套配置：一个每次都 clearMap（旧行为），一个缓存
    scan_match fresh;
    const auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < 4; ++r)
        for (const auto& c : cur) { bool ok; fresh.match_rebuild(ref, c, Pose2::Identity(), ok); }
    const auto t1 = std::chrono::steady_clock::now();
    for (int r = 0; r < 4; ++r)
        for (const auto& c : cur) { bool ok; m.match(ref, c, Pose2::Identity(), ok); }
    const auto t2 = std::chrono::steady_clock::now();
    const double old_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / 200;
    const double new_ms = std::chrono::duration<double, std::milli>(t2 - t1).count() / 200;
    std::printf("[bench] ref=%ld cur=%ld 点  每帧重建 %.3fms  缓存 %.3fms  省 %.0f%%\n",
                static_cast<long>(ref.getNbPoints()), static_cast<long>(cur[0].getNbPoints()),
                old_ms, new_ms, 100.0 * (1.0 - new_ms / old_ms));
    EXPECT_LT(new_ms, old_ms);
}
