// scan_matcher_2d 单元测试：合成房间地图 + 光线投射 scan，从扰动初值收敛回真值
#include "../src/map_localizer/scan_matcher_2d.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace map_localizer;

namespace {

constexpr double RES = 0.05;
constexpr int W = 200, H = 160;   // 10m x 8m
constexpr double OX = -5.0, OY = -4.0;

// 外墙 + 两个箱子 + 一段内墙，避免对称
std::vector<int8_t> make_map() {
    std::vector<int8_t> m(W * H, 0);
    auto box = [&](double x0, double y0, double x1, double y1) {
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const double wx = OX + (x + 0.5) * RES, wy = OY + (y + 0.5) * RES;
                if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) m[y * W + x] = 100;
            }
    };
    box(-5, -4, 5, -3.9); box(-5, 3.9, 5, 4); box(-5, -4, -4.9, 4); box(4.9, -4, 5, 4);
    box(1.0, 1.0, 1.6, 1.4); box(-3.0, -2.5, -2.2, -2.0); box(-1.0, 1.5, -0.9, 3.9);
    return m;
}

bool occ(const std::vector<int8_t>& m, double wx, double wy) {
    const int x = int(std::floor((wx - OX) / RES)), y = int(std::floor((wy - OY) / RES));
    return x < 0 || y < 0 || x >= W || y >= H || m[y * W + x] >= 65;
}

std::vector<Eigen::Vector2d> raycast(const std::vector<int8_t>& m, const Pose2& p, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, 0.015);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<Eigen::Vector2d> pts;
    for (int i = 0; i < 360; ++i) {
        const double a = -M_PI + i * 2 * M_PI / 360;
        for (double r = 0.1; r < 8.0; r += RES * 0.5) {
            if (occ(m, p.x + r * std::cos(p.yaw + a), p.y + r * std::sin(p.yaw + a))) {
                if (u(rng) < 0.1) break;          // 10% 丢点
                const double rr = r + noise(rng);
                pts.emplace_back(rr * std::cos(a), rr * std::sin(a));
                break;
            }
        }
    }
    return pts;
}

}  // namespace

TEST(ScanMatcher2D, PoseAlgebra) {
    const Pose2 a{1.0, 2.0, 0.7}, b{-0.3, 0.5, -1.2};
    const Pose2 r = compose(compose(a, b), inverse(b));
    EXPECT_NEAR(r.x, a.x, 1e-9);
    EXPECT_NEAR(r.y, a.y, 1e-9);
    EXPECT_NEAR(r.yaw, a.yaw, 1e-9);
}

TEST(ScanMatcher2D, DistanceFieldExact) {
    auto m = make_map();
    DistanceField df;
    df.build(m, W, H, RES, OX, OY, 65, 1.0);
    // 房间中心离箱子 (1.0~1.6, 1.0~1.4) 最近，(0,0) 到 (1.0,1.0) 角点 ≈ 1.41 → 截断 1.0
    EXPECT_NEAR(df.eval(0.0, 0.0), 1.0, 1e-6);
    // 下墙 y=-3.9 上方 0.3m
    EXPECT_NEAR(df.eval(0.0, -3.6), 0.3, 0.05);
}

TEST(ScanMatcher2D, ConvergesFromPerturbedPrediction) {
    auto m = make_map();
    DistanceField df;
    df.build(m, W, H, RES, OX, OY);
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> ux(-3.5, 3.5), uy(-3.0, 3.0), uyaw(-M_PI, M_PI), up(-1, 1);

    int ok = 0, trials = 0;
    while (trials < 30) {
        const Pose2 gt{ux(rng), uy(rng), uyaw(rng)};
        if (occ(m, gt.x, gt.y) || df.eval(gt.x, gt.y) < 0.3) continue;
        ++trials;
        const auto pts = raycast(m, gt, rng);
        // 预测误差：里程计 0.1s 内的典型漂移量级再放大
        const Pose2 pred{gt.x + 0.15 * up(rng), gt.y + 0.15 * up(rng), gt.yaw + 0.08 * up(rng)};
        const auto r = match_scan(df, pts, pred);
        const double et = std::hypot(r.pose.x - gt.x, r.pose.y - gt.y);
        const double ey = std::abs(wrap_angle(r.pose.yaw - gt.yaw));
        if (et < 0.03 && ey < 0.01 && r.inlier_ratio > 0.8) ++ok;
        else ADD_FAILURE() << "trial " << trials << " et=" << et << " ey=" << ey << " inl=" << r.inlier_ratio;
    }
    EXPECT_GE(ok, 28);
}

TEST(ScanMatcher2D, WrongPredictionScoresLow) {
    auto m = make_map();
    DistanceField df;
    df.build(m, W, H, RES, OX, OY);
    std::mt19937 rng(2);
    const Pose2 gt{0.5, -1.0, 0.3};
    const auto pts = raycast(m, gt, rng);
    // 偏 1.5m / 60°：远超收敛域，评分应低于门限，节点据此拒绝
    const auto r = match_scan(df, pts, Pose2{2.0, 0.5, 1.35});
    EXPECT_LT(r.inlier_ratio, 0.5);
}
