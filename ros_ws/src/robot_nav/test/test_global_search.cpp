// 重定位外扩搜索 + 动态点过滤 单元测试（合成房间地图 + 光线投射，和 test_scan_matcher_2d 同一套地图）
#include "../src/common/dynamic_filter.hpp"
#include "../src/map_localizer/global_search.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace map_localizer;

namespace {

constexpr double RES = 0.05;
constexpr int W = 200, H = 160;   // 10m x 8m
constexpr double OX = -5.0, OY = -4.0;

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
    std::vector<Eigen::Vector2d> pts;
    for (int i = 0; i < 360; ++i) {
        const double a = -M_PI + i * 2 * M_PI / 360;
        for (double r = 0.1; r < 8.0; r += RES * 0.5) {
            if (occ(m, p.x + r * std::cos(p.yaw + a), p.y + r * std::sin(p.yaw + a))) {
                const double rr = r + noise(rng);
                pts.emplace_back(rr * std::cos(a), rr * std::sin(a));
                break;
            }
        }
    }
    return pts;
}

DistanceField field() {
    DistanceField df;
    df.build(make_map(), W, H, RES, OX, OY, 65, 0.5);
    return df;
}

}  // namespace

// 位置差 0.4m、朝向差 120°：第一圈（0.5m）就该找回来
TEST(GlobalSearch, NearbyFoundInFirstRing) {
    std::mt19937 rng(1);
    const auto df = field();
    const Pose2 truth{0.5, -1.0, 0.3};
    const auto pts = raycast(make_map(), truth, rng);
    const SearchResult r = expanding_search(df, pts, {0.8, -0.8, 0.3 + 2.1}, SearchParams{}, MatchParams{});
    ASSERT_TRUE(r.found);
    EXPECT_EQ(r.rings, 1);
    EXPECT_NEAR(r.pose.x, truth.x, 0.05);
    EXPECT_NEAR(r.pose.y, truth.y, 0.05);
    EXPECT_NEAR(wrap_angle(r.pose.yaw - truth.yaw), 0.0, 0.03);
    EXPECT_GE(r.score, 0.8);
}

// 差 3m：要扩到第 4 圈（4m）才找到，前面几圈都不该误报
TEST(GlobalSearch, FarNeedsOuterRing) {
    std::mt19937 rng(2);
    const auto df = field();
    const Pose2 truth{-3.0, 2.0, -1.0};
    const auto pts = raycast(make_map(), truth, rng);
    const SearchResult r = expanding_search(df, pts, {-0.5, 0.5, 0.0}, SearchParams{}, MatchParams{});
    ASSERT_TRUE(r.found);
    EXPECT_GE(r.rings, 3);
    EXPECT_NEAR(r.pose.x, truth.x, 0.05);
    EXPECT_NEAR(r.pose.y, truth.y, 0.05);
    EXPECT_NEAR(wrap_angle(r.pose.yaw - truth.yaw), 0.0, 0.03);
}

// 和地图对不上的 scan（全是噪点）：搜完也不能报 found
TEST(GlobalSearch, GarbageScanNotAccepted) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-3, 3);
    std::vector<Eigen::Vector2d> pts;
    for (int i = 0; i < 300; ++i) pts.emplace_back(u(rng), u(rng));
    const SearchResult r = expanding_search(field(), pts, {0, 0, 0}, SearchParams{}, MatchParams{});
    EXPECT_FALSE(r.found);
    EXPECT_LT(r.score, 0.6);
}

// 动态过滤：静止墙 + 一个以 1.5m/s 顺着自己身体方向走的人（最难的情况）→ 人的点被去掉、墙保留；
// 机器人自己转动不算动态
TEST(DynamicFilter, RemovesMovingPersonKeepsWalls) {
    dynfilter::DynamicFilter f;
    auto frame = [](double person_y, double robot_yaw) {
        std::vector<Eigen::Vector2d> pts;             // 车体系
        const Eigen::Rotation2Dd Rinv(-robot_yaw);
        for (int i = 0; i < 100; ++i) pts.push_back(Rinv * Eigen::Vector2d(3.0, -2.5 + i * 0.05));   // 墙 x=3
        for (int i = 0; i < 6; ++i) pts.push_back(Rinv * Eigen::Vector2d(1.5, person_y + i * 0.04)); // 人，宽 0.2m
        return pts;
    };
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
    T.rotate(0.05);                                    // 车每帧转 0.05rad
    dynfilter::Stats st;
    std::vector<uint8_t> mask;
    for (int k = 0; k <= 4; ++k) {                     // 0.1s 一帧，人每帧走 0.15m
        mask = f.update(frame(0.15 * k, 0.05 * k), 0.1 * k, k ? T : Eigen::Isometry2d::Identity(), &st);
        if (k < 2) EXPECT_FALSE(st.applied);           // 还没有 0.15s 以前的帧
    }
    ASSERT_TRUE(st.applied);
    int wall_dyn = 0, person_dyn = 0;
    for (int i = 0; i < 100; ++i) wall_dyn += mask[i];
    for (int i = 100; i < 106; ++i) person_dyn += mask[i];
    EXPECT_EQ(wall_dyn, 0);
    EXPECT_EQ(person_dyn, 6);
}

// 慢慢走的人（0.2m/s）不算「高速」，保留
TEST(DynamicFilter, SlowObjectKept) {
    dynfilter::DynamicFilter f;
    dynfilter::Stats st;
    std::vector<uint8_t> mask;
    for (int k = 0; k <= 4; ++k) {
        std::vector<Eigen::Vector2d> pts;
        for (int i = 0; i < 100; ++i) pts.emplace_back(3.0, -2.5 + i * 0.05);
        for (int i = 0; i < 6; ++i) pts.emplace_back(1.5 + 0.02 * k, i * 0.04);
        mask = f.update(pts, 0.1 * k, Eigen::Isometry2d::Identity(), &st);
    }
    ASSERT_TRUE(st.applied);
    EXPECT_EQ(std::count(mask.begin(), mask.end(), 1), 0);
}

// 运动预测错了（大部分点都对不上）：本帧不过滤，免得把整帧扔掉
TEST(DynamicFilter, TooManyDynamicMeansBadPrediction) {
    dynfilter::DynamicFilter f;
    std::vector<Eigen::Vector2d> a, b;
    for (int i = 0; i < 100; ++i) { a.emplace_back(3.0, -2.5 + i * 0.05); b.emplace_back(3.5, -2.5 + i * 0.05); }
    dynfilter::Stats st;
    f.update(a, 0.0, Eigen::Isometry2d::Identity(), &st);
    f.update(a, 0.1, Eigen::Isometry2d::Identity(), &st);
    const auto mask = f.update(b, 0.3, Eigen::Isometry2d::Identity(), &st);
    EXPECT_FALSE(st.applied);
    EXPECT_EQ(std::count(mask.begin(), mask.end(), 1), 0);
}
