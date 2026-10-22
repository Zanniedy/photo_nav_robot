// VelocityKf 单元测试：匀速收敛、被拒帧不当成静止、离群剔除与重置
#include "../src/ekf_odom/lio_ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using lio_ekf::VelocityKf;

namespace {

VelocityKf make_kf() {
    VelocityKf kf;
    kf.set_params(0.05, 0.0004, 13.8, 3);
    return kf;
}

}  // namespace

TEST(VelocityKf, ConvergesOnNoisyConstantVelocity) {
    VelocityKf kf = make_kf();
    std::mt19937 rng(1);
    std::normal_distribution<double> n(0.0, 0.015);
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    for (int k = 0; k < 100; ++k) {
        const double t = 0.1 * k;
        kf.step(t, 0.3 * t + n(rng), -0.1 * t + n(rng), true);
        if (k >= 50) sum += kf.velocity();
    }
    EXPECT_NEAR(sum(0) / 50, 0.3, 0.02);    // 无偏
    EXPECT_NEAR(sum(1) / 50, -0.1, 0.02);
    EXPECT_LT(std::sqrt(kf.velocity_cov()(0, 0)), 0.12);   // 稳态约 0.1，远小于初始化时的 0.5（bridge 用它排除未收敛）
}

TEST(VelocityKf, RejectedFramesDoNotLookStationary) {
    VelocityKf kf = make_kf();
    for (int k = 0; k < 30; ++k) kf.step(0.1 * k, 0.3 * 0.1 * k, 0.0, true);
    // 连续 3 帧 ICP 被拒：lio 平移冻结在旧值，valid=false
    const double frozen = 0.3 * 2.9;
    for (int k = 30; k < 33; ++k) kf.step(0.1 * k, frozen, 0.0, false);
    EXPECT_NEAR(kf.velocity()(0), 0.3, 0.02);
    // 匹配恢复，位移一次补回：不应出现尖峰
    kf.step(3.3, 0.3 * 3.3, 0.0, true);
    EXPECT_NEAR(kf.velocity()(0), 0.3, 0.03);
}

TEST(VelocityKf, GatesOutlierThenResetsOnPersistentJump) {
    VelocityKf kf = make_kf();
    for (int k = 0; k < 30; ++k) kf.step(0.1 * k, 0.0, 0.0, true);
    EXPECT_FALSE(kf.step(3.0, 1.0, 0.0, true));   // 单帧跳 1m：剔除
    EXPECT_NEAR(kf.velocity()(0), 0.0, 0.01);
    EXPECT_FALSE(kf.step(3.1, 1.0, 0.0, true));
    EXPECT_FALSE(kf.step(3.2, 1.0, 0.0, true));   // 第 3 次：重置到新位置（lio 重置关键帧等）
    EXPECT_NEAR(kf.velocity()(0), 0.0, 1e-9);
    EXPECT_TRUE(kf.step(3.3, 1.0, 0.0, true));    // 之后在新位置正常跟踪
}
