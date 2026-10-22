// cmd_control.hpp 单元测试：查表前馈、角速度 PI（含简化底盘模型闭环）、线速度增益自适应
#include "cmd_control.hpp"

#include <gtest/gtest.h>

#include <deque>

using namespace robot_bridge;

namespace {

SpeedTable wz_table() {
    SpeedTable t;
    EXPECT_TRUE(t.set({0.17, 0.20, 0.30, 0.50}, {0.45, 0.558, 0.855, 1.676}));
    return t;
}

}  // namespace

TEST(SpeedTable, RejectsBadInput) {
    SpeedTable t;
    EXPECT_FALSE(t.set({0.1}, {0.1}));
    EXPECT_FALSE(t.set({0.1, 0.2}, {0.1}));
    EXPECT_FALSE(t.set({0.2, 0.1}, {0.1, 0.2}));
    EXPECT_FALSE(t.set({0.1, 0.2}, {0.3, 0.3}));
}

TEST(SpeedTable, InterpolatesAndRoundTrips) {
    const SpeedTable t = wz_table();
    EXPECT_NEAR(t.u_of(0.855), 0.30, 1e-9);                    // 节点
    EXPECT_NEAR(t.u_of((0.855 + 1.676) / 2), 0.40, 1e-9);      // 段中点
    EXPECT_NEAR(t.u_of(0.1), 0.17, 1e-9);                      // 低于首点：给起步 u
    EXPECT_NEAR(t.u_of(2.087), 0.60, 1e-3);                    // 末段外推
    for (double w : {0.5, 0.7, 1.0, 1.5})
        EXPECT_NEAR(t.v_of(t.u_of(w)), w, 1e-9);
}

TEST(Feedforward, SignAndDeadband) {
    const SpeedTable t = wz_table();
    EXPECT_EQ(feedforward_u(0.05, t, 0.12, 0.6), 0.0);
    EXPECT_NEAR(feedforward_u(-0.855, t, 0.12, 0.6), -0.30, 1e-9);
    EXPECT_NEAR(feedforward_u(5.0, t, 0.12, 0.6), 0.6, 1e-9);   // 限幅
}

TEST(RatePi, OutputKeepsDirectionAndFloor) {
    RatePi pi;
    // 测量远快于指令：PI 往下压，但不低于 u_floor、不反向
    for (int i = 0; i < 200; ++i) {
        const double u = pi.step(+1, 0.3 - 2.0, 0.17, 0.02);
        EXPECT_GE(u, pi.u_floor - 1e-12);
    }
    // 压在下限时不继续积分（抗饱和）
    const double integ = pi.integ;
    pi.step(+1, -1.7, 0.17, 0.02);
    EXPECT_EQ(pi.integ, integ);
    // 负方向对称
    pi.reset();
    EXPECT_LE(pi.step(-1, -0.3 + 2.0, -0.17, 0.02), -pi.u_floor + 1e-12);
}

// 简化底盘模型：纯滞后 0.15s + 一阶 0.1s，真实增益比前馈表低 25%（模拟电量低/地面阻力大），
// 闭环应把稳态误差消掉，纯前馈会差 25%
TEST(RatePi, ClosedLoopRemovesGainError) {
    const SpeedTable t = wz_table();
    const double dt = 0.02, delay = 0.15, tau = 0.1, plant_scale = 0.75;
    auto run = [&](bool closed) {
        RatePi pi;
        std::deque<double> pipe(static_cast<size_t>(delay / dt), 0.0);
        double w = 0.0;
        const double ref = 0.8;
        for (int k = 0; k < 250; ++k) {   // 5s
            double u = feedforward_u(ref, t, 0.12, 0.6);
            if (closed) u = pi.step(+1, ref - w, u, dt);
            pipe.push_back(u);
            const double ud = pipe.front();
            pipe.pop_front();
            const double w_ss = plant_scale * t.v_of(std::abs(ud));
            w += (w_ss - w) * dt / tau;
        }
        return w;
    };
    EXPECT_NEAR(run(false), 0.6, 0.02);
    EXPECT_NEAR(run(true), 0.8, 0.02);
}

TEST(GainAdapter, ConvergesAndClamps) {
    GainAdapter g;
    // 真实速度 = 指令 / 1.2（底盘偏慢）。k 收敛到 1.2 时 table(k·v) 正好补上
    for (int i = 0; i < 400; ++i) g.update(0.3, 0.3 * g.k / 1.2, 0.1);
    EXPECT_NEAR(g.k, 1.2, 0.01);
    // 坏的估计（一直测到 0）只能把 k 推到上限
    for (int i = 0; i < 400; ++i) g.update(0.3, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(g.k, g.k_max);
    // 倒车同样适用
    GainAdapter r;
    for (int i = 0; i < 400; ++i) r.update(-0.3, -0.3 * r.k / 1.1, 0.1);
    EXPECT_NEAR(r.k, 1.1, 0.01);
}

TEST(SmoothAxis, ContinuousAccelerationAndReachesTarget) {
    SmoothAxis ax;
    const double dt = 0.02, acc = 0.4, dec = 0.6, jerk = 1.5;
    double prev_a = 0.0, max_da = 0.0, max_v = 0.0;
    for (int k = 0; k < 150; ++k) {   // 3s 阶跃 0→0.25
        ax.step(0.25, dt, acc, dec, jerk);
        max_da = std::max(max_da, std::abs(ax.a - prev_a));
        max_v = std::max(max_v, ax.v);
        prev_a = ax.a;
    }
    EXPECT_NEAR(ax.v, 0.25, 1e-6);
    EXPECT_LE(max_da, jerk * dt + 1e-9);     // 加速度每步变化不超过 jerk·dt
    EXPECT_LE(max_v, 0.25 + 0.005);          // 基本不超调
    // 中途反向（0.25 → -0.2）：仍然连续
    prev_a = ax.a; max_da = 0.0;
    for (int k = 0; k < 200; ++k) {
        ax.step(-0.2, dt, acc, dec, jerk);
        if (std::abs(ax.a - prev_a) > max_da && ax.a != 0.0) max_da = std::abs(ax.a - prev_a);
        prev_a = ax.a;
    }
    EXPECT_NEAR(ax.v, -0.2, 1e-6);
    EXPECT_LE(max_da, jerk * dt + 1e-9);
}

TEST(FeedforwardWz, MovingRemovesDeadbandLift) {
    const SpeedTable t = wz_table();
    // 原地：小指令要么 0 要么抬到起步 u
    EXPECT_EQ(feedforward_wz(0.08, t, 0.12, 0.01, 0.6, 0.0), 0.0);
    EXPECT_NEAR(feedforward_wz(0.2, t, 0.12, 0.01, 0.6, 0.0), 0.17, 1e-9);
    // 行进中：过原点线性，0.2rad/s 给 0.2/0.45 × 0.17
    EXPECT_NEAR(feedforward_wz(0.2, t, 0.12, 0.01, 0.6, 1.0), 0.2 / 0.45 * 0.17, 1e-9);
    EXPECT_NEAR(feedforward_wz(-0.05, t, 0.12, 0.01, 0.6, 1.0), -0.05 / 0.45 * 0.17, 1e-9);
    // 表内速度两种模式一致
    EXPECT_NEAR(feedforward_wz(0.855, t, 0.12, 0.01, 0.6, 1.0), 0.30, 1e-9);
    // 行进中 u 随 w 单调连续（无跳变）
    double prev = 0.0;
    for (double w = 0.01; w < 1.5; w += 0.005) {
        const double u = feedforward_wz(w, t, 0.12, 0.01, 0.6, 1.0);
        EXPECT_GE(u, prev);
        EXPECT_LT(u - prev, 0.01);
        prev = u;
    }
}
