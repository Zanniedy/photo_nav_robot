// LioEkf 单元测试：lio 和 MCU 的 yaw 参考系不同时，静止状态下 yaw 不应来回跳
#include "../src/ekf_odom/lio_ekf.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

using lio_ekf::LioEkf;

namespace {
constexpr double kDeg = M_PI / 180.0;
}

// 实机 10-26：静止时 lio yaw 16.9°、MCU yaw -86.7°（差 104°）。
// 以前直接融合，EKF yaw 被两边轮流拉，在 -80~-83° 之间跳；对齐偏差后应该稳在 lio 的 16.9°
TEST(LioEkf, StationaryWithYawFrameOffsetStaysPut) {
    LioEkf ekf;
    ekf.set_state(-0.97, 4.59, 16.9 * kDeg);
    double lo = 1e9, hi = -1e9;
    for (int i = 0; i < 1000; ++i) {                 // 10s：MCU 100Hz，lio 10Hz
        ekf.predict(0.01, 0.0);
        ekf.update_yaw(-86.7 * kDeg);
        if (i % 10 == 0) ekf.update_lio(-0.97, 4.59, 16.9 * kDeg);
        if (i > 100) {
            lo = std::min(lo, ekf.state()(2));
            hi = std::max(hi, ekf.state()(2));
        }
    }
    EXPECT_NEAR(ekf.state()(2), 16.9 * kDeg, 0.05 * kDeg);
    EXPECT_LT((hi - lo) / kDeg, 0.1);
    EXPECT_NEAR(ekf.yaw_offset() / kDeg, 103.6, 0.01);
}

// 对齐后 MCU yaw 的变化（转动）照常传进来
TEST(LioEkf, FollowsMcuYawChangeAfterAlignment) {
    LioEkf ekf;
    ekf.set_state(0, 0, 10 * kDeg);
    ekf.update_yaw(-50 * kDeg);                      // 对齐：偏差 60°
    for (int i = 0; i < 200; ++i) {
        ekf.predict(0.01, 0.0);
        ekf.update_yaw(-40 * kDeg);                  // MCU 说转了 +10°
    }
    EXPECT_NEAR(ekf.state()(2), 20 * kDeg, 0.5 * kDeg);
}

// 跨 ±180° 的偏差也要正确
TEST(LioEkf, OffsetWrapsAroundPi) {
    LioEkf ekf;
    ekf.set_state(0, 0, 170 * kDeg);
    ekf.update_yaw(-170 * kDeg);
    EXPECT_NEAR(ekf.yaw_offset() / kDeg, -20.0, 1e-6);
    for (int i = 0; i < 100; ++i) ekf.update_yaw(-170 * kDeg);
    EXPECT_NEAR(ekf.state()(2) / kDeg, 170.0, 1e-3);
}
