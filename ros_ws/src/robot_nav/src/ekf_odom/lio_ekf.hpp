#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace lio_ekf {

/*
 * 状态: x = [px, py, yaw]
 * predict: IMU angular_velocity.z 积分 yaw，位置靠速度外推
 * update:  lio_odom 提供位姿观测
 */
class LioEkf {
public:
    static constexpr int N = 3;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    LioEkf() {
        x_.setZero();
        P_ = Mat3::Identity() * 0.1;

        // 过程噪声 Q：反映"predict 不做位置外推"带来的不确定性增长速度
        // 机器人最大速度约 0.5m/s，LIO 周期 0.1s → 单周期位置不确定度 ~(0.05m)²
        // Q_pos = (0.05)² / 0.1s = 0.025 m²/s，让 P_pos 在一个 LIO 周期内
        // 长到与 R_pos 同量级，保证 update 时增益有意义
        Q_ = Mat3::Zero();
        Q_(0,0) = 0.025;   // m²/s
        Q_(1,1) = 0.025;   // m²/s
        Q_(2,2) = 1e-3;    // rad²/s（yaw 靠 IMU 积分，不确定度小得多）

        // 量测噪声 R：lio_odom 的实际精度
        // 位置误差 ~5cm → 0.05² = 0.0025 m²
        // yaw 误差 ~2° → (0.035rad)² ≈ 0.0012 rad²
        R_ = Mat3::Zero();
        R_(0,0) = 0.0025;
        R_(1,1) = 0.0025;
        R_(2,2) = 0.0012;
    }

    // predict: dt 秒，imu_yaw_rate rad/s
    void predict(double dt, double imu_yaw_rate) {
        dt = std::clamp(dt, 1e-5, 0.2);

        // 用当前位姿外推位置（速度未知，靠量测更新来修正，预测只转 yaw）
        x_(2) = wrap(x_(2) + imu_yaw_rate * dt);

        // F = I（位置在 predict 里不动，等 update 修正）
        // 实际 P 扩散靠 Q
        P_ += Q_ * dt;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // update: lio_odom 给出的 px, py, yaw
    void update(double px, double py, double yaw) {
        Vec3 z(px, py, yaw);
        Vec3 innov = z - x_;
        innov(2) = wrap(innov(2));

        // H = I
        const Mat3 S = P_ + R_;
        const Mat3 K = P_ * S.inverse();
        x_ += K * innov;
        x_(2) = wrap(x_(2));
        P_ = (Mat3::Identity() - K) * P_;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    const Vec3 & state() const { return x_; }
    void set_state(double px, double py, double yaw) {
        x_ << px, py, yaw;
    }

private:
    static double wrap(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    Vec3 x_;
    Mat3 P_, Q_, R_;
};

} // namespace lio_ekf
