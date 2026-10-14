#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace lio_ekf {

/*
 * 状态: x = [px, py, yaw]
 *
 * predict:    yaw_rate (rad/s) 积分 yaw，协方差膨胀
 * update_lio: lio_odom 提供完整位姿观测 (px, py, yaw)
 * update_yaw: /robot/yaw 提供单独 yaw 观测，R 独立配置
 */
class LioEkf {
public:
    static constexpr int N = 3;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    LioEkf() {
        x_.setZero();
        P_ = Mat3::Identity() * 0.1;

        Q_ = Mat3::Zero();
        Q_(0,0) = 0.025;    // m²/s
        Q_(1,1) = 0.025;    // m²/s
        Q_(2,2) = 1e-3;     // rad²/s

        // lio_odom 观测噪声 (px, py, yaw)
        R_lio_ = Mat3::Zero();
        R_lio_(0,0) = 0.0025;   // (0.05 m)²
        R_lio_(1,1) = 0.0025;
        R_lio_(2,2) = 0.0012;   // (~2°)²

        // /robot/yaw 观测噪声（单轴）
        r_yaw_ = 0.0003;        // (~1°)²，比 lio yaw 更信任 MCU IMU
    }

    // ── 配置接口（供 yaml 参数覆盖）────────────────────────────────────────
    void set_Q(double qxy, double qyaw)       { Q_(0,0)=Q_(1,1)=qxy; Q_(2,2)=qyaw; }
    void set_R_lio(double rxy, double ryaw)   { R_lio_(0,0)=R_lio_(1,1)=rxy; R_lio_(2,2)=ryaw; }
    void set_r_yaw(double r)                  { r_yaw_ = r; }

    // ── predict: yaw_rate 积分 ──────────────────────────────────────────────
    void predict(double dt, double yaw_rate) {
        dt = std::clamp(dt, 1e-5, 0.2);
        x_(2) = wrap(x_(2) + yaw_rate * dt);
        P_ += Q_ * dt;
    }

    // ── update: lio_odom 完整位姿 ───────────────────────────────────────────
    void update_lio(double px, double py, double yaw) {
        Vec3 innov(px - x_(0), py - x_(1), wrap(yaw - x_(2)));
        const Mat3 S = P_ + R_lio_;
        const Mat3 K = P_ * S.inverse();
        x_ += K * innov;
        x_(2) = wrap(x_(2));
        P_ = (Mat3::Identity() - K) * P_;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // ── update: /robot/yaw 单轴观测 ────────────────────────────────────────
    // H = [0, 0, 1]，只更新 yaw 分量
    void update_yaw(double yaw) {
        // 卡尔曼增益 k = P * H^T / (H * P * H^T + r)
        // H*P*H^T = P(2,2)，H*P = P 第 3 列
        double s = P_(2,2) + r_yaw_;
        Eigen::Vector3d k = P_.col(2) / s;

        double innov = wrap(yaw - x_(2));
        x_ += k * innov;
        x_(2) = wrap(x_(2));

        // P = (I - k*H) * P
        Mat3 I_kH = Mat3::Identity();
        I_kH.col(2) -= k;
        P_ = I_kH * P_;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    const Vec3& state() const { return x_; }

    void set_state(double px, double py, double yaw) {
        x_ << px, py, yaw;
    }

private:
    static double wrap(double a) {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        return a < 0.0 ? a + M_PI : a - M_PI;
    }

    Vec3   x_;
    Mat3   P_, Q_, R_lio_;
    double r_yaw_;
};

} // namespace lio_ekf
