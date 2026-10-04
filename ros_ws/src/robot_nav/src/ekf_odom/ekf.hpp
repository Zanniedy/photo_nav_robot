#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace ekf {

/*
 * 状态向量 x = [ px, py, yaw, vx, vy, yaw_rate ]
 * 平面差速机器人，imu 只提供 yaw_rate 修正
 */
class Ekf
{
public:
    static constexpr int N = 6;
    using Vec = Eigen::Matrix<double, N, 1>;
    using Mat = Eigen::Matrix<double, N, N>;

    Ekf()
    {
        x_.setZero();

        P_ = Mat::Identity();
        P_(0, 0) = 1.0;
        P_(1, 1) = 1.0;
        P_(2, 2) = 0.5;
        P_(3, 3) = 1.0;
        P_(4, 4) = 1.0;
        P_(5, 5) = 0.5;

        Q_ = Mat::Zero();
        Q_(0, 0) = 0.01;
        Q_(1, 1) = 0.01;
        Q_(2, 2) = 0.005;
        Q_(3, 3) = 0.1;
        Q_(4, 4) = 0.1;
        Q_(5, 5) = 0.05;
    }

    // ── predict ────────────────────────────────────────────────────────────
    void predict(double dt)
    {
        dt = std::clamp(dt, MIN_DT, MAX_DT);

        const double yaw      = x_(2);
        const double vx       = x_(3);
        const double vy       = x_(4);
        const double yaw_rate = x_(5);

        // 运动方程（局部速度转全局坐标）
        x_(0) += dt * (vx * std::cos(yaw) - vy * std::sin(yaw));
        x_(1) += dt * (vx * std::sin(yaw) + vy * std::cos(yaw));
        x_(2)  = normalize(yaw + yaw_rate * dt);
        // vx, vy, yaw_rate 由量测更新，predict 阶段保持不变

        // 雅可比 F = ∂f/∂x
        Mat F = Mat::Identity();
        F(0, 2) = dt * (-vx * std::sin(yaw) - vy * std::cos(yaw));
        F(0, 3) = dt *  std::cos(yaw);
        F(0, 4) = dt * -std::sin(yaw);
        F(1, 2) = dt * ( vx * std::cos(yaw) - vy * std::sin(yaw));
        F(1, 3) = dt *  std::sin(yaw);
        F(1, 4) = dt *  std::cos(yaw);
        F(2, 5) = dt;

        P_ = F * P_ * F.transpose() + Q_ * dt;
        P_ = 0.5 * (P_ + P_.transpose());  // 保持对称
    }

    // ── update: odom 提供 vx, vy, yaw_rate ────────────────────────────────
    void update_odom(double vx, double vy, double yaw_rate,
                     double var_vx, double var_vy, double var_yr)
    {
        Eigen::Vector3d z, z_pred;
        z      << vx, vy, yaw_rate;
        z_pred << x_(3), x_(4), x_(5);

        Eigen::Matrix<double, 3, N> H;
        H.setZero();
        H(0, 3) = 1.0;
        H(1, 4) = 1.0;
        H(2, 5) = 1.0;

        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R(0, 0) = std::max(var_vx, 1e-9);
        R(1, 1) = std::max(var_vy, 1e-9);
        R(2, 2) = std::max(var_yr, 1e-9);

        apply_update(H, R, (z - z_pred).eval());
    }

    // ── update: imu 只提供 yaw_rate ────────────────────────────────────────
    void update_imu(double yaw_rate, double var_yr)
    {
        Eigen::Matrix<double, 1, N> H;
        H.setZero();
        H(0, 5) = 1.0;

        Eigen::Matrix<double, 1, 1> R;
        R(0, 0) = std::max(var_yr, 1e-9);

        Eigen::Matrix<double, 1, 1> innov;
        innov(0, 0) = yaw_rate - x_(5);

        apply_update(H, R, innov);
    }

    const Vec & state() const { return x_; }

private:
    // 通用量测更新，任意维度 H
    template <int M>
    void apply_update(const Eigen::Matrix<double, M, N> & H,
                      const Eigen::Matrix<double, M, M> & R,
                      const Eigen::Matrix<double, M, 1> & innov)
    {
        const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, N, M> K = P_ * H.transpose() * S.inverse();

        x_ += K * innov;
        x_(2) = normalize(x_(2));

        // Joseph 形式，数值稳定
        const Mat I_KH = Mat::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    static double normalize(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static constexpr double MIN_DT = 1e-5;
    static constexpr double MAX_DT = 0.2;

    Vec x_;
    Mat P_;
    Mat Q_;
};

} // namespace ekf
