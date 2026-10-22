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

/*
 * 速度估计：odom 系匀速模型 KF，状态 v = [px, py, vx, vy]
 *
 * 只吃 lio 位置，dt 用扫描时间戳（不用接收时刻，ICP 处理时间抖动不影响速度）。
 * 和位姿 EKF 分开：位姿 EKF 的参数是按定位效果调的，速度估计要的是另一组折中（噪声 vs 滞后）。
 *   - 白噪声加速度模型，q_acc 越大越跟手、越噪
 *   - valid=false（ICP 被拒帧，lio 只转 yaw 不动平移）只预测不更新，否则会估出假的 0 和补回时的尖峰
 *   - 马氏距离门限剔除离群；连续离群说明模型跟丢（急停、重置关键帧），直接重置
 */
class VelocityKf {
public:
    using Vec4 = Eigen::Vector4d;
    using Mat4 = Eigen::Matrix4d;

    void set_params(double q_acc, double r_xy, double gate_chi2, int max_outliers) {
        q_acc_ = q_acc; r_xy_ = r_xy; gate_ = gate_chi2; max_outliers_ = max_outliers;
    }

    // 返回 false 表示本帧被门限剔除
    bool step(double t, double px, double py, bool valid) {
        const double dt = t - t_;
        if (!init_ || dt <= 0.0 || dt > max_dt_) {
            if (valid) reset(t, px, py);
            return valid;
        }
        t_ = t;

        // predict
        Mat4 F = Mat4::Identity();
        F(0, 2) = F(1, 3) = dt;
        Mat4 Q = Mat4::Zero();
        const double q11 = q_acc_ * dt * dt * dt / 3.0, q12 = q_acc_ * dt * dt / 2.0, q22 = q_acc_ * dt;
        Q(0, 0) = Q(1, 1) = q11;
        Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = q12;
        Q(2, 2) = Q(3, 3) = q22;
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
        if (!valid) return true;

        // update，H = [I2 0]
        const Eigen::Vector2d innov(px - x_(0), py - x_(1));
        const Eigen::Matrix2d S = P_.topLeftCorner<2, 2>() + Eigen::Matrix2d::Identity() * r_xy_;
        if (innov.dot(S.ldlt().solve(innov)) > gate_) {
            if (++outliers_ >= max_outliers_) reset(t, px, py);
            return false;
        }
        outliers_ = 0;
        const Eigen::Matrix<double, 4, 2> K = P_.leftCols<2>() * S.inverse();
        x_ += K * innov;
        P_ = (Mat4::Identity() - K * Eigen::Matrix<double, 2, 4>::Identity()) * P_;
        P_ = 0.5 * (P_ + P_.transpose());
        return true;
    }

    bool initialized() const { return init_; }
    Eigen::Vector2d velocity() const { return x_.tail<2>(); }
    Eigen::Matrix2d velocity_cov() const { return P_.bottomRightCorner<2, 2>(); }

private:
    void reset(double t, double px, double py) {
        x_ << px, py, 0.0, 0.0;
        P_ = Mat4::Zero();
        P_(0, 0) = P_(1, 1) = r_xy_;
        P_(2, 2) = P_(3, 3) = 0.25;   // (0.5 m/s)²：初始速度未知
        t_ = t; init_ = true; outliers_ = 0;
    }

    Vec4   x_{Vec4::Zero()};
    Mat4   P_{Mat4::Identity()};
    double t_{0.0};
    bool   init_{false};
    int    outliers_{0};
    double q_acc_{0.05}, r_xy_{0.0004}, gate_{13.8}, max_dt_{0.5};
    int    max_outliers_{3};
};

} // namespace lio_ekf
