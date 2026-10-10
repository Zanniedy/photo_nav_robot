#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// 代办：用 yaml 文件配置相应的参数
namespace lio_ekf {

/*
 * 状态: x = [px, py, yaw]
 * predict: yaw_rate 积分 yaw，位置靠 update 修正
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

        Q_ = Mat3::Zero();
        Q_(0,0) = 0.025;   // m²/s
        Q_(1,1) = 0.025;   // m²/s
        Q_(2,2) = 1e-3;    // rad²/s

        R_ = Mat3::Zero();
        R_(0,0) = 0.0025;  // (0.05m)²
        R_(1,1) = 0.0025;
        R_(2,2) = 0.0012;  // (~2°)²
    }

    void predict(double dt, double yaw_rate) {
        dt = std::clamp(dt, 1e-5, 0.2);
        x_(2) = wrap(x_(2) + yaw_rate * dt);
        P_ += Q_ * dt;
    }

    void update(double px, double py, double yaw) {
        Vec3 z(px, py, yaw);
        Vec3 innov = z - x_;
        innov(2) = wrap(innov(2));

        const Mat3 S = P_ + R_;
        const Mat3 K = P_ * S.inverse();
        x_ += K * innov;
        x_(2) = wrap(x_(2));
        P_ = (Mat3::Identity() - K) * P_;
        // 保持对称性
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

    Vec3 x_;
    Mat3 P_, Q_, R_;
};

} // namespace lio_ekf
