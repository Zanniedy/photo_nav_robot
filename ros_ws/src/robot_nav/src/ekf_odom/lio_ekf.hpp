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

        // 过程噪声：位置 1cm²，角度 0.5deg²
        Q_ = Mat3::Zero();
        Q_(0,0) = 1e-4;
        Q_(1,1) = 1e-4;
        Q_(2,2) = 1e-4;

        // 量测噪声：lio_odom 位置 5cm²，角度 2deg²
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
