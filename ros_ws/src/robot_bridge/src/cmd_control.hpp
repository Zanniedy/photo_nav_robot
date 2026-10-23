#pragma once
// cmd_vel(m/s, rad/s) → MCU 摇杆量 u∈[-1,1] 的映射与闭环，纯函数，不依赖 ROS（便于单测）
//
//   前馈：实测 (u, 稳态速度) 表，分段线性反查；|v| 低于表首速度时给表首 u（刚过起步死区）
//   角速度闭环：陀螺 yaw_speed 反馈的 PI，叠加在前馈上
//   线速度自适应：没有快的线速度反馈，只用 EKF 速度慢慢修正前馈增益

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace robot_bridge {

class SpeedTable {
public:
    // u、v 都要严格递增，至少 2 个点；不合法返回 false 并写 err
    bool set(const std::vector<double>& u, const std::vector<double>& v, std::string* err = nullptr) {
        auto fail = [&](const char* m) { if (err) *err = m; return false; };
        if (u.size() != v.size()) return fail("u/v 长度不一致");
        if (u.size() < 2)         return fail("至少 2 个点");
        for (size_t i = 1; i < u.size(); ++i)
            if (!(u[i] > u[i - 1]) || !(v[i] > v[i - 1])) return fail("u/v 必须严格递增");
        if (u.front() < 0.0 || v.front() <= 0.0) return fail("u 需 >=0，v 需 >0");
        u_ = u; v_ = v;
        return true;
    }

    // |v| → u（非负）；表外：低于首点取首点 u，高于末点按末段斜率外推
    double u_of(double v_abs) const {
        if (v_abs <= v_.front()) return u_.front();
        size_t i = 1;
        while (i + 1 < v_.size() && v_abs > v_[i]) ++i;
        const double r = (v_abs - v_[i - 1]) / (v_[i] - v_[i - 1]);
        return u_[i - 1] + r * (u_[i] - u_[i - 1]);
    }

    // u → 速度（非负），供日志/调试；u 低于首点视为 0
    double v_of(double u_abs) const {
        if (u_abs < u_.front()) return 0.0;
        size_t i = 1;
        while (i + 1 < u_.size() && u_abs > u_[i]) ++i;
        const double r = (u_abs - u_[i - 1]) / (u_[i] - u_[i - 1]);
        return v_[i - 1] + r * (v_[i] - v_[i - 1]);
    }

    double v_first() const { return v_.front(); }

private:
    std::vector<double> u_{0.0, 1.0}, v_{0.0, 1.0};
};

// 加加速度限制的速度平滑（S 曲线）：速度、加速度都连续，每步加速度变化不超过 jerk·dt，不会顿挫
//   正常段：加速度以 jerk 爬到 a_lim（|v| 增大用 acc，减小/过零用 dec）
//   收尾段：当前加速度按 jerk 收到 0 还会带来 a|a|/(2·jerk) 的速度变化，剩余误差不够这么多就开始收
//   误差用下一步的速度算，减小离散带来的超调（实测阶跃 0→0.25 超调约 3mm/s）
struct SmoothAxis {
    double v{0.0}, a{0.0};

    double step(double target, double dt, double acc, double dec, double jerk) {
        const bool speeding_up = std::abs(target) > std::abs(v) && v * target >= 0.0;
        const double a_lim = speeding_up ? acc : dec;
        const double e = target - (v + a * dt);
        const double s = e > 0.0 ? 1.0 : -1.0;
        const double tail = a * std::abs(a) / (2.0 * jerk);
        const double a_des = (e - tail) * s <= 0.0 ? 0.0 : s * a_lim;
        a += std::clamp(a_des - a, -jerk * dt, jerk * dt);
        v += a * dt;
        if (std::abs(a) < 1e-12 && std::abs(target - v) < 3e-3) v = target;   // 加速度已为 0，落到目标上
        return v;
    }
    void reset() { v = a = 0.0; }
};

// 软件急停：目标速度归零、平滑器状态清零。解除后从 0 按加速度 / 加加速度限制起步，
// 不会带着急停前的速度冲出去。只是软件保护，不替代遥控器和硬件急停
inline void apply_estop(double& target_vx, double& target_wz, SmoothAxis& vx, SmoothAxis& wz) {
    target_vx = target_wz = 0.0;
    vx.reset();
    wz.reset();
}

// 前馈：带符号速度 → 带符号 u；|v| < v_min 视为 0
inline double feedforward_u(double v, const SpeedTable& t, double v_min, double u_max) {
    if (std::abs(v) < v_min) return 0.0;
    const double u = std::min(t.u_of(std::abs(v)), u_max);
    return v > 0.0 ? u : -u;
}

// 转向前馈，按"是否在前进"在两种映射间连续过渡（moving∈[0,1]）：
//   静止（原地转）：要克服静摩擦，低于表首速度也给表首 u（起步死区补偿），低于 v_min 归零
//   行进中：轮子已经在转，没有起步死区，小转向按表首段斜率过原点线性映射，不抬、不归零
//   旧做法行进中也抬：MPPI 循迹的小修正（0.1~0.3rad/s）被抬到 0.45rad/s 或归零，转向一顿一顿
inline double feedforward_wz(double w, const SpeedTable& t, double v_min_static, double v_min_moving,
                             double u_max, double moving) {
    moving = std::clamp(moving, 0.0, 1.0);
    const double a = std::abs(w);
    const double v_min = (1.0 - moving) * v_min_static + moving * v_min_moving;
    if (a < v_min) return 0.0;
    const double u_tab = t.u_of(a);
    const double u_lin = a < t.v_first() ? a * t.u_of(t.v_first()) / t.v_first() : u_tab;
    const double u = std::min((1.0 - moving) * u_tab + moving * u_lin, u_max);
    return w > 0.0 ? u : -u;
}

// 角速度 PI（叠加在前馈上）
//   - 输出幅值限制在 [u_floor, u_max]，方向始终跟 target 一致：
//     不让 PI 把 u 拉到死区以下停住再积分回来（会变成走走停停的极限环），也不让它反向打
//   - 条件积分抗饱和：输出被限幅且误差还在往外推时不积分
//   - target 换向时由调用方 reset
struct RatePi {
    double kp{0.05}, ki{0.5}, i_max{0.15}, u_floor{0.12}, u_max{0.6};
    double integ{0.0};

    void reset() { integ = 0.0; }

    // sgn：target 方向（±1）；err = 参考 - 测量（rad/s）；u_ff 带符号
    double step(double sgn, double err, double u_ff, double dt) {
        const double i_new = std::clamp(integ + ki * err * dt, -i_max, i_max);
        double mag = sgn * (u_ff + kp * err + i_new);
        const double push = sgn * err;   // >0：还想更快
        bool hold = false;
        if (mag > u_max)   { mag = u_max;   hold = push > 0.0; }
        if (mag < u_floor) { mag = u_floor; hold = push < 0.0; }
        if (!hold) integ = i_new;
        return sgn * mag;
    }
};

// 线速度前馈增益自适应：u = table(k·|v_cmd|)
//   实际比指令慢 → k 增大。按相对误差积分，rate 是 1/s，clamp 防止被坏的速度估计带偏
struct GainAdapter {
    double k{1.0}, rate{0.3}, k_min{0.8}, k_max{1.3};

    void update(double v_cmd, double v_meas, double dt) {
        const double a = std::abs(v_cmd);
        if (a < 1e-3) return;
        const double rel = (a - std::copysign(1.0, v_cmd) * v_meas) / a;
        k = std::clamp(k + rate * std::clamp(rel, -0.5, 0.5) * dt, k_min, k_max);
    }
};

}  // namespace robot_bridge
