#pragma once
// 2D scan-to-map 匹配：地图距离场 + Gauss-Newton（带预测先验）
//
// 残差 r_i = D(R(θ) p_i + t)，D 为到最近占据格的距离（双线性插值，截断 max_dist）
// 最小化 Σ huber(r_i) + (x - x_pred)^T W (x - x_pred)
// 先验项防止走廊等退化方向上 GN 乱跳：信息不足的方向保持预测值

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace map_localizer {

struct Pose2 {
    double x{0}, y{0}, yaw{0};
};

inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// T_a_b * T_b_c
inline Pose2 compose(const Pose2& a, const Pose2& b) {
    const double c = std::cos(a.yaw), s = std::sin(a.yaw);
    return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_angle(a.yaw + b.yaw)};
}

inline Pose2 inverse(const Pose2& a) {
    const double c = std::cos(a.yaw), s = std::sin(a.yaw);
    return {-c * a.x - s * a.y, s * a.x - c * a.y, wrap_angle(-a.yaw)};
}

class DistanceField {
public:
    // occ: 行优先，row 0 = y 最小（与 nav_msgs/OccupancyGrid 一致）
    void build(const std::vector<int8_t>& occ, int w, int h, double res,
               double ox, double oy, int occ_thresh = 65, double max_dist = 0.5, int free_thresh = 25)
    {
        w_ = w; h_ = h; res_ = res; ox_ = ox; oy_ = oy; max_dist_ = max_dist;
        free_.resize(occ.size());
        for (size_t i = 0; i < occ.size(); ++i) free_[i] = occ[i] >= 0 && occ[i] < free_thresh;
        // Felzenszwalb-Huttenlocher 精确 EDT，先列后行，单位：格²
        const double INF = 1e20;
        std::vector<double> f(static_cast<size_t>(w) * h);
        for (size_t i = 0; i < f.size(); ++i) f[i] = occ[i] >= occ_thresh ? 0.0 : INF;

        std::vector<double> col(std::max(w, h)), out(std::max(w, h));
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) col[y] = f[static_cast<size_t>(y) * w + x];
            edt1d(col.data(), out.data(), h);
            for (int y = 0; y < h; ++y) f[static_cast<size_t>(y) * w + x] = out[y];
        }
        for (int y = 0; y < h; ++y) {
            edt1d(&f[static_cast<size_t>(y) * w], out.data(), w);
            std::copy(out.begin(), out.begin() + w, f.begin() + static_cast<size_t>(y) * w);
        }
        dist_.resize(f.size());
        for (size_t i = 0; i < f.size(); ++i)
            dist_[i] = static_cast<float>(std::min(std::sqrt(f[i]) * res, max_dist));
    }

    bool empty() const { return dist_.empty(); }
    // 已知空闲格（重定位候选位置用；未知区域、障碍都不算）
    bool is_free(double wx, double wy) const {
        const int ix = static_cast<int>(std::floor((wx - ox_) / res_));
        const int iy = static_cast<int>(std::floor((wy - oy_) / res_));
        return ix >= 0 && iy >= 0 && ix < w_ && iy < h_ && free_[static_cast<size_t>(iy) * w_ + ix];
    }
    double extent() const { return std::hypot(w_ * res_, h_ * res_); }   // 地图对角线 m
    double resolution() const { return res_; }
    double max_dist() const { return max_dist_; }

    // 双线性插值的距离和梯度（世界坐标，m）；越界返回 max_dist、梯度 0
    double eval(double wx, double wy, double* gx = nullptr, double* gy = nullptr) const {
        // 以格子中心为采样点
        const double fx = (wx - ox_) / res_ - 0.5, fy = (wy - oy_) / res_ - 0.5;
        const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
        if (x0 < 0 || y0 < 0 || x0 + 1 >= w_ || y0 + 1 >= h_) {
            if (gx) *gx = 0.0;
            if (gy) *gy = 0.0;
            return max_dist_;
        }
        const double ax = fx - x0, ay = fy - y0;
        const double d00 = at(x0, y0), d10 = at(x0 + 1, y0), d01 = at(x0, y0 + 1), d11 = at(x0 + 1, y0 + 1);
        if (gx) *gx = ((d10 - d00) * (1 - ay) + (d11 - d01) * ay) / res_;
        if (gy) *gy = ((d01 - d00) * (1 - ax) + (d11 - d10) * ax) / res_;
        return (d00 * (1 - ax) + d10 * ax) * (1 - ay) + (d01 * (1 - ax) + d11 * ax) * ay;
    }

private:
    double at(int x, int y) const { return dist_[static_cast<size_t>(y) * w_ + x]; }

    // 1D 下包络抛物线，f/d 长度 n
    static void edt1d(const double* f, double* d, int n) {
        std::vector<int> v(n);
        std::vector<double> z(n + 1);
        int k = 0;
        v[0] = 0; z[0] = -std::numeric_limits<double>::infinity(); z[1] = std::numeric_limits<double>::infinity();
        auto inter = [&](int q, int p) {
            return ((f[q] + double(q) * q) - (f[p] + double(p) * p)) / (2.0 * q - 2.0 * p);
        };
        for (int q = 1; q < n; ++q) {
            double s = inter(q, v[k]);
            while (s <= z[k]) { --k; s = inter(q, v[k]); }   // z[0] = -inf，k 不会越界
            ++k;
            v[k] = q; z[k] = s; z[k + 1] = std::numeric_limits<double>::infinity();
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (z[k + 1] < q) ++k;
            d[q] = (q - v[k]) * double(q - v[k]) + f[v[k]];
        }
    }

    std::vector<float> dist_;
    std::vector<uint8_t> free_;
    int w_{0}, h_{0};
    double res_{0.05}, ox_{0}, oy_{0}, max_dist_{0.5};
};

struct MatchParams {
    int    max_iters    = 15;
    double huber_delta  = 0.10;   // m
    double prior_xy     = 2.0;    // 先验权重，量级≈几个贴墙点的约束，只在退化方向起作用，单位 1/m²
    double prior_yaw    = 2.0;    // 1/rad²
    double inlier_dist  = 0.10;   // m，评分用
};

struct MatchResult {
    Pose2  pose;
    double inlier_ratio{0};
    int    iters{0};
    int    used_points{0};
};

// pts: base 系激光点；pred: map 系预测位姿
inline MatchResult match_scan(const DistanceField& df, const std::vector<Eigen::Vector2d>& pts,
                              const Pose2& pred, const MatchParams& mp = {})
{
    MatchResult res;
    Eigen::Vector3d x(pred.x, pred.y, pred.yaw);
    const Eigen::Vector3d xp = x;
    const Eigen::Vector3d W(mp.prior_xy, mp.prior_xy, mp.prior_yaw);
    const double cutoff = df.max_dist() * 0.99;   // 截断区域无梯度，不参与

    for (int it = 0; it < mp.max_iters; ++it) {
        const double c = std::cos(x(2)), s = std::sin(x(2));
        Eigen::Matrix3d H = W.asDiagonal();
        Eigen::Vector3d b = W.cwiseProduct(Eigen::Vector3d(x(0) - xp(0), x(1) - xp(1), wrap_angle(x(2) - xp(2))));
        int used = 0;
        for (const auto& p : pts) {
            const double qx = c * p.x() - s * p.y() + x(0);
            const double qy = s * p.x() + c * p.y() + x(1);
            double gx, gy;
            const double r = df.eval(qx, qy, &gx, &gy);
            if (r >= cutoff) continue;
            const double w = std::abs(r) <= mp.huber_delta ? 1.0 : mp.huber_delta / std::abs(r);
            // dq/dθ
            const double dqx = -s * p.x() - c * p.y(), dqy = c * p.x() - s * p.y();
            const Eigen::Vector3d J(gx, gy, gx * dqx + gy * dqy);
            H.noalias() += w * J * J.transpose();
            b.noalias() += w * r * J;
            ++used;
        }
        res.used_points = used;
        const Eigen::Vector3d dx = -H.ldlt().solve(b);
        x += dx;
        x(2) = wrap_angle(x(2));
        res.iters = it + 1;
        if (dx.head<2>().norm() < 1e-4 && std::abs(dx(2)) < 1e-4) break;
    }

    res.pose = {x(0), x(1), x(2)};
    const double c = std::cos(x(2)), s = std::sin(x(2));
    int inl = 0;
    for (const auto& p : pts)
        if (df.eval(c * p.x() - s * p.y() + x(0), s * p.x() + c * p.y() + x(1)) < mp.inlier_dist) ++inl;
    res.inlier_ratio = pts.empty() ? 0.0 : double(inl) / pts.size();
    return res;
}

}  // namespace map_localizer
