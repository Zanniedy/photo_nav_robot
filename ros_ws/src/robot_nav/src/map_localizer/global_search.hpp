#pragma once
// 定位丢失后的重定位：从当前（最后可信的）位置开始，一圈圈往外扩的穷举 + 精配
//
// 第 k 圈只搜 r_{k-1} < |p - center| <= r_k 的环（第一圈是圆盘），每个位置 360° 枚举 yaw：
//   1. 粗评分：点在距离场上的 mean exp(-d²/2σ²)（和 relocalize_node.py 一样），只取地图里空闲、离墙够远的位置
//   2. 这一圈粗评分最高的 top_k 个候选，各自 Gauss-Newton 精配（match_scan，先验很弱），按内点率排序
//   3. 最好的内点率 >= accept_score 就停，不再往外扩；否则扩下一圈，直到 max_radius
// 近处先搜：误差不大时很快找到，也不容易被远处相似的结构（对称走廊）抢走。
//
// 纯 Eigen，单测 test/test_global_search.cpp

#include "scan_matcher_2d.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace map_localizer {

struct SearchParams {
    std::vector<double> radii{0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 40.0};   // m，每一圈的外半径
    double xy_step      = 0.15;     // m，粗搜位置步长
    double yaw_step     = 5.0 * M_PI / 180.0;
    double sigma        = 0.15;     // m，粗评分
    double clearance    = 0.15;     // m，候选位置离墙至少这么远（机器人不会在墙里）
    int    top_k        = 8;        // 每圈精配几个候选
    int    max_points   = 120;      // 粗搜用的点数（均匀抽稀）
    double accept_score = 0.6;      // 内点率达到就停止外扩
};

struct SearchResult {
    bool   found{false};     // 内点率 >= accept_score
    Pose2  pose;
    double score{0};          // 最好的内点率
    double radius{0};         // 在哪一圈找到 / 最后搜到哪一圈
    int    rings{0};
    long   evals{0};          // 粗评分的位姿数
};

namespace detail {
inline double coarse_score(const DistanceField& df, const std::vector<Eigen::Vector2d>& pts,
                           double x, double y, double c, double s, double inv2s2) {
    double acc = 0;
    for (const auto& p : pts) {
        const double d = df.eval(c * p.x() - s * p.y() + x, s * p.x() + c * p.y() + y);
        acc += std::exp(d * d * inv2s2);
    }
    return acc / std::max<size_t>(1, pts.size());
}
}  // namespace detail

// pts: 车体系激光点（已去掉动态点）；center: 搜索中心（最后可信的 map 系位姿）
inline SearchResult expanding_search(const DistanceField& df, const std::vector<Eigen::Vector2d>& pts,
                                     const Pose2& center, const SearchParams& sp, const MatchParams& mp_track)
{
    SearchResult out;
    if (pts.size() < 20 || df.empty()) return out;

    // 粗搜点均匀抽稀
    std::vector<Eigen::Vector2d> few;
    const double step = std::max(1.0, double(pts.size()) / std::max(1, sp.max_points));
    for (double i = 0; i < pts.size(); i += step) few.push_back(pts[static_cast<size_t>(i)]);

    MatchParams mp = mp_track;      // 精配：先验很弱，让 GN 自己收敛
    mp.prior_xy = mp.prior_yaw = 1e-3;
    mp.max_iters = std::max(mp.max_iters, 20);

    const double inv2s2 = -0.5 / (sp.sigma * sp.sigma);
    const int n_yaw = std::max(1, static_cast<int>(std::round(2 * M_PI / sp.yaw_step)));
    std::vector<double> cs(n_yaw), sn(n_yaw);
    for (int k = 0; k < n_yaw; ++k) { cs[k] = std::cos(k * sp.yaw_step); sn[k] = std::sin(k * sp.yaw_step); }

    struct Cand { double score; double x, y, yaw; };
    double r_in = -1.0;
    for (double r_out : sp.radii) {
        ++out.rings;
        out.radius = r_out;
        std::vector<Cand> ring;
        const int n = static_cast<int>(std::ceil(r_out / sp.xy_step));
        for (int i = -n; i <= n; ++i) {
            for (int j = -n; j <= n; ++j) {
                const double dx = i * sp.xy_step, dy = j * sp.xy_step;
                const double rr = std::hypot(dx, dy);
                if (rr > r_out || rr <= r_in) continue;
                const double x = center.x + dx, y = center.y + dy;
                if (!df.is_free(x, y) || df.eval(x, y) < sp.clearance) continue;
                // 每个位置只留最好的 yaw，避免 top_k 全是同一个位置
                Cand best{-1, x, y, 0};
                for (int k = 0; k < n_yaw; ++k) {
                    const double sc = detail::coarse_score(df, few, x, y, cs[k], sn[k], inv2s2);
                    if (sc > best.score) best = {sc, x, y, k * sp.yaw_step};
                }
                out.evals += n_yaw;
                ring.push_back(best);
            }
        }
        r_in = r_out;
        if (ring.empty()) continue;
        const size_t k = std::min<size_t>(sp.top_k, ring.size());
        std::partial_sort(ring.begin(), ring.begin() + k, ring.end(),
                          [](const Cand& a, const Cand& b) { return a.score > b.score; });
        for (size_t c = 0; c < k; ++c) {
            const MatchResult m = match_scan(df, pts, {ring[c].x, ring[c].y, ring[c].yaw}, mp);
            if (m.inlier_ratio > out.score) {
                out.score = m.inlier_ratio;
                out.pose = m.pose;
            }
        }
        if (out.score >= sp.accept_score) {
            out.found = true;
            return out;
        }
        if (r_out > df.extent()) break;     // 已经盖住整张地图
    }
    return out;
}

}  // namespace map_localizer
