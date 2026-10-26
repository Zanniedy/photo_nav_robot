#pragma once
// 动态点过滤：去掉帧间移动太快的激光点（走动的人、推车），不让它们参与位姿计算
//
// 做法：和约 ref_dt 秒前的一帧比（相邻两帧只隔 0.1s，人顺着身体方向走时前后两帧的点还重叠，看不出动）。
// 那一帧的点按里程计链算出的相对位姿变换到当前车体系，当前每个点找那一帧里的最近点：
//   最近距离 > speed·dt + margin  → 这个点在这段时间里「动了」（或者是刚露出来的），标为动态
// 再沿扫描顺序扩张到同一物体（相邻点距离 < grow_dr）：人腿边缘的点即使恰好和上一帧重合也一起去掉。
// 动态点比例超过 max_ratio 时说明是运动预测错了（转太快、打滑），本帧不过滤，全部保留。
// 刚露出来的静态背景只会被去掉一帧，下一帧上一帧里就有它了。
//
// 纯 Eigen，无 ROS 依赖；单测 test/test_dynamic_filter.cpp

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace dynfilter {

struct Params {
    bool   enable    = true;
    double speed     = 0.5;    // m/s，比这快的点算动态（人走路 1m/s 左右）
    double margin    = 0.06;   // m，激光噪声 + 预测误差
    double grow_dr   = 0.12;   // m，扩张：扫描顺序相邻、距离小于这个算同一物体
    int    grow_n    = 3;      // 每个动态点向两边最多扩张几个点
    double max_ratio = 0.3;    // 动态点超过这个比例本帧不过滤
    double ref_dt    = 0.3;    // s，和多久以前的一帧比
    double max_dt    = 0.8;    // s，找不到这么近的历史帧就不比
};

struct Stats {
    int  total{0};
    int  dynamic{0};
    bool applied{false};       // false：第一帧 / 隔太久 / 比例超限，本帧没过滤
};

class DynamicFilter {
public:
    explicit DynamicFilter(const Params& p = {}) : p_(p) {}
    void set_params(const Params& p) { p_ = p; }
    const Params& params() const { return p_; }
    void reset() { hist_.clear(); }

    // pts：当前帧点（车体系，按扫描顺序）；t：扫描时间 s；
    // T_prev_cur：当前车体在上一次调用时车体系下的位姿（里程计增量）。
    // 返回和 pts 等长的掩码，1 = 动态（要丢掉）
    std::vector<uint8_t> update(const std::vector<Eigen::Vector2d>& pts, double t,
                                const Eigen::Isometry2d& T_prev_cur, Stats* st = nullptr)
    {
        std::vector<uint8_t> dyn(pts.size(), 0);
        Stats s;
        s.total = static_cast<int>(pts.size());
        // 里程计链：当前帧在「链坐标系」下的位姿
        const Eigen::Isometry2d cur = hist_.empty() ? Eigen::Isometry2d::Identity() : hist_.back().pose * T_prev_cur;
        // 选和 ref_dt 最接近、又不比它新太多的历史帧
        const Frame* ref = nullptr;
        for (const auto& f : hist_) {
            const double age = t - f.t;
            if (age < 0.5 * p_.ref_dt || age > p_.max_dt) continue;
            if (!ref || std::abs(age - p_.ref_dt) < std::abs((t - ref->t) - p_.ref_dt)) ref = &f;
        }
        const double dt = ref ? t - ref->t : 0.0;
        if (p_.enable && ref) {
            prev_ = &ref->pts;
            const Eigen::Isometry2d T_ref_cur = ref->pose.inverse() * cur;
            const double gate = p_.speed * dt + p_.margin;
            const double gate2 = gate * gate;
            cell_ = std::max(gate, 0.05);
            build_grid();
            int n = 0;
            for (size_t i = 0; i < pts.size(); ++i) {
                const Eigen::Vector2d q = T_ref_cur * pts[i];
                if (nearest2(q, gate2) > gate2) { dyn[i] = 1; ++n; }
            }
            // 扩张到同一物体
            if (n > 0 && p_.grow_n > 0) {
                std::vector<uint8_t> seed = dyn;
                const double g2 = p_.grow_dr * p_.grow_dr;
                for (size_t i = 0; i < pts.size(); ++i) {
                    if (!seed[i]) continue;
                    for (int dir : {-1, 1}) {
                        size_t prev = i;
                        for (int k = 1; k <= p_.grow_n; ++k) {
                            const long j = static_cast<long>(i) + dir * k;
                            if (j < 0 || j >= static_cast<long>(pts.size())) break;
                            if ((pts[j] - pts[prev]).squaredNorm() > g2) break;
                            if (!dyn[j]) { dyn[j] = 1; ++n; }
                            prev = static_cast<size_t>(j);
                        }
                    }
                }
            }
            if (n <= p_.max_ratio * pts.size()) {
                s.dynamic = n;
                s.applied = true;
            } else {
                std::fill(dyn.begin(), dyn.end(), 0);
            }
        }
        hist_.push_back({t, cur, pts});     // 动态点也存：刚露出来的背景过一会儿就是静态了
        while (!hist_.empty() && t - hist_.front().t > p_.max_dt) hist_.pop_front();
        if (st) *st = s;
        return dyn;
    }

private:
    static int64_t key(int ix, int iy) { return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy); }

    void build_grid() {
        grid_.clear();
        const auto& pv = *prev_;
        grid_.reserve(pv.size() * 2);
        for (size_t i = 0; i < pv.size(); ++i) {
            const int ix = static_cast<int>(std::floor(pv[i].x() / cell_));
            const int iy = static_cast<int>(std::floor(pv[i].y() / cell_));
            grid_[key(ix, iy)].push_back(static_cast<int>(i));
        }
    }

    // 3x3 邻域里最近点的距离平方（cell >= gate，所以 gate 以内的点一定在邻域里）
    double nearest2(const Eigen::Vector2d& q, double limit2) const {
        const int ix = static_cast<int>(std::floor(q.x() / cell_));
        const int iy = static_cast<int>(std::floor(q.y() / cell_));
        double best = limit2 * 4 + 1.0;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                auto it = grid_.find(key(ix + dx, iy + dy));
                if (it == grid_.end()) continue;
                for (int j : it->second) {
                    const double d2 = ((*prev_)[j] - q).squaredNorm();
                    if (d2 < best) { best = d2; if (best <= limit2) return best; }
                }
            }
        return best;
    }

    struct Frame {
        double t;
        Eigen::Isometry2d pose;               // 链坐标系下的车体位姿
        std::vector<Eigen::Vector2d> pts;     // 车体系
    };
    Params p_;
    std::deque<Frame> hist_;
    const std::vector<Eigen::Vector2d>* prev_{nullptr};   // 本次比较用的历史帧
    double cell_{0.1};
    std::unordered_map<int64_t, std::vector<int>> grid_;
};

}  // namespace dynfilter
