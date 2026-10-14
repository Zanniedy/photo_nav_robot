
#pragma once

#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pointmatcher/PointMatcher.h>
#include <sstream>
#include <algorithm>
#include <execution>
#include <numeric>
#include <vector>
#include <cmath>

namespace lioslam {

using Pose2 = Eigen::Isometry2d;

inline Pose2 make_pose(double x, double y, double yaw) {
    Pose2 T = Pose2::Identity();
    T.linear()      = Eigen::Rotation2Dd(yaw).toRotationMatrix();
    T.translation() = Eigen::Vector2d(x, y);
    return T;
}

inline double pose_yaw(const Pose2 & T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

using PM  = PointMatcher<float>;
using DP  = PM::DataPoints;
using ICP = PM::ICP;

class scan_match {
public:
    scan_match() {
        min_points_ = 20;

        // reading 也加 SurfaceNormalDataPointsFilter，然后用 SurfaceNormalOutlierFilter
        // 剔除法向量夹角过大的点对，减少薄墙/边缘的不稳定匹配
        std::istringstream cfg(R"yaml(
readingDataPointsFilters:
  - RandomSamplingDataPointsFilter:
      prob: 0.75
  - SurfaceNormalDataPointsFilter:
      knn: 7

referenceDataPointsFilters:
  - SurfaceNormalDataPointsFilter:
      knn: 7

matcher:
  KDTreeMatcher:
    knn: 1
    epsilon: 0.1

outlierFilters:
  - TrimmedDistOutlierFilter:
      ratio: 0.75
  - SurfaceNormalOutlierFilter:
      maxAngle: 0.785398

errorMinimizer:
  PointToPlaneErrorMinimizer

transformationCheckers:
  - CounterTransformationChecker:
      maxIterationCount: 40
  - DifferentialTransformationChecker:
      minDiffRotErr:   0.0001
      minDiffTransErr: 0.0001
      smoothLength:    4

inspector:
  NullInspector

logger:
  NullLogger
)yaml");
        icp_.loadFromYaml(cfg);
    }
    ~scan_match() = default;

    // 返回 T_ref_curr：curr 激光系 → ref 激光系；init 为同一含义的初值
    // ok=false 表示点数不足，返回值无意义
    // 注：ref 是关键帧，和 curr 之间可能有较大位移，按索引比较的动态点剔除不再适用，不传 ref_scan
    Pose2 match(const sensor_msgs::msg::LaserScan & ref_scan,
                const sensor_msgs::msg::LaserScan & curr,
                const Pose2 & init,
                bool & ok)
    {
        DP ref  = scan_to_dp(ref_scan);
        DP data = scan_to_dp(curr);
        ok = ref.getNbPoints()  >= min_points_ &&
             data.getNbPoints() >= min_points_;
        if (!ok) return init;
        return run_icp(ref, data, init);
    }

private:
    ICP    icp_;
    size_t min_points_;
    float  dynamic_thresh_{0.3f};  // 同角度距离差超过此值（m）视为动态物体

    DP scan_to_dp(const sensor_msgs::msg::LaserScan & scan,
                  const sensor_msgs::msg::LaserScan * ref_scan = nullptr) const
    {
        const size_t N = scan.ranges.size();

        const bool can_filter = ref_scan &&
            std::abs(scan.angle_increment - ref_scan->angle_increment) < 1e-5f &&
            N == ref_scan->ranges.size();

        // Step 1: 筛选有效索引（动态点剔除 + 距离分层采样），并行标记
        std::vector<int> indices(N);
        std::iota(indices.begin(), indices.end(), 0);

        // 用普通 rand 做确定性降采样（无需线程安全，仅按索引奇偶/模运算）
        std::vector<uint8_t> keep(N, 0);
        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
            [&](int i) {
                float r = scan.ranges[i];
                if (r < scan.range_min || r > scan.range_max) return;

                // 动态点剔除：同方向突然变近
                if (can_filter) {
                    float r_ref = ref_scan->ranges[i];
                    bool ref_valid = (r_ref >= ref_scan->range_min &&
                                      r_ref <= ref_scan->range_max);
                    if (ref_valid && (r_ref - r) > dynamic_thresh_) return;
                }

                // 距离分层采样：
                //   < 1.5m  全部保留（几何约束强）
                //   1.5~4m  保留偶数索引（~50%）
                //   > 4m    保留 4 的倍数索引（~25%，远处噪声大）
                if (r >= 1.5f && r < 4.0f && (i & 1)) return;
                if (r >= 4.0f && (i % 4 != 0)) return;

                keep[i] = 1;
            });

        // Step 2: 并行计算 x/y（只处理 keep 的点）
        // 先收集保留的索引
        std::vector<int> valid_idx;
        valid_idx.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            if (keep[i]) valid_idx.push_back(static_cast<int>(i));
        }

        const int n = static_cast<int>(valid_idx.size());
        std::vector<float> xs(n), ys(n);

        // 用输出位置索引并行计算，避免指针差在 par_unseq 下的 UB
        std::vector<int> out_pos(n);
        std::iota(out_pos.begin(), out_pos.end(), 0);

        std::for_each(std::execution::par_unseq,
            out_pos.begin(), out_pos.end(),
            [&](int idx) {
                int i       = valid_idx[idx];
                float r     = scan.ranges[i];
                float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
                xs[idx] = r * std::cos(angle);
                ys[idx] = r * std::sin(angle);
            });

        // Step 3: 组装 DataPoints
        DP::Labels feat_labels;
        feat_labels.push_back(DP::Label("x", 1));
        feat_labels.push_back(DP::Label("y", 1));
        feat_labels.push_back(DP::Label("pad", 1));

        PM::Matrix feat(3, n);
        for (int i = 0; i < n; ++i) {
            feat(0, i) = xs[i];
            feat(1, i) = ys[i];
            feat(2, i) = 1.0f;
        }
        return DP(feat, feat_labels);
    }

    Pose2 run_icp(DP & ref, DP & data, const Pose2 & init_pose)
    {
        PM::TransformationParameters init = PM::TransformationParameters::Identity(3, 3);
        init.topLeftCorner<2, 2>() = init_pose.linear().cast<float>();
        init.topRightCorner<2, 1>() = init_pose.translation().cast<float>();

        PM::TransformationParameters T = icp_(data, ref, init);

        return make_pose(T(0, 2), T(1, 2), std::atan2(T(1, 0), T(0, 0)));
    }
};

} // namespace lioslam
