
#pragma once

#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pointmatcher/PointMatcher.h>
#include <sstream>

namespace lioslam {

struct drobotpose {
    double dx;
    double dy;
    double dyaw;
};

using PM  = PointMatcher<float>;
using DP  = PM::DataPoints;
using ICP = PM::ICP;

class scan_match {
public:
    scan_match() {
        // 最小有效点数：太稀疏时直接拒绝
        min_points_ = 20;

        // 用 YAML 字符串配置 ICP，比 setDefault() 可控得多
        std::istringstream cfg(R"yaml(
readingDataPointsFilters:
  - RandomSamplingDataPointsFilter:
      prob: 0.75

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

    drobotpose match(const sensor_msgs::msg::LaserScan & prev,
                     const sensor_msgs::msg::LaserScan & curr)
    {
        DP ref  = scan_to_dp(prev);
        DP data = scan_to_dp(curr);
        if (ref.getNbPoints()  < min_points_ ||
            data.getNbPoints() < min_points_) {
            return {0.0, 0.0, 0.0};
        }
        return run_icp(ref, data);
    }

private:
    ICP    icp_;
    size_t min_points_;

    // Convert a 2-D LaserScan into a libpointmatcher DataPoints (x, y, pad=1)
    DP scan_to_dp(const sensor_msgs::msg::LaserScan & scan) const
    {
        std::vector<float> xs, ys;
        float angle = scan.angle_min;
        for (float r : scan.ranges) {
            if (r >= scan.range_min && r <= scan.range_max) {
                xs.push_back(r * std::cos(angle));
                ys.push_back(r * std::sin(angle));
            }
            angle += scan.angle_increment;
        }

        const int n = static_cast<int>(xs.size());
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

    drobotpose run_icp(DP & ref, DP & data)
    {
        if (ref.getNbPoints() == 0 || data.getNbPoints() == 0) {
            return {0.0, 0.0, 0.0};
        }

        PM::TransformationParameters T = icp_(data, ref);

        // T is a 3x3 homogeneous matrix for 2-D: [R | t; 0 0 1]
        drobotpose result;
        result.dx   = static_cast<double>(T(0, 2));
        result.dy   = static_cast<double>(T(1, 2));
        result.dyaw = static_cast<double>(std::atan2(T(1, 0), T(0, 0)));
        return result;
    }
};

} // namespace lioslam
