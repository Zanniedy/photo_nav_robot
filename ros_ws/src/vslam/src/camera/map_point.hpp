#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_parameters.hpp"
#include "camera/camera_orb.hpp"
#include "camera/pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <cstdint>
#include <vector>

namespace vslam {

struct MapPoint {
    uint64_t      id;
    cv::Point3f   pos;
};

inline std::vector<MapPoint> triangulate(
    const cv::Mat&                  R1, const cv::Mat& t1,
    const cv::Mat&                  R2, const cv::Mat& t2,
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const CameraIntrinsics&         intr
) {
    cv::Mat K = intr.camera_matrix();

    // 投影矩阵 P = K * [R | t]
    cv::Mat P1(3, 4, CV_64F), P2(3, 4, CV_64F);
    R1.copyTo(P1.colRange(0, 3));
    t1.copyTo(P1.col(3));
    P1 = K * P1;

    R2.copyTo(P2.colRange(0, 3));
    t2.copyTo(P2.col(3));
    P2 = K * P2;

    cv::Mat pts4d;
    cv::triangulatePoints(P1, P2, pts1, pts2, pts4d);

    std::vector<MapPoint> result;
    result.reserve(pts4d.cols);

    static uint64_t id_counter = 0;
    for (int i = 0; i < pts4d.cols; ++i) {
        float w = pts4d.at<float>(3, i);
        if (std::abs(w) < 1e-6f)
            continue;

        float x = pts4d.at<float>(0, i) / w;
        float y = pts4d.at<float>(1, i) / w;
        float z = pts4d.at<float>(2, i) / w;

        // 过滤深度为负或过远的点
        if (z <= 0.f || z > 50.f)
            continue;

        result.push_back({ ++id_counter, cv::Point3f(x, y, z) });
    }

    AURORA_DEBUG("三角化: {}/{} 个点有效", result.size(), pts4d.cols);
    return result;
}

} // namespace vslam
