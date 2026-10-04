#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/frame_do.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

namespace vslam {

struct OrbParams {
    int   n_features     = 500;
    float scale_factor   = 1.2f;
    int   n_levels       = 8;
    int   edge_threshold = 31;
    int   fast_threshold = 20;

    static OrbParams load(const YAML::Node& node) {
        OrbParams p;
        p.n_features     = node["n_features"].as<int>(500);
        p.scale_factor   = node["scale_factor"].as<float>(1.2f);
        p.n_levels       = node["n_levels"].as<int>(8);
        p.edge_threshold = node["edge_threshold"].as<int>(31);
        p.fast_threshold = node["fast_threshold"].as<int>(20);
        return p;
    }
};

struct OrbResult {
    uint64_t                    frame_id;
    cv::Mat                     image;
    std::vector<cv::KeyPoint>   keypoints;
    cv::Mat                     descriptors;
};

class CameraOrb {
public:
    explicit CameraOrb(const OrbParams& params = OrbParams{}) : params_(params) {
        orb_ = cv::ORB::create(
            params_.n_features,
            params_.scale_factor,
            params_.n_levels,
            params_.edge_threshold,
            0, 2, cv::ORB::HARRIS_SCORE,
            31,
            params_.fast_threshold
        );
        AURORA_INFO("ORB created: features={} levels={} scale={:.2f}",
            params_.n_features, params_.n_levels, params_.scale_factor);
    }

    // 单帧提取
    OrbResult extract(const Frame& frame) {
        OrbResult result;
        result.frame_id = frame.id;
        result.image    = frame.image.clone();

        cv::Mat gray;
        if (frame.image.channels() == 3)
            cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = frame.image;

        orb_->detectAndCompute(gray, cv::noArray(), result.keypoints, result.descriptors);

        AURORA_DEBUG("frame id={} keypoints={}", frame.id, result.keypoints.size());
        return result;
    }

    // 两帧匹配，返回筛选后的匹配对
    std::vector<cv::DMatch> match(const OrbResult& a, const OrbResult& b, float ratio = 0.75f) {
        if (a.descriptors.empty() || b.descriptors.empty())
            return {};

        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher_->knnMatch(a.descriptors, b.descriptors, knn_matches, 2);

        std::vector<cv::DMatch> good;
        good.reserve(knn_matches.size());
        for (const auto& m : knn_matches) {
            if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
                good.push_back(m[0]);
        }

        AURORA_DEBUG("match: raw={} good={}", knn_matches.size(), good.size());
        return good;
    }

    // 可视化关键点
    cv::Mat draw_keypoints(const Frame& frame, const OrbResult& result) {
        cv::Mat out;
        cv::drawKeypoints(frame.image, result.keypoints, out,
            cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::putText(out,
            "id=" + std::to_string(result.frame_id) +
            " kps=" + std::to_string(result.keypoints.size()),
            {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        return out;
    }

    // 可视化匹配
    cv::Mat draw_matches(
        const OrbResult& ra,
        const OrbResult& rb,
        const std::vector<cv::DMatch>& matches
    ) {
        cv::Mat out;
        cv::drawMatches(ra.image, ra.keypoints, rb.image, rb.keypoints,
            matches, out, cv::Scalar::all(-1), cv::Scalar::all(-1),
            {}, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        return out;
    }

    const OrbParams& params() const { return params_; }

private:
    OrbParams              params_;
    cv::Ptr<cv::ORB>    orb_;
    cv::Ptr<cv::BFMatcher> matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
};

} // namespace vslam
