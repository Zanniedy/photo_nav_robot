#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_orb.hpp"
#include "camera/camera_parameters.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

namespace vslam {

struct PoseResult {
    bool    success  = false;
    cv::Mat R;
    cv::Mat t;
    int     inliers  = 0;
};

class PoseEstimator {
public:
    explicit PoseEstimator(const CameraIntrinsics& intr) : intr_(intr) {}

    PoseResult estimate(
        const OrbResult&              prev,
        const OrbResult&              curr,
        const std::vector<cv::DMatch>& matches
    ) {
        PoseResult result;
        if (matches.size() < 8) {
            AURORA_WARN("pose_estimator: too few matches ({})", matches.size());
            return result;
        }

        std::vector<cv::Point2f> pts_prev, pts_curr;
        pts_prev.reserve(matches.size());
        pts_curr.reserve(matches.size());
        for (const auto& m : matches) {
            pts_prev.push_back(prev.keypoints[m.queryIdx].pt);
            pts_curr.push_back(curr.keypoints[m.trainIdx].pt);
        }

        cv::Mat K = intr_.camera_matrix();
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(
            pts_prev, pts_curr, K,
            cv::RANSAC, 0.999, 1.0, mask
        );

        if (E.empty()) {
            AURORA_WARN("pose_estimator: findEssentialMat failed");
            return result;
        }

        cv::Mat R, t;
        int inliers = cv::recoverPose(E, pts_prev, pts_curr, K, R, t, mask);

        if (inliers < 8) {
            AURORA_WARN("pose_estimator: recoverPose inliers too few ({})", inliers);
            return result;
        }

        result.success = true;
        result.R       = R;
        result.t       = t;
        result.inliers = inliers;
        return result;
    }

private:
    CameraIntrinsics intr_;
};

} // namespace vslam
