#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_orb.hpp"
#include "camera/camera_parameters.hpp"
#include "camera/pose_estimator.hpp"
#include "camera/map_point.hpp"
#include "camera/frame_do.hpp"

#include <opencv2/opencv.hpp>

namespace vslam {

enum class TrackState { INIT, TRACKING, LOST };

class Tracker {
public:
    explicit Tracker(const CameraIntrinsics& intr)
        : intr_(intr), orb_(), estimator_(intr) {
        reset();
    }

    // 每帧调用，返回当前状态
    TrackState update(const Frame& frame) {
        OrbResult curr = orb_.extract(frame);
        curr_vis_ = curr;

        if (state_ == TrackState::INIT) {
            if (curr.keypoints.size() >= 50) {
                prev_ = std::move(curr);
                state_ = TrackState::TRACKING;
                AURORA_INFO("tracker: initialized at frame {}", frame.id);
            }
            return state_;
        }

        auto matches = orb_.match(prev_, curr);
        if (static_cast<int>(matches.size()) < 10) {
            AURORA_WARN("tracker: lost at frame {} (matches={})", frame.id, matches.size());
            state_ = TrackState::LOST;
            reset();
            prev_ = std::move(curr);
            state_ = TrackState::TRACKING;
            return TrackState::LOST;
        }

        PoseResult pose = estimator_.estimate(prev_, curr, matches);
        if (!pose.success) {
            AURORA_WARN("tracker: pose estimation failed at frame {}", frame.id);
            state_ = TrackState::LOST;
            reset();
            prev_ = std::move(curr);
            state_ = TrackState::TRACKING;
            return TrackState::LOST;
        }

        last_inliers_ = pose.inliers;

        // 累积位姿：t_world = R_world * t_rel + t_world
        global_t_ = global_R_ * pose.t + global_t_;
        global_R_ = pose.R * global_R_;

        prev_ = std::move(curr);
        state_ = TrackState::TRACKING;
        return state_;
    }

    cv::Vec3d position() const {
        return cv::Vec3d(
            global_t_.at<double>(0),
            global_t_.at<double>(1),
            global_t_.at<double>(2)
        );
    }

    TrackState     state()        const { return state_; }
    int            last_inliers() const { return last_inliers_; }
    const OrbResult& curr_vis()   const { return curr_vis_; }

private:
    void reset() {
        global_R_ = cv::Mat::eye(3, 3, CV_64F);
        global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    }

    CameraIntrinsics intr_;
    CameraOrb        orb_;
    PoseEstimator    estimator_;

    OrbResult        prev_;
    OrbResult        curr_vis_;
    TrackState       state_       = TrackState::INIT;
    int              last_inliers_ = 0;

    cv::Mat          global_R_;
    cv::Mat          global_t_;
};

} // namespace vslam
