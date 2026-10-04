#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_orb.hpp"
#include "camera/camera_parameters.hpp"
#include "camera/pose_estimator.hpp"
#include "camera/frame_do.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <vector>

namespace vslam {

enum class TrackState { INIT, TRACKING_GOOD, TRACKING_BAD, LOST };

struct FrameDiag {
    int    tracked      = 0;
    int    inliers      = 0;
    double inlier_ratio = 0.0;
    double rotation_deg = 0.0;
    double translation  = 0.0;
    double kf_disp      = 0.0;  // 当前帧相对关键帧的平均像素位移
    bool   is_keyframe  = false;
};

class Tracker {
public:
    explicit Tracker(const CameraIntrinsics& intr,
                     int    min_tracked_good  = 50,
                     int    min_tracked_bad   = 20,
                     int    min_kf_features   = 80,
                     double min_kf_disp_px    = 15.0)  // 关键帧基线门槛（像素）
        : estimator_(intr)
        , min_tracked_good_(min_tracked_good)
        , min_tracked_bad_(min_tracked_bad)
        , min_kf_features_(min_kf_features)
        , min_kf_disp_px_(min_kf_disp_px)
    {
        gftt_ = cv::GFTTDetector::create(500, 0.01, 20);
        reset_pose();
    }

    TrackState update(const Frame& frame) {
        cv::Mat curr_gray;
        if (frame.image.channels() == 3)
            cv::cvtColor(frame.image, curr_gray, cv::COLOR_BGR2GRAY);
        else
            curr_gray = frame.image.clone();

        last_diag_ = FrameDiag{};

        // ── INIT ─────────────────────────────────────────────────────────
        if (state_ == TrackState::INIT) {
            detect_gftt(curr_gray, prev_pts_);
            if (static_cast<int>(prev_pts_.size()) < 50) {
                AURORA_WARN("跟踪器: 初始化 GFTT 点不足 ({})", prev_pts_.size());
                return TrackState::INIT;
            }
            prev_kf_orb_ = orb_.extract(frame);
            kf_pts_      = prev_pts_;   // 保存关键帧时的点坐标
            prev_gray_   = curr_gray;
            state_       = TrackState::TRACKING_GOOD;
            lost_count_  = 0;
            AURORA_INFO("跟踪器: 第 {} 帧初始化成功, GFTT={}", frame.id, prev_pts_.size());
            return state_;
        }

        // ── LK 光流追踪（prev → curr）────────────────────────────────────
        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar>       status;
        std::vector<float>       err;
        cv::calcOpticalFlowPyrLK(
            prev_gray_, curr_gray, prev_pts_, curr_pts,
            status, err, cv::Size(21, 21), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01));

        // 同时过滤 prev_pts、curr_pts、kf_pts（三者一一对应）
        std::vector<cv::Point2f> good_prev, good_curr, good_kf;
        good_prev.reserve(prev_pts_.size());
        good_curr.reserve(prev_pts_.size());
        good_kf.reserve(kf_pts_.size());
        for (std::size_t i = 0; i < status.size(); ++i) {
            if (status[i] && i < kf_pts_.size()) {
                good_prev.push_back(prev_pts_[i]);
                good_curr.push_back(curr_pts[i]);
                good_kf.push_back(kf_pts_[i]);
            }
        }
        int count = static_cast<int>(good_curr.size());
        last_diag_.tracked = count;

        // ── 状态判断 ─────────────────────────────────────────────────────
        if (count >= min_tracked_good_) {
            state_ = TrackState::TRACKING_GOOD;
        } else if (count >= min_tracked_bad_) {
            state_ = TrackState::TRACKING_BAD;
        } else {
            AURORA_WARN("跟踪器: 第 {} 帧丢失 (追踪点={})", frame.id, count);
            state_ = TrackState::LOST;
            ++lost_count_;
            if (lost_count_ > 30) {
                detect_gftt(curr_gray, prev_pts_);
                if (static_cast<int>(prev_pts_.size()) >= 50) {
                    prev_kf_orb_ = orb_.extract(frame);
                    kf_pts_      = prev_pts_;
                    prev_gray_   = curr_gray;
                    lost_count_  = 0;
                    AURORA_INFO("跟踪器: 连续丢失 30 帧后强制重初始化");
                }
            }
            return TrackState::LOST;
        }

        // ── 关键帧基线检查：量当前点相对关键帧点的累计位移 ──────────────
        double kf_disp = 0.0;
        for (std::size_t i = 0; i < good_curr.size(); ++i) {
            double dx = good_curr[i].x - good_kf[i].x;
            double dy = good_curr[i].y - good_kf[i].y;
            kf_disp += std::sqrt(dx * dx + dy * dy);
        }
        kf_disp /= static_cast<double>(good_curr.size());
        last_diag_.kf_disp = kf_disp;

        prev_pts_  = good_curr;
        prev_gray_ = curr_gray;

        if (kf_disp < min_kf_disp_px_) {
            // 基线不够，不做 pose 估计
            return state_;
        }

        // ── ORB 匹配 + 位姿估计（关键帧 → 当前帧）───────────────────────
        OrbResult curr_orb = orb_.extract(frame);
        auto matches = orb_.match(prev_kf_orb_, curr_orb);

        if (static_cast<int>(matches.size()) < 8) {
            AURORA_WARN("跟踪器: 第 {} 帧 ORB 匹配不足 ({}) kf_disp={:.1f}px",
                        frame.id, matches.size(), kf_disp);
            ++lost_count_;
            return state_;
        }

        PoseResult pose = estimator_.estimate(prev_kf_orb_, curr_orb, matches);
        if (!pose.success) {
            AURORA_WARN("跟踪器: 第 {} 帧位姿估计失败 kf_disp={:.1f}px", frame.id, kf_disp);
            ++lost_count_;
            return state_;
        }

        last_inliers_           = pose.inliers;
        last_diag_.inliers      = pose.inliers;
        last_diag_.inlier_ratio = pose.inlier_ratio;
        last_diag_.rotation_deg = pose.rotation_deg;
        last_diag_.translation  = pose.translation;
        lost_count_ = 0;

        // 累积位姿
        global_t_ = global_R_ * pose.t + global_t_;
        global_R_ = pose.R * global_R_;

        // ── 关键帧更新 ───────────────────────────────────────────────────
        // 追踪点不足 OR 已经做完一次 pose 估计（避免对同一关键帧反复估计漂移）
        if (count < min_kf_features_ || pose.success) {
            detect_gftt(curr_gray, prev_pts_);
            kf_pts_                = prev_pts_;
            prev_kf_orb_           = curr_orb;
            last_diag_.is_keyframe = true;
            AURORA_INFO("跟踪器: 第 {} 帧插入关键帧 (追踪={}, kf_disp={:.1f}px, 内点={})",
                        frame.id, count, kf_disp, pose.inliers);
        }

        return state_;
    }

    cv::Vec3d position() const {
        return { global_t_.at<double>(0), global_t_.at<double>(1), global_t_.at<double>(2) };
    }
    TrackState       state()        const { return state_; }
    int              last_inliers() const { return last_inliers_; }
    const FrameDiag& last_diag()    const { return last_diag_; }
    const OrbResult& curr_kf_orb()  const { return prev_kf_orb_; }

private:
    void detect_gftt(const cv::Mat& gray, std::vector<cv::Point2f>& pts) {
        std::vector<cv::KeyPoint> kps;
        gftt_->detect(gray, kps);
        pts.clear();
        pts.reserve(kps.size());
        for (const auto& kp : kps)
            pts.push_back(kp.pt);
    }

    void reset_pose() {
        global_R_ = cv::Mat::eye(3, 3, CV_64F);
        global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    }

    CameraOrb                 orb_;
    PoseEstimator             estimator_;
    cv::Ptr<cv::GFTTDetector> gftt_;

    int    min_tracked_good_;
    int    min_tracked_bad_;
    int    min_kf_features_;
    double min_kf_disp_px_;

    cv::Mat                  prev_gray_;
    std::vector<cv::Point2f> prev_pts_;   // 上一帧追踪点
    std::vector<cv::Point2f> kf_pts_;    // 当前关键帧时对应的点坐标（与prev_pts_一一对应）
    OrbResult                prev_kf_orb_;

    TrackState state_        = TrackState::INIT;
    int        lost_count_   = 0;
    int        last_inliers_ = 0;
    FrameDiag  last_diag_;

    cv::Mat    global_R_;
    cv::Mat    global_t_;
};

} // namespace vslam
