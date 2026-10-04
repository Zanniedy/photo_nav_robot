#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_orb.hpp"
#include "camera/camera_parameters.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <vector>

namespace vslam {

struct PoseResult {
    bool    success       = false;
    cv::Mat R;
    cv::Mat t;
    int     inliers       = 0;
    double  inlier_ratio  = 0.0;
    double  rotation_deg  = 0.0;
    double  translation   = 0.0;
};

// 重投影误差 cost functor（用于 Ceres BA 精化）
// 归一化坐标系下的单位焦距模型，外参用 angle-axis + translation 参数化
struct ReprojectionCost {
    ReprojectionCost(double obs_x, double obs_y,   // 归一化平面观测
                     double pt3_x, double pt3_y, double pt3_z)  // 三角化后的3D点（相机1系下）
        : ox(obs_x), oy(obs_y), px(pt3_x), py(pt3_y), pz(pt3_z) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose[0..2] = angle-axis,  pose[3..5] = translation
        T p[3] = { T(px), T(py), T(pz) };
        T rp[3];
        ceres::AngleAxisRotatePoint(pose, p, rp);
        rp[0] += pose[3];
        rp[1] += pose[4];
        rp[2] += pose[5];

        T xp = rp[0] / rp[2];
        T yp = rp[1] / rp[2];
        residual[0] = xp - T(ox);
        residual[1] = yp - T(oy);
        return true;
    }

    static ceres::CostFunction* Create(double ox, double oy,
                                       double px, double py, double pz) {
        return new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6>(
            new ReprojectionCost(ox, oy, px, py, pz));
    }

    double ox, oy, px, py, pz;
};

class PoseEstimator {
public:
    explicit PoseEstimator(const CameraIntrinsics& intr,
                           int    min_inliers       = 30,
                           double min_inlier_ratio  = 0.5,
                           double max_rotation_deg  = 5.0)
        : intr_(intr)
        , min_inliers_(min_inliers)
        , min_inlier_ratio_(min_inlier_ratio)
        , max_rotation_deg_(max_rotation_deg)
    {}

    PoseResult estimate(
        const OrbResult&               prev,
        const OrbResult&               curr,
        const std::vector<cv::DMatch>& matches)
    {
        PoseResult result;

        if (static_cast<int>(matches.size()) < min_inliers_) {
            AURORA_WARN("位姿估计: 匹配点太少 ({}/{})", matches.size(), min_inliers_);
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
        cv::Mat E = cv::findEssentialMat(pts_prev, pts_curr, K, cv::RANSAC, 0.999, 1.0, mask);
        if (E.empty()) {
            AURORA_WARN("位姿估计: findEssentialMat 失败");
            return result;
        }

        cv::Mat R, t;
        int inliers = cv::recoverPose(E, pts_prev, pts_curr, K, R, t, mask);

        double ratio = static_cast<double>(inliers) / static_cast<double>(matches.size());
        if (inliers < min_inliers_) {
            AURORA_WARN("位姿估计: 内点太少 ({}, 要求>={})", inliers, min_inliers_);
            return result;
        }
        if (ratio < min_inlier_ratio_) {
            AURORA_WARN("位姿估计: 内点比例太低 ({:.2f})", ratio);
            return result;
        }

        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        double rot_deg = cv::norm(rvec) * 180.0 / M_PI;
        if (rot_deg > max_rotation_deg_) {
            AURORA_WARN("位姿估计: 单帧旋转过大 ({:.2f}°)", rot_deg);
            return result;
        }

        // ── Ceres 精化：把 recoverPose 结果作为初值做 BA ──────────────
        refine_with_ceres(pts_prev, pts_curr, mask, K, R, t, inliers);

        // 精化后重新计算旋转角
        cv::Rodrigues(R, rvec);
        rot_deg = cv::norm(rvec) * 180.0 / M_PI;

        result.success      = true;
        result.R            = R;
        result.t            = t;
        result.inliers      = inliers;
        result.inlier_ratio = ratio;
        result.rotation_deg = rot_deg;
        result.translation  = cv::norm(t);

        AURORA_DEBUG("位姿估计: 内点={} 比例={:.2f} 旋转={:.3f}° |t|={:.4f}",
                     inliers, ratio, rot_deg, result.translation);
        return result;
    }

private:
    // 用内点对做三角化 + Ceres 重投影误差精化 R,t
    void refine_with_ceres(
        const std::vector<cv::Point2f>& pts1,
        const std::vector<cv::Point2f>& pts2,
        const cv::Mat&                  mask,
        const cv::Mat&                  K,
        cv::Mat&                        R,
        cv::Mat&                        t,
        int                             /*inliers*/)
    {
        double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);

        // 归一化坐标
        auto to_norm = [&](const cv::Point2f& p) -> cv::Point2d {
            return { (p.x - cx) / fx, (p.y - cy) / fy };
        };

        // P1 = K[I|0],  P2 = K[R|t]
        cv::Mat P1(3, 4, CV_64F, cv::Scalar(0));
        K.copyTo(P1.colRange(0, 3));

        cv::Mat P2(3, 4, CV_64F);
        cv::Mat Rt;
        cv::hconcat(R, t, Rt);
        P2 = K * Rt;

        // 收集内点
        std::vector<cv::Point2f> in1, in2;
        for (int i = 0; i < static_cast<int>(pts1.size()); ++i) {
            if (mask.at<uchar>(i)) {
                in1.push_back(pts1[i]);
                in2.push_back(pts2[i]);
            }
        }
        if (static_cast<int>(in1.size()) < 8) return;

        // 三角化
        cv::Mat pts4d;
        cv::triangulatePoints(P1, P2, in1, in2, pts4d);

        // 构建 pose 参数：angle-axis(3) + translation(3)
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        double pose[6] = {
            rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
            t.at<double>(0),    t.at<double>(1),    t.at<double>(2)
        };

        ceres::Problem problem;
        ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

        int added = 0;
        for (int i = 0; i < pts4d.cols; ++i) {
            float w = pts4d.at<float>(3, i);
            if (std::abs(w) < 1e-6f) continue;
            float x = pts4d.at<float>(0, i) / w;
            float y = pts4d.at<float>(1, i) / w;
            float z = pts4d.at<float>(2, i) / w;
            if (z <= 0.f || z > 50.f) continue;

            auto obs = to_norm(in2[i]);
            problem.AddResidualBlock(
                ReprojectionCost::Create(obs.x, obs.y, x, y, z),
                loss, pose);
            ++added;
        }
        if (added < 6) return;

        ceres::Solver::Options opts;
        opts.linear_solver_type           = ceres::DENSE_QR;
        opts.max_num_iterations           = 10;
        opts.minimizer_progress_to_stdout = false;
        opts.num_threads                  = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(opts, &problem, &summary);

        // 写回 R, t
        cv::Mat rv = (cv::Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);
        cv::Rodrigues(rv, R);
        t.at<double>(0) = pose[3];
        t.at<double>(1) = pose[4];
        t.at<double>(2) = pose[5];
    }

    CameraIntrinsics intr_;
    int    min_inliers_;
    double min_inlier_ratio_;
    double max_rotation_deg_;
};

} // namespace vslam
