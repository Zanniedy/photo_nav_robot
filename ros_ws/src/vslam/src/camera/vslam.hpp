#pragma once

#include "utils/log/logger.hpp"
#include "camera/open_camera.hpp"
#include "camera/camera_parameters.hpp"
#include "camera/frame_do.hpp"
#include "camera/tracker.hpp"

#include <functional>
#include <memory>
#include <string>

namespace vslam {

class Vslam {
public:
    using PoseCallback = std::function<void(uint64_t frame_id, cv::Vec3d pos, int inliers, TrackState state, const OrbResult& vis)>;

    explicit Vslam(const std::string& config_path)
        : cam_params_(config_path)
    {
        cam_params_.load();
        const auto& intr = cam_params_.intrinsics();

        YAML::Node root = YAML::LoadFile(config_path);
        cam_ = std::make_unique<OpenCamera>(root);
        tracker_ = std::make_unique<Tracker>(intr);
        pipeline_ = std::make_unique<FramePipeline>(*cam_);

        pipeline_->set_infer_callback([this](const Frame& frame) {
            TrackState state = tracker_->update(frame);
            if (pose_cb_) {
                pose_cb_(frame.id, tracker_->position(),
                         tracker_->last_inliers(), state,
                         tracker_->curr_vis());
            }
        });
    }

    void set_pose_callback(PoseCallback cb) { pose_cb_ = std::move(cb); }

    bool start() {
        if (!cam_->start())
            return false;
        pipeline_->start();
        return true;
    }

    void stop() {
        pipeline_->stop();
        cam_->stop();
    }

private:
    CameraParameters                 cam_params_;
    std::unique_ptr<OpenCamera>      cam_;
    std::unique_ptr<Tracker>         tracker_;
    std::unique_ptr<FramePipeline>   pipeline_;
    PoseCallback                     pose_cb_;
};

} // namespace vslam
