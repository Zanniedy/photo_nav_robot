#include "camera/vslam.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef HAVE_PANGOLIN
#include "camera/trajectory_viewer.hpp"
#endif

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

static const std::string CONFIG_PATH =
    std::string(std::getenv("HOME")) + "/cs_project/robot/ros_ws/src/vslam/config/camera.yaml";

static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running = false; }

static const char* state_str(vslam::TrackState s) {
    switch (s) {
        case vslam::TrackState::INIT:          return "初始化";
        case vslam::TrackState::TRACKING_GOOD: return "跟踪[好]";
        case vslam::TrackState::TRACKING_BAD:  return "跟踪[差]";
        case vslam::TrackState::LOST:          return "丢失";
    }
    return "未知";
}

int main() {
    aurora::init_logger();

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

#ifdef HAVE_PANGOLIN
    auto viewer = std::make_unique<vslam::TrajectoryViewer>();
    viewer->start();
#endif

    vslam::Vslam slam(CONFIG_PATH);

    slam.set_pose_callback([&](
        uint64_t                frame_id,
        cv::Vec3d               pos,
        vslam::TrackState       state,
        const vslam::FrameDiag& diag,
        const vslam::OrbResult& vis
    ) {
#ifdef HAVE_PANGOLIN
        viewer->push(pos[0], pos[1], pos[2]);
#endif
        std::printf(
            "[%4lu] %-8s  累计(x=%7.3f y=%7.3f z=%7.3f)"
            "  追踪=%3d KF位移=%5.1fpx 内点=%3d(%.0f%%)"
            "  旋转=%5.2f°  |t|=%.4f%s\n",
            static_cast<unsigned long>(frame_id),
            state_str(state),
            pos[0], pos[1], pos[2],
            diag.tracked, diag.kf_disp,
            diag.inliers,
            diag.inlier_ratio * 100.0,
            diag.rotation_deg,
            diag.translation,
            diag.is_keyframe ? "  [KF]" : "");
        std::fflush(stdout);

        if (!vis.image.empty()) {
            cv::Mat out;
            cv::drawKeypoints(vis.image, vis.keypoints, out,
                cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
            cv::putText(out,
                std::string(state_str(state)) +
                "  kps=" + std::to_string(vis.keypoints.size()) +
                "  in=" + std::to_string(diag.inliers),
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                (state == vslam::TrackState::TRACKING_GOOD ||
                 state == vslam::TrackState::TRACKING_BAD)
                    ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            cv::imshow("vslam", out);
            cv::waitKey(1);
        }
    });

    if (!slam.start()) {
        AURORA_ERROR("启动 VSLAM 失败");
        return 1;
    }

    AURORA_INFO("VSLAM 运行中，Ctrl+C 停止。");
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    slam.stop();
#ifdef HAVE_PANGOLIN
    viewer->stop();
#endif
    cv::destroyAllWindows();
    return 0;
}
