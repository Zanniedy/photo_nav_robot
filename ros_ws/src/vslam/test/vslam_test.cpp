#include "camera/vslam.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <string>

static const std::string CONFIG_PATH =
    std::string(std::getenv("HOME")) + "/cs_project/robot/ros_ws/src/vslam/config/camera.yaml";

static const char* state_str(vslam::TrackState s) {
    switch (s) {
        case vslam::TrackState::INIT:     return "INIT";
        case vslam::TrackState::TRACKING: return "TRACKING";
        case vslam::TrackState::LOST:     return "LOST";
    }
    return "UNKNOWN";
}

int main() {
    aurora::init_logger();

    vslam::Vslam slam(CONFIG_PATH);

    slam.set_pose_callback([](
        uint64_t frame_id,
        cv::Vec3d pos,
        int inliers,
        vslam::TrackState state,
        const vslam::OrbResult& vis
    ) {
        std::printf("[frame=%4lu] x=%7.3f  y=%7.3f  z=%7.3f  inliers=%3d  state=%s\n",
            static_cast<unsigned long>(frame_id),
            pos[0], pos[1], pos[2],
            inliers, state_str(state));
        std::fflush(stdout);

        if (!vis.image.empty()) {
            cv::Mat out;
            cv::drawKeypoints(vis.image, vis.keypoints, out,
                cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
            cv::putText(out,
                std::string("state=") + state_str(state) +
                "  kps=" + std::to_string(vis.keypoints.size()),
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                state == vslam::TrackState::TRACKING
                    ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            cv::imshow("vslam", out);
            cv::waitKey(1);
        }
    });

    if (!slam.start()) {
        AURORA_ERROR("Failed to start VSLAM");
        return 1;
    }

    AURORA_INFO("VSLAM running. Press [Enter] to stop.");
    std::cin.get();

    slam.stop();
    cv::destroyAllWindows();
    return 0;
}
