#include "camera/camera_orb.hpp"
#include "camera/frame_do.hpp"
#include "camera/open_camera.hpp"
#include "utils/log/logger.hpp"

#include <opencv2/highgui.hpp>

static const std::string CONFIG_PATH =
    std::string(std::getenv("HOME")) + "/cs_project/robot/ros_ws/src/vslam/config/camera.yaml";

int main() {
    aurora::init_logger();

    YAML::Node config = YAML::LoadFile(CONFIG_PATH);
    vslam::OpenCamera cam(config);

    if (!cam.start()) {
        AURORA_ERROR("Failed to start camera");
        return 1;
    }

    vslam::CameraOrb orb;
    vslam::OrbResult prev;

    vslam::FramePipeline pipeline(cam);
    pipeline.set_infer_callback([&](const vslam::Frame& frame) {
        auto result = orb.extract(frame);

        cv::Mat vis;
        if (!prev.descriptors.empty()) {
            auto matches = orb.match(prev, result);
            vis = orb.draw_matches(prev, result, matches);
            AURORA_INFO("frame={} kps={} matches={}", frame.id, result.keypoints.size(), matches.size());
        } else {
            vis = orb.draw_keypoints(frame, result);
            AURORA_INFO("frame={} kps={}", frame.id, result.keypoints.size());
        }

        prev = std::move(result);

        cv::imshow("orb", vis);
        cv::waitKey(1);
    });

    pipeline.start();
    AURORA_INFO("Press [Enter] to stop");
    std::cin.get();

    pipeline.stop();
    cam.stop();
    cv::destroyAllWindows();
    return 0;
}
