#include "camera/open_camera.hpp"
#include "utils/log/logger.hpp"

#include <opencv2/highgui.hpp>

#include <filesystem>
#include <iostream>

static const std::string CONFIG_PATH =
    std::string(std::getenv("HOME")) + "/cs_project/robot/ros_ws/src/vslam/config/camera.yaml";

int main() {
    aurora::init_logger();

    AURORA_INFO("Loading config: {}", CONFIG_PATH);

    if (!std::filesystem::exists(CONFIG_PATH)) {
        AURORA_ERROR("Config file not found: {}", CONFIG_PATH);
        return 1;
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(CONFIG_PATH);
    } catch (const std::exception& e) {
        AURORA_ERROR("Failed to parse config: {}", e.what());
        return 1;
    }

    vslam::OpenCamera cam(config);

    if (!cam.start()) {
        AURORA_ERROR("Camera failed to start");
        return 1;
    }

    AURORA_INFO("Press 'q' to quit");

    cv::Mat frame;
    int frame_count = 0;

    while (true) {
        if (!cam.read(frame)) {
            AURORA_WARN("Read failed, retrying...");
            continue;
        }

        ++frame_count;
        cv::imshow("camera_test", frame);

        if (frame_count % 60 == 0)
            AURORA_INFO("Frames captured: {}", frame_count);

        if (cv::waitKey(1) == 'q')
            break;
    }

    AURORA_INFO("Total frames captured: {}", frame_count);
    cam.stop();
    cv::destroyAllWindows();
    return 0;
}
