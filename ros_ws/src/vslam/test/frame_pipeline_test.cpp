#include "camera/frame_do.hpp"
#include "camera/open_camera.hpp"
#include "utils/log/logger.hpp"

#include <opencv2/highgui.hpp>

#include <chrono>

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

    vslam::FramePipeline pipeline(cam);

    pipeline.set_infer_callback([](const vslam::Frame& frame) {
        AURORA_INFO("infer frame id={} size={}x{}",
            frame.id, frame.image.cols, frame.image.rows);

        cv::imshow("pipeline", frame.image);
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
