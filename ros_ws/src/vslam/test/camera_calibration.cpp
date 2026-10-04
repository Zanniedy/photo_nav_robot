#include "camera/camera_parameters.hpp"
#include "camera/open_camera.hpp"
#include "utils/log/logger.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <vector>

static const std::string CONFIG_PATH =
    std::string(std::getenv("HOME")) + "/cs_project/robot/ros_ws/src/vslam/config/camera.yaml";

// 棋盘格内角点 9x12，方格 25mm
static constexpr int   BOARD_W      = 8;
static constexpr int   BOARD_H      = 11;
static constexpr float SQUARE_MM    = 25.0f;
static constexpr int   REQUIRED_IMGS = 20;

static std::vector<cv::Point3f> make_object_points() {
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < BOARD_H; ++r)
        for (int c = 0; c < BOARD_W; ++c)
            pts.emplace_back(c * SQUARE_MM, r * SQUARE_MM, 0.f);
    return pts;
}

int main() {
    aurora::init_logger();

    vslam::CameraParameters cam_params(CONFIG_PATH);
    cam_params.load();

    YAML::Node config = YAML::LoadFile(CONFIG_PATH);
    vslam::CameraParams open_params = vslam::CameraParams::load(config);
    vslam::OpenCamera cam(open_params);

    if (!cam.start()) {
        AURORA_ERROR("Failed to start camera");
        return 1;
    }

    const cv::Size board_size(BOARD_W, BOARD_H);
    const auto     obj_pts_template = make_object_points();

    std::vector<std::vector<cv::Point3f>> all_obj_pts;
    std::vector<std::vector<cv::Point2f>> all_img_pts;

    cv::Size img_size;
    cv::Mat  frame, gray;

    AURORA_INFO("Press [SPACE] to capture, [q] to quit");
    AURORA_INFO("Need {} frames with detected corners", REQUIRED_IMGS);

    while (true) {
        if (!cam.read(frame))
            continue;

        img_size = frame.size();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
        );

        cv::Mat display = frame.clone();
        cv::drawChessboardCorners(display, board_size, corners, found);

        int captured = static_cast<int>(all_img_pts.size());
        std::string status = found
            ? "DETECTED  [SPACE to capture " + std::to_string(captured) + "/" +
                  std::to_string(REQUIRED_IMGS) + "]"
            : "searching... [" + std::to_string(captured) + "/" +
                  std::to_string(REQUIRED_IMGS) + " captured]";

        cv::putText(
            display, status, {10, 30},
            cv::FONT_HERSHEY_SIMPLEX, 0.7,
            found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2
        );

        cv::imshow("calibration", display);
        int key = cv::waitKey(1);

        if (key == 'q')
            break;

        if (key == ' ' && found) {
            cv::cornerSubPix(
                gray, corners, {11, 11}, {-1, -1},
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001)
            );
            all_obj_pts.push_back(obj_pts_template);
            all_img_pts.push_back(corners);
            AURORA_INFO("Captured {}/{}", all_img_pts.size(), REQUIRED_IMGS);

            if (static_cast<int>(all_img_pts.size()) >= REQUIRED_IMGS) {
                AURORA_INFO("Enough frames, starting calibration...");
                break;
            }
        }
    }

    cv::destroyAllWindows();
    cam.stop();

    if (static_cast<int>(all_img_pts.size()) < REQUIRED_IMGS) {
        AURORA_WARN("Only {} frames captured, need {}. Calibration aborted.",
            all_img_pts.size(), REQUIRED_IMGS);
        return 1;
    }

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        all_obj_pts, all_img_pts, img_size,
        K, D, rvecs, tvecs
    );

    AURORA_INFO("Calibration done. RMS reprojection error: {:.4f} px", rms);
    AURORA_INFO("K =\n  fx={:.4f} fy={:.4f} cx={:.4f} cy={:.4f}",
        K.at<double>(0,0), K.at<double>(1,1),
        K.at<double>(0,2), K.at<double>(1,2));
    AURORA_INFO("D = k1={:.6f} k2={:.6f} p1={:.6f} p2={:.6f} k3={:.6f}",
        D.at<double>(0), D.at<double>(1),
        D.at<double>(2), D.at<double>(3),
        D.at<double>(4));

    auto& intr = cam_params.intrinsics();
    intr.fx = K.at<double>(0, 0);
    intr.fy = K.at<double>(1, 1);
    intr.cx = K.at<double>(0, 2);
    intr.cy = K.at<double>(1, 2);
    intr.k1 = D.at<double>(0);
    intr.k2 = D.at<double>(1);
    intr.p1 = D.at<double>(2);
    intr.p2 = D.at<double>(3);
    intr.k3 = D.at<double>(4);

    if (cam_params.save()) {
        AURORA_INFO("Parameters saved to: {}", CONFIG_PATH);
    }

    return 0;
}
