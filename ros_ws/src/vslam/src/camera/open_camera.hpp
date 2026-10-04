#pragma once

#include "utils/log/logger_utils.hpp"
#include "camera/camera_parameters.hpp"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <string>

namespace vslam {

struct CameraParams {
    std::string device;
    int width          = 640;
    int height         = 720;
    int fps            = 30;
    std::string format = "YU12";
    int buffers        = 4;
    double exposure    = 100.0;
    bool auto_exposure = false;

    CameraIntrinsics intrinsics;

    static CameraParams load(const YAML::Node& root) {
        CameraParams p;
        const auto& cam = root["camera"];
        p.device        = cam["device"].as<std::string>();
        p.width         = cam["width"].as<int>();
        p.height        = cam["height"].as<int>();
        p.fps           = cam["fps"].as<int>();
        p.format        = cam["format"].as<std::string>("YU12");
        p.buffers       = cam["buffers"].as<int>(4);
        p.exposure      = cam["exposure"].as<double>(100.0);
        p.auto_exposure = cam["auto_exposure"].as<bool>(false);

        if (root["params"])
            p.intrinsics = CameraIntrinsics::load(root["params"]);

        return p;
    }
};

class OpenCamera {
public:
    explicit OpenCamera(const CameraParams& params) : params_(params) {}

    explicit OpenCamera(const YAML::Node& root) : params_(CameraParams::load(root)) {}

    ~OpenCamera() { stop(); }

    bool start() {
        if (running_)
            return true;

        AURORA_INFO("Opening camera: {}", params_.device);

        if (!cap_.open(params_.device, cv::CAP_V4L2)) {
            AURORA_ERROR("Failed to open camera: {}", params_.device);
            return false;
        }

        set_and_check(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'), "FOURCC");
        set_and_check(cv::CAP_PROP_FRAME_WIDTH,  params_.width,  "WIDTH");
        set_and_check(cv::CAP_PROP_FRAME_HEIGHT, params_.height, "HEIGHT");
        set_and_check(cv::CAP_PROP_FPS,          params_.fps,    "FPS");
        set_and_check(cv::CAP_PROP_AUTO_EXPOSURE, params_.auto_exposure ? 3.0 : 1.0, "AUTO_EXPOSURE");
        if (!params_.auto_exposure)
            set_and_check(cv::CAP_PROP_EXPOSURE, params_.exposure, "EXPOSURE");

        if (params_.intrinsics.is_calibrated()) {
            AURORA_INFO("Intrinsics loaded: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}",
                params_.intrinsics.fx, params_.intrinsics.fy,
                params_.intrinsics.cx, params_.intrinsics.cy);
        } else {
            AURORA_WARN("Camera not calibrated, intrinsics are zero");
        }

        running_ = true;
        AURORA_INFO("Camera started: {}x{} @ {} fps", params_.width, params_.height, params_.fps);
        return true;
    }

    void stop() {
        if (!running_)
            return;
        running_ = false;
        cap_.release();
        AURORA_INFO("Camera stopped: {}", params_.device);
    }

    bool read(cv::Mat& frame) {
        if (!cap_.isOpened()) {
            AURORA_ERROR("Camera is not open");
            return false;
        }
        if (!cap_.read(frame)) {
            AURORA_WARN("Failed to read frame from: {}", params_.device);
            return false;
        }
        return true;
    }

    // 读帧并去畸变，需要已标定
    bool read_undistorted(cv::Mat& frame) {
        cv::Mat raw;
        if (!read(raw))
            return false;
        if (!params_.intrinsics.is_calibrated()) {
            frame = raw;
            return true;
        }
        cv::undistort(raw, frame,
            params_.intrinsics.camera_matrix(),
            params_.intrinsics.dist_coeffs());
        return true;
    }

    bool is_running() const { return running_; }

    const CameraParams&    params()     const { return params_; }
    const CameraIntrinsics& intrinsics() const { return params_.intrinsics; }

    void set_exposure(double exposure) {
        params_.exposure = exposure;
        if (cap_.isOpened())
            set_and_check(cv::CAP_PROP_EXPOSURE, exposure, "EXPOSURE");
    }

private:
    bool set_and_check(int prop, double value, const std::string& name, double tol = 1e-3) {
        if (!cap_.set(prop, value)) {
            AURORA_WARN("{} set() failed", name);
            return false;
        }
        double actual = cap_.get(prop);
        if (std::abs(actual - value) > tol) {
            AURORA_WARN("{} mismatch: requested={} actual={}", name, value, actual);
            return false;
        }
        AURORA_INFO("{} = {}", name, actual);
        return true;
    }

    CameraParams     params_;
    cv::VideoCapture cap_;
    std::atomic_bool running_ { false };
};

} // namespace vslam
