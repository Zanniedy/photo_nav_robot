#pragma once

#include "utils/log/logger_utils.hpp"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>

namespace vslam {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    // 畸变: k1, k2, p1, p2, k3
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;

    bool is_calibrated() const {
        return fx > 0.0 && fy > 0.0;
    }

    cv::Mat camera_matrix() const {
        return (cv::Mat_<double>(3, 3)
            << fx, 0, cx,
               0, fy, cy,
               0,  0,  1);
    }

    cv::Mat dist_coeffs() const {
        return (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    }

    static CameraIntrinsics load(const YAML::Node& node) {
        CameraIntrinsics p;
        p.fx = node["fx"].as<double>(0.0);
        p.fy = node["fy"].as<double>(0.0);
        p.cx = node["cx"].as<double>(0.0);
        p.cy = node["cy"].as<double>(0.0);
        p.k1 = node["k1"].as<double>(0.0);
        p.k2 = node["k2"].as<double>(0.0);
        p.p1 = node["p1"].as<double>(0.0);
        p.p2 = node["p2"].as<double>(0.0);
        p.k3 = node["k3"].as<double>(0.0);
        return p;
    }

    void save(YAML::Node& node) const {
        node["fx"] = fx;
        node["fy"] = fy;
        node["cx"] = cx;
        node["cy"] = cy;
        node["k1"] = k1;
        node["k2"] = k2;
        node["p1"] = p1;
        node["p2"] = p2;
        node["k3"] = k3;
    }

    void log() const {
        AURORA_INFO("fx={:.4f} fy={:.4f} cx={:.4f} cy={:.4f}", fx, fy, cx, cy);
        AURORA_INFO("k1={:.6f} k2={:.6f} p1={:.6f} p2={:.6f} k3={:.6f}", k1, k2, p1, p2, k3);
    }
};

class CameraParameters {
public:
    explicit CameraParameters(const std::string& config_path) : config_path_(config_path) {}

    bool load() {
        try {
            root_ = YAML::LoadFile(config_path_);
            intrinsics_ = CameraIntrinsics::load(root_["params"]);
            AURORA_INFO("Loaded camera params from: {}", config_path_);
            intrinsics_.log();
            return true;
        } catch (const std::exception& e) {
            AURORA_ERROR("Failed to load camera params: {}", e.what());
            return false;
        }
    }

    bool save() {
        try {
            YAML::Node params_node = root_["params"];
            intrinsics_.save(params_node);
            std::ofstream fout(config_path_);
            fout << root_;
            AURORA_INFO("Saved camera params to: {}", config_path_);
            return true;
        } catch (const std::exception& e) {
            AURORA_ERROR("Failed to save camera params: {}", e.what());
            return false;
        }
    }

    const CameraIntrinsics& intrinsics() const { return intrinsics_; }
    CameraIntrinsics& intrinsics() { return intrinsics_; }

private:
    std::string      config_path_;
    YAML::Node       root_;
    CameraIntrinsics intrinsics_;
};

} // namespace vslam
