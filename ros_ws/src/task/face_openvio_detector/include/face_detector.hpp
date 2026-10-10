#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace face {

struct FaceBox {
    float x;          // top-left x in image pixels
    float y;          // top-left y in image pixels
    float w;          // bounding box width
    float h;          // bounding box height
    float confidence; // detection score [0, 1]
};

class FaceDetector {
public:
    explicit FaceDetector(const std::string& model_path, const std::string& device = "CPU");
    ~FaceDetector();

    // Returns detected face bounding boxes in image pixel coordinates.
    // img must be BGR, as returned by cv::VideoCapture or cv::imread.
    std::vector<FaceBox> detect(const cv::Mat& img);

    void setConfidenceThreshold(float thresh);
    float confidenceThreshold() const;

private:
    struct Impl;
    Impl* impl_;
};

} // namespace face
