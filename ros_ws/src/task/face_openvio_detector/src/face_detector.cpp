#include "face_detector.hpp"

#include <openvino/openvino.hpp>
#include <stdexcept>

namespace face {

// ---------------------------------------------------------------------------
// Impl — fill in the detection logic here
// ---------------------------------------------------------------------------
struct FaceDetector::Impl {
    ov::Core core;
    ov::CompiledModel compiled_model;
    float conf_thresh = 0.5f;

    Impl(const std::string& model_path, const std::string& device) {
        auto model = core.read_model(model_path);

        // TODO: configure pre/post-processing to match your model's requirements
        //   e.g. ov::preprocess::PrePostProcessor, set layout, element type, etc.

        compiled_model = core.compile_model(model, device);
    }

    std::vector<FaceBox> detect(const cv::Mat& img) {
        std::vector<FaceBox> results;
        if (img.empty()) return results;

        // TODO: implement inference
        //   1. Resize / preprocess img into a cv::Mat that matches model input
        //   2. Wrap in ov::Tensor and call infer_request.infer()
        //   3. Parse output tensors into FaceBox entries
        //   4. Filter by conf_thresh and push_back to results

        return results;
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
FaceDetector::FaceDetector(const std::string& model_path, const std::string& device)
    : impl_(new Impl(model_path, device)) {}

FaceDetector::~FaceDetector() {
    delete impl_;
}

std::vector<FaceBox> FaceDetector::detect(const cv::Mat& img) {
    return impl_->detect(img);
}

void FaceDetector::setConfidenceThreshold(float thresh) {
    impl_->conf_thresh = thresh;
}

float FaceDetector::confidenceThreshold() const {
    return impl_->conf_thresh;
}

} // namespace face
