#pragma once

#include "utils/log/logger_utils.hpp"
#include "open_camera.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace vslam {

struct Frame {
    uint64_t  id;
    cv::Mat   image;
};

class FramePipeline {
public:
    using InferCallback = std::function<void(const Frame&)>;

    explicit FramePipeline(OpenCamera& camera, std::size_t max_queue = 8)
        : camera_(camera), max_queue_(max_queue) {}

    ~FramePipeline() { stop(); }

    void set_infer_callback(InferCallback cb) { infer_cb_ = std::move(cb); }

    void start() {
        if (running_)
            return;
        running_ = true;
        capture_thread_ = std::thread(&FramePipeline::capture_loop, this);
        infer_thread_   = std::thread(&FramePipeline::infer_loop,   this);
        AURORA_INFO("FramePipeline started");
    }

    void stop() {
        if (!running_)
            return;
        running_ = false;
        cv_.notify_all();
        if (capture_thread_.joinable()) capture_thread_.join();
        if (infer_thread_.joinable())   infer_thread_.join();
        AURORA_INFO("FramePipeline stopped, total frames: {}", frame_counter_.load());
    }

    std::size_t queue_size() const {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

private:
    void capture_loop() {
        while (running_) {
            cv::Mat img;
            if (!camera_.read(img))
                continue;

            Frame frame { ++frame_counter_, std::move(img) };

            {
                std::lock_guard lock(mutex_);
                if (queue_.size() >= max_queue_) {
                    // 队列满时丢弃最老的帧
                    AURORA_WARN("Queue full, dropping frame id={}", queue_.front().id);
                    queue_.pop_front();
                }
                queue_.push_back(std::move(frame));
            }
            cv_.notify_one();
        }
    }

    void infer_loop() {
        while (running_) {
            Frame frame;
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
                if (!running_ && queue_.empty())
                    break;
                frame = std::move(queue_.front());
                queue_.pop_front();
            }

            if (infer_cb_)
                infer_cb_(frame);
        }
    }

    OpenCamera&             camera_;
    std::size_t             max_queue_;
    InferCallback           infer_cb_;

    std::deque<Frame>       queue_;
    mutable std::mutex      mutex_;
    std::condition_variable cv_;

    std::atomic<uint64_t>   frame_counter_ { 0 };
    std::atomic_bool        running_       { false };
    std::thread             capture_thread_;
    std::thread             infer_thread_;
};

} // namespace vslam
