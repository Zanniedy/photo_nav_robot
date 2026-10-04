#pragma once

#include "utils/log/logger_utils.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wextra"
#include <pangolin/pangolin.h>
#pragma GCC diagnostic pop

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace vslam {

// 轨迹可视化窗口，单独线程运行
class TrajectoryViewer {
public:
    TrajectoryViewer() = default;

    ~TrajectoryViewer() { stop(); }

    void start() {
        running_ = true;
        thread_  = std::thread(&TrajectoryViewer::render_loop, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    // 主线程调用：更新当前位置
    void push(double x, double y, double z) {
        std::lock_guard<std::mutex> lk(mu_);
        traj_.push_back({x, y, z});
        cur_ = {x, y, z};
    }

private:
    struct Pt3 { double x, y, z; };

    void render_loop() {
        pangolin::CreateWindowAndBind("VSLAM Trajectory", 1024, 768);
        glEnable(GL_DEPTH_TEST);

        pangolin::OpenGlRenderState cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, pangolin::AxisNegY));

        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(cam));

        while (running_ && !pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

            d_cam.Activate(cam);

            // 画坐标轴
            glLineWidth(2);
            glBegin(GL_LINES);
            glColor3f(1, 0, 0); glVertex3f(0,0,0); glVertex3f(1,0,0);
            glColor3f(0, 1, 0); glVertex3f(0,0,0); glVertex3f(0,1,0);
            glColor3f(0, 0, 1); glVertex3f(0,0,0); glVertex3f(0,0,1);
            glEnd();

            // 画轨迹
            std::vector<Pt3> traj_copy;
            Pt3 cur_copy;
            {
                std::lock_guard<std::mutex> lk(mu_);
                traj_copy = traj_;
                cur_copy  = cur_;
            }

            glLineWidth(2);
            glColor3f(0.2f, 0.8f, 0.2f);
            glBegin(GL_LINE_STRIP);
            for (const auto& p : traj_copy)
                glVertex3d(p.x, p.y, p.z);
            glEnd();

            // 当前位置画红点
            glPointSize(8);
            glColor3f(1.0f, 0.2f, 0.2f);
            glBegin(GL_POINTS);
            glVertex3d(cur_copy.x, cur_copy.y, cur_copy.z);
            glEnd();

            pangolin::FinishFrame();
        }
        running_ = false;
    }

    std::atomic<bool>    running_{false};
    std::thread          thread_;
    std::mutex           mu_;
    std::vector<Pt3>     traj_;
    Pt3                  cur_{0, 0, 0};
};

} // namespace vslam
