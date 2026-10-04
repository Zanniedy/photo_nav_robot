#include "robot_nav/pcd_map_receiver.hpp"
#include "robot_nav/point_cloud_mapper.hpp"
#include "robot_nav/point_cloud_pose_transformer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <csignal>
#include <thread>

// ── 全局信号标志，Ctrl+C 触发保存 ─────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};

static void signal_handler(int) { g_shutdown = true; }

// ── 计算线程 ───────────────────────────────────────────────────────────────
static void mapping_loop(
    std::shared_ptr<pcdmap::PcdMapReceiver>  node,
    std::shared_ptr<tf2_ros::Buffer>         tf_buf,
    const std::string &                      map_frame,
    const std::string &                      output_path,
    double                                   voxel_size)
{
    pcdmap::PoseTransformer transformer(tf_buf);
    pcdmap::PointCloudMapper mapper(voxel_size);

    RCLCPP_INFO(node->get_logger(), "Mapping thread started. Ctrl+C to save.");

    while (!g_shutdown) {
        auto scans = node->drain_scans();

        for (const auto & s : scans) {
            auto cloud = transformer.transform_scan(s.msg, map_frame);
            if (!cloud) continue;          // TF 失败跳过
            mapper.add_cloud(cloud);
        }

        if (scans.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ── 保存 ──
    RCLCPP_INFO(node->get_logger(),
        "Saving PCD map to %s  (%zu points)...",
        output_path.c_str(), mapper.size());

    if (mapper.save(output_path)) {
        RCLCPP_INFO(node->get_logger(), "Saved successfully.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Save failed (no points?).");
    }
}

// ── main ───────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<pcdmap::PcdMapReceiver>();

    node->declare_parameter("map_frame",    "map");
    node->declare_parameter("output_path",  "map.pcd");
    node->declare_parameter("voxel_size",   0.05);

    const auto map_frame   = node->get_parameter("map_frame").as_string();
    const auto output_path = node->get_parameter("output_path").as_string();
    const auto voxel_size  = node->get_parameter("voxel_size").as_double();

    auto tf_buf      = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buf);

    // 计算线程独立跑，rclcpp::spin 只处理回调搬运
    std::thread worker(mapping_loop, node, tf_buf, map_frame, output_path, voxel_size);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    while (!g_shutdown && rclcpp::ok()) {
        exec.spin_some(std::chrono::milliseconds(10));
    }

    worker.join();
    rclcpp::shutdown();
    return 0;
}
