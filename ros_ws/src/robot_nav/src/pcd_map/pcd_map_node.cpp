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

    RCLCPP_INFO(node->get_logger(), "Mapping thread started (map_frame=%s). Ctrl+C to save.",
        map_frame.c_str());

    // 等待 TF 链 map→lidar_link 第一次可用（slam_toolbox 发布 map→odom 之前先阻塞）
    RCLCPP_INFO(node->get_logger(), "Waiting for TF: %s -> lidar_link ...", map_frame.c_str());
    while (!g_shutdown) {
        if (tf_buf->canTransform(map_frame, "lidar_link", tf2::TimePointZero)) {
            RCLCPP_INFO(node->get_logger(), "TF available, mapping starts.");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (g_shutdown) return;

    int tf_fail_count = 0;

    while (!g_shutdown) {
        // 阻塞等待，有扫描帧才唤醒
        auto scans = node->drain_scans(g_shutdown);

        for (const auto & s : scans) {
            auto cloud = transformer.transform_scan(s.msg, map_frame);
            if (!cloud) {
                ++tf_fail_count;
                if (tf_fail_count % 20 == 1) {
                    RCLCPP_WARN(node->get_logger(),
                        "TF lookup failed %d times (map_frame=%s, scan_frame=%s). "
                        "Is slam_toolbox running and publishing map->odom?",
                        tf_fail_count, map_frame.c_str(),
                        s.msg.header.frame_id.c_str());
                }
                continue;
            }
            tf_fail_count = 0;
            mapper.add_cloud(cloud);
        }
    }

    // ── 保存 ──
    RCLCPP_INFO(node->get_logger(),
        "Saving PCD map to %s  (%zu points)...",
        output_path.c_str(), mapper.size());

    if (mapper.save(output_path)) {
        RCLCPP_INFO(node->get_logger(), "Saved successfully.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Save failed — no valid points accumulated.");
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
    g_shutdown = true;
    node->notify_shutdown();  // 唤醒 drain_scans 的 wait

    worker.join();
    rclcpp::shutdown();
    return 0;
}
