#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

// TODO: 点云匹配定位（ICP / NDT）
//   目标：订阅 /scan，将当前激光帧与加载的地图做匹配，发布 map->base_footprint 定位结果
//   候选方案：
//     - pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> 做帧-地图 ICP
//     - pclomp::NormalDistributionsTransform（速度更快，需要 ndt_omp 包）
//   接口草图：
//     订阅: /scan (sensor_msgs/LaserScan)  初始位姿: /initialpose (geometry_msgs/PoseWithCovarianceStamped)
//     发布: /map_odom_tf (通过 tf2_ros::TransformBroadcaster 发 map->odom)
//           /localization_score (std_msgs/Float32, 匹配得分供监控)

namespace pcdmap {

class PcdMapServerNode : public rclcpp::Node
{
public:
    explicit PcdMapServerNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
    : Node("pcd_map_server_node", opts)
    {
        declare_parameter("map_path",       "map.pcd");
        declare_parameter("map_frame",      "map");
        declare_parameter("publish_rate_hz", 1.0);
        declare_parameter("voxel_size",      0.05);

        map_path_    = get_parameter("map_path").as_string();
        map_frame_   = get_parameter("map_frame").as_string();
        publish_rate_ = get_parameter("publish_rate_hz").as_double();
        voxel_size_  = get_parameter("voxel_size").as_double();

        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pcd_map", rclcpp::QoS(1).transient_local());

        if (!load_map()) {
            RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", map_path_.c_str());
            return;
        }

        publish_map();  // 立即发一次（transient_local 保证后来者也能收到）

        const auto period_ms = static_cast<int>(1000.0 / publish_rate_);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this] { publish_map(); });
    }

private:
    bool load_map()
    {
        auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *raw) != 0) {
            return false;
        }
        cloud_ = raw;  // 保存时已做体素滤波，直接用
        RCLCPP_INFO(get_logger(), "Loaded map: %s  (%zu points)",
            map_path_.c_str(), cloud_->size());
        return true;
    }

    void publish_map()
    {
        if (!cloud_ || cloud_->empty()) return;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud_, msg);
        msg.header.stamp    = now();
        msg.header.frame_id = map_frame_;
        map_pub_->publish(msg);
    }

    std::string map_path_, map_frame_;
    double      publish_rate_, voxel_size_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr          cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr                 timer_;
};

} // namespace pcdmap

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcdmap::PcdMapServerNode>());
    rclcpp::shutdown();
    return 0;
}
