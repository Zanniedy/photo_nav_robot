#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <memory>
#include <string>

namespace pcdmap {

class PoseTransformer
{
public:
    explicit PoseTransformer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : tf_buffer_(tf_buffer) {}

    // 将一帧 scan 变换到 target_frame，返回点云（失败返回空指针）
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_scan(
        const sensor_msgs::msg::LaserScan & scan,
        const std::string & target_frame)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                target_frame, scan.header.frame_id, scan.header.stamp);
        } catch (const tf2::TransformException &) {
            return nullptr;
        }

        const auto & t = tf.transform.translation;
        const auto & r = tf.transform.rotation;

        Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T(0, 3) = t.x;
        T(1, 3) = t.y;
        T(2, 3) = t.z;

        double angle = scan.angle_min;
        for (const float range : scan.ranges) {
            if (!std::isfinite(range) ||
                range < scan.range_min ||
                range > scan.range_max)
            {
                angle += scan.angle_increment;
                continue;
            }

            const Eigen::Vector4d p_laser(
                range * std::cos(angle), range * std::sin(angle), 0.0, 1.0);
            const Eigen::Vector4d p_target = T * p_laser;

            cloud->emplace_back(
                static_cast<float>(p_target.x()),
                static_cast<float>(p_target.y()),
                static_cast<float>(p_target.z()));

            angle += scan.angle_increment;
        }

        return cloud;
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace pcdmap
