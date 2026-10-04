#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <memory>
#include <string>

namespace pcdmap {

class PointCloudMapper
{
public:
    explicit PointCloudMapper(double voxel_size = 0.05)
    : voxel_size_(voxel_size),
      cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {}

    // 追加一帧已变换到全局坐标系的点云
    void add_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & frame)
    {
        if (!frame || frame->empty()) return;
        *cloud_ += *frame;
        ++frames_added_;

        // 每积累 N 帧做一次体素滤波，避免内存无限增长
        if (frames_added_ % DOWNSAMPLE_INTERVAL == 0) {
            downsample();
        }
    }

    std::size_t size() const { return cloud_->size(); }

    // 保存 .pcd，保存前做最终体素滤波
    bool save(const std::string & path)
    {
        if (cloud_->empty()) return false;

        downsample();

        std::filesystem::path p(path);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }

        return pcl::io::savePCDFileBinary(path, *cloud_) == 0;
    }

private:
    void downsample()
    {
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_);
        vg.setLeafSize(
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_));
        vg.filter(*filtered);
        cloud_ = filtered;
    }

    static constexpr int DOWNSAMPLE_INTERVAL = 50;

    double   voxel_size_;
    int      frames_added_{0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

} // namespace pcdmap
