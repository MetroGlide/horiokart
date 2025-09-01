#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <map>
#include <utility>

struct GridCellFeature
{
    float z_min;
    float z_max;
    float z_variance;
    Eigen::Vector3f mean_normal;
    Eigen::Vector3f mean_rgb;
};

class PointCloudProcessor
{
public:
    PointCloudProcessor();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float leaf_size);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int mean_k, double stddev_mul_thresh);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const Eigen::Affine3f &transform);
    std::map<std::pair<int, int>, GridCellFeature> computeGridFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float grid_resolution);
};
