// ファイル: point_cloud_processor.cpp
// 概要: 点群前処理ユーティリティを提供します。
//       - downsample: VoxelGrid によるダウンサンプリング
//       - removeOutliers: StatisticalOutlierRemoval による外れ値除去
//       - transform: Eigen 変換を用いた点群座標変換
//       - computeGridFeatures: 点群をグリッドに投影し，各セルごとに高さや色，法線などの特徴量を算出

#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <numeric>
#include <algorithm>
#include <vector>
#include <map>
#include <Eigen/Dense>

PointCloudProcessor::PointCloudProcessor() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudProcessor::downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float leaf_size)
{
    // VoxelGrid によるダウンサンプリング
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.filter(*filtered);
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudProcessor::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int mean_k, double stddev_mul_thresh)
{
    // StatisticalOutlierRemoval による外れ値除去
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    sor.filter(*filtered);
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudProcessor::transform(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const Eigen::Affine3f &transform)
{
    // Eigen 変換を用いた点群座標変換
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *transformed, transform);
    return transformed;
}

std::map<std::pair<int, int>, GridCellFeature> PointCloudProcessor::computeGridFeatures(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    float grid_resolution)
{
    // 点群をグリッドに投影し，各セルごとに高さや色，法線などの特徴量を算出
    std::map<std::pair<int, int>, std::vector<pcl::PointXYZRGB>> grid_map;
    for (const auto &pt : cloud->points)
    {
        // skip invalid points
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        int ix = static_cast<int>(std::floor(pt.x / grid_resolution));
        int iy = static_cast<int>(std::floor(pt.y / grid_resolution));
        grid_map[{ix, iy}].push_back(pt);
    }
    std::map<std::pair<int, int>, GridCellFeature> features;
    for (const auto &kv : grid_map)
    {
        const auto &pts = kv.second;
        if (pts.empty())
            continue;
        GridCellFeature feat;
        std::vector<float> zs;
        Eigen::Vector3f normal_sum(0, 0, 0);
        Eigen::Vector3f rgb_sum(0, 0, 0);
        for (const auto &pt : pts)
        {
            zs.push_back(pt.z);
            rgb_sum += Eigen::Vector3f(static_cast<float>(pt.r), static_cast<float>(pt.g), static_cast<float>(pt.b));
        }
        feat.z_min = *std::min_element(zs.begin(), zs.end());
        feat.z_max = *std::max_element(zs.begin(), zs.end());
        float mean_z = std::accumulate(zs.begin(), zs.end(), 0.0f) / zs.size();
        float var_z = 0.0f;
        for (float z : zs)
            var_z += (z - mean_z) * (z - mean_z);
        feat.z_variance = var_z / zs.size();
        feat.mean_rgb = rgb_sum / static_cast<float>(pts.size());
        // 法線推定（簡易版）
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cell_cloud->points.assign(pts.begin(), pts.end());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        ne.setInputCloud(cell_cloud);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        ne.setKSearch(std::min<int>(10, static_cast<int>(cell_cloud->size())));
        try
        {
            ne.compute(*normals);
        }
        catch (...)
        {
            // if normal estimation fails, set a default normal pointing up
            feat.mean_normal = Eigen::Vector3f(0, 0, 1);
            features[kv.first] = feat;
            continue;
        }
        for (const auto &n : normals->points)
            normal_sum += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
        if (normals->size() > 0)
            feat.mean_normal = normal_sum / static_cast<float>(normals->size());
        else
            feat.mean_normal = Eigen::Vector3f(0, 0, 1);
        features[kv.first] = feat;
    }
    return features;
}
