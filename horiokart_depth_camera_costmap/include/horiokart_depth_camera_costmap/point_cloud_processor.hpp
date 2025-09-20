#pragma once

#include "horiokart_depth_camera_costmap/core_types.hpp"
#include <vector>
#include <map>
#include <utility>

namespace horiokart
{
    namespace depth_camera_costmap
    {

        // Minimal, ROS/core-independent point cloud utilities interface.
        class PointCloudProcessor
        {
        public:
            PointCloudProcessor();

            // downsample points using voxel grid (simple CPU implementation)
            std::vector<Point3D> downsample(const std::vector<Point3D> &points, float leaf_size);

            // remove statistical outliers (naive implementation)
            std::vector<Point3D> removeOutliers(const std::vector<Point3D> &points, int mean_k, double stddev_mul_thresh);

            // compute per-cell features
            std::map<std::pair<int, int>, GridCellFeature> computeGridFeatures(const std::vector<Point3D> &points, float grid_resolution);
        };

    } // namespace depth_camera_costmap
} // namespace horiokart
