#pragma once

#include "core_types.hpp"
#include <vector>

namespace horiokart::depth_camera_costmap
{

    GridCostMap processPointCloud(const std::vector<Point3D> &points, const CoreParams &params);

    std::vector<ObstacleCluster> clusterCostMap(const GridCostMap &grid, const ClusterParams &params);

    GridCostMap mergeCostMaps(const GridCostMap &a, const GridCostMap &b, MergeMode m);

    std::vector<uint8_t> convertToOccupancyArray(const GridCostMap &grid, const OccupancyOptions &opt);

} // namespace
