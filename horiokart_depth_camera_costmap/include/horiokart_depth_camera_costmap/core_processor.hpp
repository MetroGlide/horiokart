#pragma once

// core の処理インタフェース宣言（ROS 非依存）
// 実装は src/core_processor.cpp に置くことを想定

#include "horiokart_depth_camera_costmap/core_types.hpp"
#include <vector>
#include <cstdint>

namespace horiokart
{
    namespace depth_camera_costmap
    {

        // 点群（Point3D の配列）から GridCostMap を生成する
        GridCostMap processPointCloud(const std::vector<Point3D> &points, const CoreParams &params);

        // GridCostMap から障害クラスタを抽出する
        std::vector<ObstacleCluster> clusterCostMap(const GridCostMap &grid, const ClusterParams &params);

        // 複数の GridCostMap をマージして 1 つの GridCostMap を返す
        // マージ戦略は実装に依存（例: cell.cost の最大値を採用）
        GridCostMap mergeCostMaps(const std::vector<GridCostMap> &maps, const CoreParams &params);

        // GridCostMap を占有配列（row-major の uint8_t 配列）に変換する
        // 戻り値の長さは grid.width * grid.height。セルコストは 0..254、255 は unknown
        std::vector<std::uint8_t> convertToOccupancyArray(const GridCostMap &grid);

    } // namespace depth_camera_costmap
} // namespace horiokart
