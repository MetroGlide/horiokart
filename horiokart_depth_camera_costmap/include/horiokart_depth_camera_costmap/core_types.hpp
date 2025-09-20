#pragma once

// core は ROS 非依存で動作することを意図した型定義ヘッダ
// このヘッダを唯一の型定義源 (single source of truth) として使ってください。

#include <cstdint>
#include <string>
#include <vector>
#include <array>

namespace horiokart
{
    namespace depth_camera_costmap
    {

        // ---------------------------------
        // 基本型
        // ---------------------------------

        // 3次元点（最小限）
        struct Point3D
        {
            float x{0.f};
            float y{0.f};
            float z{0.f};
        };

        // グリッドセルの特徴量（軽量 POD）
        struct GridCellFeature
        {
            std::uint32_t count{0};                       // セル内ポイント数
            float mean_z{0.f};                            // 平均高さ
            float var_z{0.f};                             // z の分散
            float z_min{0.f};                             // 最小 z
            float z_max{0.f};                             // 最大 z
            std::array<float, 3> normal{{0.f, 0.f, 0.f}}; // 法線ベクトル
            float traversability{1.f};                    // 0..1 の簡易スコア
            std::uint8_t cost{255};                       // 0..254: cost, 255: unknown/reserved
        };

        // 行優先 (row-major) のグリッド表現（dense）
        struct GridCostMap
        {
            std::uint32_t width{0};        // 列数 (x方向セル数)
            std::uint32_t height{0};       // 行数 (y方向セル数)
            float resolution{0.05f};       // 1セルのサイズ (m)
            Point3D origin{0.f, 0.f, 0.f}; // grid 原点のワールド座標（セル中心）
            std::vector<GridCellFeature> cells;

            inline std::size_t index(std::uint32_t ix, std::uint32_t iy) const
            {
                return static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) + static_cast<std::size_t>(ix);
            }
        };

        // ---------------------------------
        // 障害クラスタ型
        // ---------------------------------

        enum class ObstacleType : std::uint8_t
        {
            UNKNOWN = 0,
            POINT,
            LINE,
            POLYGON,
            WALL,
        };

        struct ObstacleCluster
        {
            std::uint64_t id{0};
            ObstacleType type{ObstacleType::UNKNOWN};
            Point3D centroid{0.f, 0.f, 0.f};
            std::vector<Point3D> points;
            float confidence{1.f};
            // store associated grid cell coordinates for visualization/size
            std::vector<std::pair<int, int>> cells;
        };

        // ---------------------------------
        // パラメータ
        // ---------------------------------

        struct CoreParams
        {
            float voxel_size{0.02f};
            int sor_mean_k{50};
            float sor_stddev_mul_thresh{1.0f};

            float grid_resolution{0.05f};
            std::uint32_t grid_width{400};
            std::uint32_t grid_height{400};

            float z_variance_threshold{0.01f};

            int normal_k{10};
            float normal_angle_threshold_deg{30.f};

            std::uint8_t cost_lethal{254}; // 0..254 are usable costs, 255 is reserved unknown
        };

        struct ClusterParams
        {
            float cluster_tolerance{0.05f};
            std::size_t min_cluster_size{10};
            std::size_t max_cluster_size{100000};
            float merge_distance{0.1f};
        };

    } // namespace depth_camera_costmap
} // namespace horiokart
