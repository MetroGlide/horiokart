#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <utility>
#include <Eigen/Core>

namespace horiokart::depth_camera_costmap
{

    struct Point3D
    {
        float x{0}, y{0}, z{0};
        uint8_t r{0}, g{0}, b{0};
    };

    struct GridCellFeature
    {
        float z_min{0.f};
        float z_max{0.f};
        float z_mean{0.f};
        float z_variance{0.f};
        float z_m2{0.f}; // running sum of squares for Welford
        Eigen::Vector3f mean_normal{0.f, 0.f, 1.f};
        Eigen::Vector3f mean_rgb{0.f, 0.f, 0.f};
        int count{0};
    };

    // hash for pair<int,int>
    struct PairHash
    {
        std::size_t operator()(const std::pair<int, int> &p) const noexcept
        {
            return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^ static_cast<unsigned long long>(p.second));
        }
    };

    struct GridCostMap
    {
        // sparse map keyed by (ix,iy)
        std::unordered_map<std::pair<int, int>, uint8_t, PairHash> costs;

        int min_ix{0}, max_ix{0}, min_iy{0}, max_iy{0};
        int width{0}, height{0};
        struct Origin
        {
            double x{0}, y{0}, z{0};
        } origin;
        double resolution_m{0.05};
        std::string frame_id{""};
    };

    struct ObstacleCluster
    {
        enum class Type
        {
            WALL,
            ROCK,
            UNKNOWN
        };
        std::vector<std::pair<int, int>> cells;
        Eigen::Vector2f centroid{0.f, 0.f};
        Type type{Type::UNKNOWN};
    };

    // Parameters used by core processing
    struct CoreParams
    {
        double grid_resolution_m{0.05};
        double voxel_leaf_size_m{0.02};
        int sor_mean_k{50};
        double sor_stddev_mul_thresh{1.0};
        int normal_k{10};

        double max_slope_angle_deg{30.0};
        double normal_angle_threshold_deg{10.0};
        double max_step_height_m{0.10};
        double z_variance_threshold{0.02};

        uint8_t cost_traversable{0};
        uint8_t cost_semi_traversable{50};
        uint8_t cost_obstacle{150};
        uint8_t cost_lethal{255}; // reserved unknown/lethal
    };

    struct ClusterParams
    {
        double cluster_distance_threshold_m{0.20};
        int cluster_min_points{3};
        int cost_threshold{150}; // cells with cost >= this are considered points
    };

    struct OccupancyOptions
    {
        // placeholder for future options
    };

    enum class MergeMode
    {
        OVERWRITE_MAX,
        OVERWRITE_NEW
    };

} // namespace
