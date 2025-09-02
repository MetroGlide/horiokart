#pragma once
#include <vector>
#include <map>
#include <utility>
#include <Eigen/Dense>

namespace horiokart_depth_camera_costmap
{

    struct ObstacleCluster
    {
        std::vector<std::pair<int, int>> cells;
        Eigen::Vector2f centroid;
        std::string type;
    };

    class ObstacleClusterer
    {
    public:
        ObstacleClusterer(float cluster_distance_threshold, int cluster_min_points, float grid_resolution_m);
        std::vector<ObstacleCluster> cluster(const std::map<std::pair<int, int>, int> &cost_map, int cost_threshold);

    private:
        float cluster_distance_threshold_;
        int cluster_min_points_;
        float grid_resolution_m_;
    };

} // namespace horiokart_depth_camera_costmap
