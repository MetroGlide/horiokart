#include "horiokart_depth_camera_costmap/obstacle_clusterer.hpp"
#include <cmath>
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
        std::string type; // "wall", "rock", "slope" など
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

    ObstacleClusterer::ObstacleClusterer(float cluster_distance_threshold, int cluster_min_points, float grid_resolution_m)
        : cluster_distance_threshold_(cluster_distance_threshold), cluster_min_points_(cluster_min_points), grid_resolution_m_(grid_resolution_m) {}

    std::vector<ObstacleCluster> ObstacleClusterer::cluster(const std::map<std::pair<int, int>, int> &cost_map, int cost_threshold)
    {
        // Collect high-cost cells
        std::vector<std::pair<int, int>> points;
        for (const auto &kv : cost_map)
        {
            if (kv.second >= cost_threshold)
            {
                points.push_back(kv.first);
            }
        }
        std::vector<ObstacleCluster> clusters;
        if (points.empty())
            return clusters;
        // Convert eps from meters to cell units
        const float eps = cluster_distance_threshold_ / grid_resolution_m_;
        const int n = static_cast<int>(points.size());
        std::vector<int> labels(n, -1);
        int cid = 0;
        for (int i = 0; i < n; ++i)
        {
            if (labels[i] != -1)
                continue;
            // find neighbors
            std::vector<int> neighbors;
            for (int j = 0; j < n; ++j)
            {
                float dx = static_cast<float>(points[i].first - points[j].first);
                float dy = static_cast<float>(points[i].second - points[j].second);
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= eps)
                    neighbors.push_back(j);
            }
            if (static_cast<int>(neighbors.size()) < cluster_min_points_)
            {
                labels[i] = -2; // noise
                continue;
            }
            // expand cluster
            std::vector<int> stack = neighbors;
            for (int idx = 0; idx < static_cast<int>(stack.size()); ++idx)
            {
                int sidx = stack[idx];
                if (labels[sidx] == -2)
                    labels[sidx] = cid;
                if (labels[sidx] != -1)
                    continue;
                labels[sidx] = cid;
                // find neighbors of sidx
                for (int j = 0; j < n; ++j)
                {
                    float dx = static_cast<float>(points[sidx].first - points[j].first);
                    float dy = static_cast<float>(points[sidx].second - points[j].second);
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist <= eps)
                    {
                        if (std::find(stack.begin(), stack.end(), j) == stack.end())
                            stack.push_back(j);
                    }
                }
            }
            cid++;
        }
        // collect clusters
        std::map<int, std::vector<std::pair<int, int>>> cluster_cells;
        for (int i = 0; i < n; ++i)
        {
            if (labels[i] >= 0)
                cluster_cells[labels[i]].push_back(points[i]);
        }
        for (const auto &kv : cluster_cells)
        {
            const auto &cells = kv.second;
            if (static_cast<int>(cells.size()) < cluster_min_points_)
                continue;
            ObstacleCluster oc;
            oc.cells = cells;
            // compute centroid in cell coordinates
            float sx = 0, sy = 0;
            for (const auto &c : cells)
            {
                sx += static_cast<float>(c.first);
                sy += static_cast<float>(c.second);
            }
            oc.centroid = Eigen::Vector2f(sx / cells.size(), sy / cells.size());
            // rudimentary type classification based on cluster shape
            if (cells.size() > 50)
                oc.type = "wall";
            else if (cells.size() > 10)
                oc.type = "rock";
            else
                oc.type = "unknown";
            clusters.push_back(oc);
        }
        return clusters;
    }

} // namespace horiokart_depth_camera_costmap
