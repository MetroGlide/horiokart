#pragma once
#include <vector>
#include <map>
#include <utility>
#include <Eigen/Dense>

struct ObstacleCluster
{
    std::vector<std::pair<int, int>> cells;
    Eigen::Vector2f centroid;
    std::string type;
};

class ObstacleClusterer
{
public:
    ObstacleClusterer(float cluster_distance_threshold, int cluster_min_points);
    std::vector<ObstacleCluster> cluster(const std::map<std::pair<int, int>, int> &cost_map, int cost_threshold);

private:
    float cluster_distance_threshold_;
    int cluster_min_points_;
};
