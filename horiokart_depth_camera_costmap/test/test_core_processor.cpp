#include <gtest/gtest.h>
#include "horiokart_depth_camera_costmap/core_processor.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"

using namespace horiokart::depth_camera_costmap;

TEST(CoreProcessorTest, SimpleFlatSurface)
{
    std::vector<Point3D> pts;
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 10; ++y)
        {
            Point3D p;
            p.x = x * 0.05f;
            p.y = y * 0.05f;
            p.z = 0.0f;
            p.r = 100;
            p.g = 100;
            p.b = 100;
            pts.push_back(p);
        }
    CoreParams params;
    params.grid_resolution_m = 0.05;
    GridCostMap grid = processPointCloud(pts, params);
    auto occ = convertToOccupancyArray(grid, OccupancyOptions());
    ASSERT_FALSE(occ.empty());
    // all should be traversable (cost 0)
    for (auto v : occ)
    {
        ASSERT_TRUE(v == params.cost_traversable || v == 255);
    }
}

TEST(CoreProcessorTest, ClusterDetection)
{
    std::vector<Point3D> pts;
    // create a small obstacle at center
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        {
            Point3D p;
            p.x = 0.5f + dx * 0.05f;
            p.y = 0.5f + dy * 0.05f;
            p.z = 0.5f;
            p.r = 200;
            p.g = 0;
            p.b = 0;
            pts.push_back(p);
        }
    CoreParams params;
    params.grid_resolution_m = 0.05;
    params.max_step_height_m = 0.1;
    params.cost_obstacle = 150;
    GridCostMap grid = processPointCloud(pts, params);
    ClusterParams cparams;
    cparams.cluster_distance_threshold_m = 0.2;
    cparams.cluster_min_points = 3;
    auto clusters = clusterCostMap(grid, cparams);
    ASSERT_GE(clusters.size(), 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
