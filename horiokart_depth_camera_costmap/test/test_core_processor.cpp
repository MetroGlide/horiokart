#include <gtest/gtest.h>
#include <cmath>

#include "horiokart_depth_camera_costmap/core_processor.hpp"

using namespace horiokart::depth_camera_costmap;

// Helper: generate flat plane points in a rectangular region
static std::vector<Point3D> generateFlatPlane(float xmin, float xmax, float ymin, float ymax, float step, float z)
{
    std::vector<Point3D> pts;
    for (float x = xmin; x <= xmax; x += step)
    {
        for (float y = ymin; y <= ymax; y += step)
        {
            pts.push_back(Point3D{x, y, z});
        }
    }
    return pts;
}

TEST(CoreProcessorTest, FlatPlaneProducesTraversableCells)
{
    CoreParams params;
    params.grid_resolution = 0.1f;
    params.voxel_size = 0.02f;
    params.z_variance_threshold = 0.001f; // low threshold to detect flatness
    params.cost_lethal = 254;

    auto pts = generateFlatPlane(-0.5f, 0.5f, -0.5f, 0.5f, 0.05f, 0.0f);
    GridCostMap grid = processPointCloud(pts, params);

    EXPECT_GT(grid.width, 0u);
    EXPECT_GT(grid.height, 0u);
    // At least one cell must be traversable (cost == 0)
    bool has_traversable = false;
    for (const auto &c : grid.cells)
    {
        if (c.count > 0 && c.cost == 0)
        {
            has_traversable = true;
            EXPECT_NEAR(c.mean_z, 0.0f, 1e-2f);
        }
    }
    EXPECT_TRUE(has_traversable);
}

TEST(CoreProcessorTest, SingleHighOutlierMarkedAsLethal)
{
    CoreParams params;
    params.grid_resolution = 0.1f;
    params.voxel_size = 0.02f;
    params.z_variance_threshold = 0.0001f; // very small to force detection
    params.cost_lethal = 200;

    // flat plane
    auto pts = generateFlatPlane(-0.5f, 0.5f, -0.5f, 0.5f, 0.05f, 0.0f);
    // add an outlier at (0.0, 0.0) with high z
    pts.push_back(Point3D{0.0f, 0.0f, 1.0f});

    GridCostMap grid = processPointCloud(pts, params);

    // find the cell that contains (0,0)
    int ix = static_cast<int>(std::floor(0.0f / grid.resolution)) - static_cast<int>(std::floor(grid.origin.x / grid.resolution));
    int iy = static_cast<int>(std::floor(0.0f / grid.resolution)) - static_cast<int>(std::floor(grid.origin.y / grid.resolution));
    ASSERT_GE(ix, 0);
    ASSERT_GE(iy, 0);
    ASSERT_LT(static_cast<size_t>(ix), grid.width);
    ASSERT_LT(static_cast<size_t>(iy), grid.height);

    const GridCellFeature &f = grid.cells[grid.index(static_cast<std::uint32_t>(ix), static_cast<std::uint32_t>(iy))];
    EXPECT_GT(f.var_z, params.z_variance_threshold - 1e-6f);
    EXPECT_EQ(f.cost, params.cost_lethal);
}

TEST(CoreProcessorTest, ClusterExtractionSimple)
{
    GridCostMap grid;
    grid.resolution = 0.1f;
    grid.width = 10;
    grid.height = 10;
    grid.origin = Point3D{-0.5f, -0.5f, 0.0f};
    grid.cells.assign(static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height), GridCellFeature());

    // mark a 3x3 block in center as occupied (cost = 200)
    for (int y = 4; y <= 6; ++y)
    {
        for (int x = 4; x <= 6; ++x)
        {
            auto &c = grid.cells[grid.index(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y))];
            c.count = 1;
            c.mean_z = 0.1f;
            c.var_z = 0.0f;
            c.cost = 200;
        }
    }

    ClusterParams cp;
    cp.cluster_tolerance = 0.15f;
    cp.min_cluster_size = 3;

    auto clusters = clusterCostMap(grid, cp);
    ASSERT_EQ(clusters.size(), 1u);
    EXPECT_GE(clusters[0].points.size(), 9u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
