#include <gtest/gtest.h>
#include "horiokart_depth_camera_costmap/depth_camera_costmap_layer.hpp"
#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"
#include "horiokart_depth_camera_costmap/traversability_evaluator.hpp"
#include "horiokart_depth_camera_costmap/obstacle_clusterer.hpp"
#include "horiokart_depth_camera_costmap/parameter_manager.hpp"

using namespace horiokart_depth_camera_costmap;

TEST(PointCloudProcessorTest, DownsampleRemoveOutliers)
{
    PointCloudProcessor pc_proc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // ダミー点群生成
    for (int i = 0; i < 100; ++i)
    {
        pcl::PointXYZRGB pt;
        pt.x = i * 0.01f;
        pt.y = i * 0.01f;
        pt.z = 0.0f;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud->points.push_back(pt);
    }
    auto ds = pc_proc.downsample(cloud, 0.05f);
    auto filtered = pc_proc.removeOutliers(ds, 10, 1.0);
    EXPECT_GT(filtered->size(), 0);
}

TEST(TraversabilityEvaluatorTest, Evaluate)
{
    TraversabilityEvaluator eval(20.0, 0.1, 0.005, 10.0, 10, 80, 200, 255);
    std::map<std::pair<int, int>, GridCellFeature> features;
    GridCellFeature feat;
    feat.z_min = 0.0f;
    feat.z_max = 0.0f;
    feat.z_variance = 0.001f;
    feat.mean_normal = Eigen::Vector3f(0, 0, 1);
    feat.mean_rgb = Eigen::Vector3f(255, 0, 0);
    features[{0, 0}] = feat;
    auto cost_map = eval.evaluate(features);
    EXPECT_EQ(cost_map[{0, 0}], 10);
}

// 他クラスのテストも追加可能

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
