#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <limits>

namespace obstacle_detection
{

struct GridCell
{
  float z_min = std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::lowest();
  bool has_point = false;
};

struct ObstacleDetectionParams
{
  // Step 2: VoxelGrid
  double voxel_leaf_size = 0.05;

  // Step 2: CropBox
  double cropbox_x_min = 0.0;
  double cropbox_x_max = 3.0;
  double cropbox_y_min = -0.75;
  double cropbox_y_max = 0.75;
  double cropbox_z_min = -0.1;
  double cropbox_z_max = 1.0;

  // Step 3: ΔZ
  double grid_size = 0.05;
  double delta_z_threshold = 0.15;

  // Step 4: Radius Outlier Removal
  double ror_radius_search = 0.2;
  int ror_min_neighbors = 3;

  // Step 4: Euclidean Cluster Extraction
  double cluster_tolerance = 0.3;
  int min_cluster_size = 10;
  int max_cluster_size = 1000;
};

class ObstacleDetector
{
public:
  using PointT = pcl::PointXYZRGB;
  using PCLCloud = pcl::PointCloud<PointT>;

  ObstacleDetector();
  explicit ObstacleDetector(const ObstacleDetectionParams & params);

  void setParams(const ObstacleDetectionParams & params);
  ObstacleDetectionParams getParams() const;

  // パイプライン全体を実行するエントリポイント
  bool process(const PCLCloud & input, PCLCloud & output_obstacle,
               std::vector<pcl::PointIndices> & output_clusters);

private:
  // Step 2: 各ステージを独立メソッドとして分割
  void downsample(const PCLCloud & in, PCLCloud & out);
  void cropROI(const PCLCloud & in, PCLCloud & out);

  // Step 3: ΔZ評価（差し替え対象の中核ロジック）
  void extractObstaclesByDeltaZ(const PCLCloud & in, PCLCloud & out);

  // Step 4: ノイズ除去・クラスタリング
  void removeOutliers(const PCLCloud & in, PCLCloud & out);
  void extractClusters(const PCLCloud & in,
                       PCLCloud & out,
                       std::vector<pcl::PointIndices> & clusters);

  ObstacleDetectionParams params_;

  // メンバ変数としての PCLオブジェクト並びにグリッド配列の使い回し
  PCLCloud::Ptr cloud_voxeled_;
  PCLCloud::Ptr cloud_cropped_;
  PCLCloud::Ptr cloud_obstacle_;
  PCLCloud::Ptr cloud_clustered_;

  // ΔZグリッド配列: フレーム間で使い回し（毎フレームnewしない）
  std::vector<GridCell> grid_;
};

}  // namespace obstacle_detection
