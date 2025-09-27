#pragma once

#include <vector>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
struct Cluster
{
  std::vector<PointXYZ> points;
  PointXYZ centroid;
  double volume;
  int id;
};

class ClusterDetector
{
public:
  ClusterDetector() = default;

  void setParameters(double cluster_tolerance, int min_cluster_size, int max_cluster_size);

  /// 任意: ボクセルグリッドによるダウンサンプリングを有効にする（メートル単位の leaf サイズ）。
  /// 0.0 を設定すると無効（デフォルト）。
  void setDownsampleLeafSize(double leaf_size);
  // 半径ベースの外れ値除去（任意）。radius=0 で無効。
  void setOutlierRadius(double radius);
  void setOutlierMinNeighbors(int min_neighbors);
  // Z 軸に対する PassThrough フィルタ（任意）。enable=false で無効。
  void setPassThroughZ(bool enable, double z_min = -1.0, double z_max = 1.0);
  // true の場合、ダウンサンプリング後の点群でクラスタリングを行い、元の点群にマッピングして
  // クラスタを拡張します
  void setExpandToOriginalCloud(bool enable);

  // 最小限の API: ベクトル化した点集合からクラスタを抽出する（ヘッドレス実装）
  std::vector<Cluster> extractClusters(const std::vector<PointXYZ> & points) const;

private:
  double cluster_tolerance_ = 0.1;
  int min_cluster_size_ = 30;
  int max_cluster_size_ = 1000000;
  double voxel_leaf_size_ = 0.0;
  // 外れ値除去
  double outlier_radius_ = 0.0;
  int outlier_min_neighbors_ = 1;
  // PassThrough（Z）フィルタ
  bool passthrough_z_enable_ = false;
  double passthrough_z_min_ = -1.0;
  double passthrough_z_max_ = 1.0;
  // 元の点群へマッピングして復元
  bool expand_to_original_cloud_ = false;
};

}  // namespace obstacle_detector
