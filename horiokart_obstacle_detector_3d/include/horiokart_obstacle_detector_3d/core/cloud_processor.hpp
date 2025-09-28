// Copyright 2025 Horiokart Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"
#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"

namespace obstacle_detector {

struct ColorInfo {
  bool has_rgb{false};
  float rgb{0.0f};
  bool has_intensity{false};
  float intensity{0.0f};
};

struct CloudProcessorConfig {
  // ROI / グリッド
  double roi_x_min{0.1}, roi_x_max{5.0}, roi_y_min{-1.5}, roi_y_max{1.5};
  double grid_cell_size{0.05};

  // 傾斜 / PCA
  std::string slope_method{"finite_difference"};
  double pca_radius_m{0.15};
  int pca_min_points{10};

  // クラスタリング / ダウンサンプル
  double voxel_leaf_size{0.03};
  bool dynamic_leaf{true};
  int target_points{50000};

  // 地面 / 障害物フィルタ
  double ground_max_distance{0.08};
  double min_obstacle_height{0.08};
  double min_obstacle_volume{0.002};

  // 色 / 強度利用
  bool use_rgb{false};
  bool use_intensity{false};
};

struct CloudProcessorResult {
  std::unique_ptr<GridHeightMap> grid;
  std::vector<PointXYZ> ground_points;
  std::vector<PointXYZ> non_ground_points;
  std::vector<Cluster> clusters;
  std::vector<PointXYZ> obstacle_points;
  // グリッドごとの PCA 傾斜 / 法線の結果
  std::unordered_map<int, std::pair<double, Eigen::Vector3d>> pca_results;
};

class CloudProcessor {
public:
  explicit CloudProcessor(const CloudProcessorConfig &cfg);

  // 抽出された点とメタデータを処理し、グリッド／クラスタ／障害物を生成します
  CloudProcessorResult
  process(const std::vector<PointXYZ> &pts,
          const std::unordered_map<std::string, ColorInfo> &point_meta,
          ClusterDetector &cluster_detector,
          GroundSeparator &ground_separator) const;

private:
  CloudProcessorConfig cfg_;
};

} // namespace obstacle_detector
