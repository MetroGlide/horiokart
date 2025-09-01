# horiokart_depth_camera_costmap

深度カメラ（Realsense D435等）点群を用いたNavigation2 costmap_2dプラグインです。

## 機能
- 点群前処理（VoxelGrid, StatisticalOutlierRemoval）
- TF変換（camera_link→base_link）
- グリッドマップ生成・特徴量抽出
- 走行可否判定・コスト割当
- 障害物クラスタリング・可視化
- パラメータ管理（yaml/launchから取得）

## 使い方
1. `colcon build`でビルド
2. launch/yamlでプラグインをcostmap_2dに追加
3. `/camera/depth/color/points`トピックをpublish
4. RVizでコストマップ・障害物Markerを可視化

## 主要パラメータ例
- max_slope_angle_deg: 最大傾斜角
- max_step_height_m: 最大段差高さ
- grid_resolution_m: グリッド解像度
- z_variance_threshold: 高さ分散閾値
- normal_angle_threshold_deg: 法線角度閾値
- cost_traversable, cost_semi_traversable, cost_obstacle, cost_lethal: コスト値
- voxel_leaf_size_m, sor_mean_k, sor_stddev_mul_thresh: 点群前処理
- cluster_distance_threshold_m, cluster_min_points: クラスタリング

## テスト
`colcon test`でgtestによる単体テストが実行されます。
