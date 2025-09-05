# 深度カメラによる地形・障害物検出システム設計書

## 1. 概要
この文書は、深度カメラ（例: Realsense D435）から得られる色付き点群を用いて、ロボット周辺の地形を2Dグリッドに変換し、各セルの走行可能性を評価してコストマップを生成し、障害物クラスタを抽出するためのアルゴリズム仕様と実装詳細を定義する。
本システムはコア処理（ROS非依存）とROSアダプタ（メッセージ変換、TF、publish/subscribe、Nav2連携）に分離して実装する。

## 2. 前提条件・用語
- ロボット基準座標系: `base_link`（すべての評価はこの座標系を基準とする）
- 点群メッセージ: `sensor_msgs/msg/PointCloud2`（色付き点群を前提）
- 出力グリッド解像度: `grid_resolution_m`（例: 0.05〜0.10 m/セル）
- コストスケール: 内部は 0..254 を有効コスト、255 を unknown（予約値）とする uint8 値で扱う

## 3. 入出力インタフェース
### 3.1 core ライブラリ（関数/型）
- 型
  - Point3D { float x, y, z; uint8_t r,g,b; }
  - GridCellFeature { float z_min, z_max, z_variance; Eigen::Vector3f mean_normal; Eigen::Vector3f mean_rgb; }
  - GridCostMap {
      std::unordered_map<std::pair<int,int>, uint8_t> costs; // key: (ix,iy), values: 0..254 valid, 255 = unknown
      int min_ix, max_ix, min_iy, max_iy;
      int width; int height; // computed as max_ix-min_ix+1, max_iy-min_iy+1
      struct Origin { double x, y, z; } origin; // world coordinates of cell (min_ix, min_iy)
      double resolution_m;
      std::string frame_id; // metadata
    }
  - ObstacleCluster { std::vector<std::pair<int,int>> cells; Eigen::Vector2f centroid; enum Type { WALL, ROCK, UNKNOWN } type; }
- 関数
  - GridCostMap processPointCloud(const std::vector<Point3D>& points, const CoreParams& params);
  - std::vector<ObstacleCluster> clusterCostMap(const GridCostMap& grid, const ClusterParams& params);
  - GridCostMap mergeCostMaps(const GridCostMap& a, const GridCostMap& b, MergeMode m);
  - std::vector<uint8_t> convertToOccupancyArray(const GridCostMap& grid, const OccupancyOptions& opt); // returns row-major byte array: 0..254 valid cost values, 255 reserved for unknown

### 3.2 ROS アダプタ（ノード/トピック）
- Subscriber: `/camera/depth/color/points` (sensor_msgs/PointCloud2)
- Publisher: `/depth_costmap/occupancy_grid` (nav_msgs/OccupancyGrid またはカスタムGridCostMapMsg)
- Publisher: `/depth_costmap/clusters` (visualization_msgs/MarkerArray)
- Optional: `/depth_costmap/sparse_costmap` (カスタム msg で sparse representation を提供)
- Nav2 連携: `costmap_adapter_node` が `/depth_costmap/occupancy_grid` を受け取り master costmap に反映する

## 4. 点群処理パイプライン
処理は以下ステップで行う。core は座標系依存を避けるため、入力点群は既に `target_frame`（通常 `base_link`）に変換済みの `std::vector<Point3D>` を受け取る。

1. Downsample（VoxelGrid）
   - 入力: Point3D 配列
   - パラメータ: `voxel_leaf_size_m`
   - 処理: 各 voxel の中心に最も近い点を代表点にする。高速化のため hash map を用いる。

2. RemoveOutliers（StatisticalOutlierRemoval）
   - 入力: downsampled 点群
   - パラメータ: `sor_mean_k`, `sor_stddev_mul_thresh`
   - 処理: 各点の近傍距離の平均と分散を計算し、閾値外の点を除去する。

3. 投影とグリッド化
   - 入力: 前処理済点群（base_link 座標）
   - グリッドインデックス計算: ix = floor(x / resolution), iy = floor(y / resolution)
   - 各セルに属する点群を集約（sparse map: unordered_map<pair<int,int>, vector<Point3D>>）

4. グリッドセル特徴量算出
   - 各セルについて:
     - z_min = min(z)
     - z_max = max(z)
     - mean_z = average(z)
     - z_variance = (1/N) * sum((z - mean_z)^2)
     - mean_rgb = avg(r,g,b)
     - 法線推定: pcl::NormalEstimation を用いるか、最小二乗法で局所平面をフィットして法線を求める。
       - パラメータ: `normal_k`（法線推定に用いる点数）
     - 法線ベクトルは正規化して保存

5. Traversability 評価（セル毎）
   - 入力: GridCellFeature と閾値パラメータ
   - パラメータ: `max_slope_angle_deg`, `max_step_height_m`, `z_variance_threshold`, `normal_angle_threshold_deg`, `cost_traversable`, `cost_semi_traversable`, `cost_obstacle`, `cost_lethal`
   - 処理ルール（順序適用）:
     1. 安全路面判定:
        - 条件: z_variance < z_variance_threshold AND angle(mean_normal, vertical) < max_slope_angle_deg
        - 出力: cost = cost_traversable
     2. 段差判定:
        - 条件: (z_max - z_min) > max_step_height_m
        - 出力: cost = cost_obstacle
     3. 半通行判定:
        - 条件: angle(mean_normal, vertical) < (max_slope_angle_deg + normal_angle_threshold_deg)
        - 出力: cost = cost_semi_traversable
     4. 致命判定:
        - それ以外 -> cost = cost_lethal
   - 補足: angle(mean_normal, vertical) は acos(clamp(n.z, -1,1)) * 180/pi で算出

6. GridCostMap 生成
   - 各セルインデックス (ix,iy) に対して上記の cost を割当てる
   - bounds を min/max ix,iy で確定する
   - 内部表現は sparse map（unordered_map<pair<int,int>, uint8_t>）で保持する

## 5. 障害物クラスタリング
クラスタリングはセルレベルでの DBSCAN 風アルゴリズムを採用する。距離はセル単位（メートル→セルへ変換）で計算する。

1. パラメータ: `cluster_distance_threshold_m`, `cluster_min_points`
2. eps_cells = cluster_distance_threshold_m / grid_resolution_m
3. データ: cost >= cost_threshold (通常 cost_obstacle 以上) のセル集合を points にする
4. DBSCAN 風アルゴリズム:
   - 初期ラベルを -1（未訪問）で設定
   - 各未訪問点 p について:
     - 近傍 N = { q | dist(p,q) <= eps_cells }
     - if |N| < cluster_min_points then label = noise (-2)
     - else create new cluster id, expand: for each q in N visit neighbors and add
   - 収集後、cluster_min_points より小さいクラスタは破棄
5. クラスタ情報生成:
   - セルリスト、centroid（セル座標の平均）を計算
   - クラスタ種別推定: size > wall_threshold -> WALL, size > rock_threshold -> ROCK, else UNKNOWN

## 6. OccupancyGrid への変換
GridCostMap を nav_msgs/OccupancyGrid もしくはカスタム msg へ変換する際の手順:

1. width = max_ix - min_ix + 1, height = max_iy - min_iy + 1
2. origin の決定: origin.x = min_ix * resolution_m, origin.y = min_iy * resolution_m（frame は GridCostMap.frame_id）
3. data 配列を row-major で作成する。初期値は -1（unknown）または 0（free）に設定する運用を選択する。
4. 内部コスト 0..254 を OccupancyGrid の 0..100 スケールに線形変換するか、保持する場合はカスタム msg を用いる。変換式例（確定）: occ = round((cost / 254.0) * 100.0)
   - 注意: コア側が返す reservation 値 255 は unknown を意味し、OccupancyGrid では -1 にマップされることを明示する。
5. 出力メタデータ: resolution, width, height, origin, header.stamp, header.frame_id

## 7. TF とアダプタの振る舞い
- ROS アダプタは点群受信後、TF lookup→点群変換→core呼び出し→OccupancyArray生成→OccupancyGrid作成→publishまでの処理を担当する。

## 8. Nav2 への反映（costmap アダプタ）
- `costmap_adapter_node` は `/depth_costmap/occupancy_grid` を購読し、差分セルのみを master costmap に反映する。conditional_overwrite パラメータで上書き条件を制御する。

## 9. パラメータ一覧（型とデフォルト値）
- grid_resolution_m: double = 0.05
- voxel_leaf_size_m: double = 0.02
- sor_mean_k: int = 50
- sor_stddev_mul_thresh: double = 1.0
- normal_k: int = 10
- max_slope_angle_deg: double = 30.0
- normal_angle_threshold_deg: double = 10.0
- max_step_height_m: double = 0.10
- z_variance_threshold: double = 0.02
- cost_traversable: int = 0
- cost_semi_traversable: int = 50
- cost_obstacle: int = 150
- cost_lethal: int = 255
- cluster_distance_threshold_m: double = 0.20
- cluster_min_points: int = 3
- tf_lookup_timeout_ms: int = 100
- tf_retry_count: int = 3
- tf_retry_backoff_ms: int = 50
- conditional_overwrite: bool = true
- output_occupancy_topic: string = "/depth_costmap/occupancy_grid"
- marker_topic: string = "/depth_costmap/clusters"

## 10. スレッド・同期設計
- アダプタ（ROSノード）と Nav2 Layer 双方が内部 state（GridCostMap, clusters）を参照する可能性があるため、共有データには `std::mutex state_mutex_` を用いる。
- コールバック内で mutex を短時間占有し、重い処理（法線推定やクラスタ化）は mutex の外で行う。更新時は新しい result を作成してから mutex で入れ替える。
- publish は mutex のコピー済みデータを用いて行うことでロック時間を最小化する。

## 11. エラー処理とロギング
- TF取得失敗: WARN ログを出力し、その点群はスキップする（リトライは上限回数まで行う）
- PCL の処理例外: ERROR ログを出力しそのメッセージをスキップ
- パラメータ不正: 事前に検証し、閾値が不適切な場合はデフォルトへフォールバックして WARN を出す
- コストマップ反映失敗: WARN を出力し、次回更新へ委ねる

## 12. テスト設計
### 12.1 単体テスト（core）
- processPointCloud に対して複数の合成点群ケースを用意し、期待される cost が割り当てられることを検証する（gtest）
- clusterCostMap に対して既知のセル集合を与え、期待するクラスタ数・centroid を検証する

### 12.2 統合テスト（adapter）
- Node を起動し、記録済み PointCloud2 を publish、受信した OccupancyGrid の値が期待と合致することを検証する
- TF を模擬して transform の異常パスも検証する

### 12.3 性能テスト
- 異なる点群密度 (e.g. 10k, 50k, 100k points) で処理時間を計測し、目標フレームレート（例: 5Hz）を満たすか確認する
- メモリ使用量、GC 負荷（C++ならメモリ確保頻度）を測定

## 13. 最適化と拡張案（実装手順に従って導入）
- 法線推定やセル特徴量計算の並列化（TBB/OpenMP）
- 大域マップとの差分更新（前回コストとの差分のみ master に送る）
- GPU アクセラレーション（将来的選択肢）
- 自動パラメータチューニング用スクリプト

## 14. CI / 品質基準
- GitHub Actions で以下を実行:
  - colcon build / cmake build
  - colcon test / unit tests
  - static analysis (clang-tidy)
  - coverage レポート
- テストしきい値を定義し、性能が下回った場合は regression を fail にする

## 15. 実装フェーズと見積
- フェーズ1 (core 実装 + unit tests): 1〜2 日
- フェーズ2 (depth_camera_processor_node): 0.5〜1 日
- フェーズ3 (costmap_adapter_node / Nav2 統合): 0.5〜1 日
- フェーズ4 (最適化, CI, docs): 2〜4 日

---

本設計書に従い、まず core の型定義と `processPointCloud` の最小実装を追加し unit test を作成する。これにより以降の adapter 実装と Nav2 連携を確実に行う準備が整う。

### コアとアダプタのインタフェース（統一ルール）

本ドキュメントではコア処理と ROS アダプタ間の受け渡し規則を明確にします。これにより実装の一貫性と将来的な再利用性を確保します。

- コア出力（convertToOccupancyArray）:
  - 返却型: `std::vector<uint8_t>`（row-major、サイズ width*height）
  - 各要素の意味:
    - 0..254: 正常なコスト値（0 = 通行可能、254 = 最大コスト）
    - 255: 未知（unknown）
  - メタ情報: `GridCostMap` または返却構造体に `origin`（x,y,z）、`resolution`、`width`、`height` を含めること。

- アダプタの役割:
  - コアの生配列を受け取り `nav_msgs::msg::OccupancyGrid` を生成する。
  - マッピング（確定式）:
    - core 0..254 -> occupancy 0..100 (線形スケーリング: occupancy = round((core_value / 254.0) * 100.0))
    - core 255 -> occupancy -1 (unknown)

注: これらのルールはコアとアダプタの責務分離を明確にし、コア側が ROS や nav_msgs に依存しない設計を保証します。

