# Redesign proposal for horiokart_depth_camera_costmap

以下は本パッケージを長期的に最良状態へ再設計するための詳細提案です。
目標は再利用性・テスト性・性能・保守性を最大化し、ROS依存を最小化したコアライブラリを提供することです。

## 1. 要点
- コア処理（点群前処理・特徴抽出・通過可能性評価・クラスタ化）はROS非依存の純粋C++ライブラリとして実装する。
- ROS側は薄いアダプタ（message↔core型変換、TF、パラメータマッピング、publish/subscribe）に限定する。
- 出力は疎なグリッド表現（GridCostMap）を基本とし、`convertToOccupancyArray` によりバイト配列を生成する。ROS型の `nav_msgs::msg::OccupancyGrid` はアダプタが生成する。
- コアは副作用を持たない純関数群を基本とし単体テストを重視する。

## 2. レイヤ構成
1. core/（非依存ライブラリ）
   - 型: Point3D, GridCellFeature, GridCostMap, ObstacleCluster, CoreParams
   - 関数: processPointCloud(), clusterCostMap(), mergeCostMaps(), utilities
2. adapters/（ROS2ラッパー群）
   - depth_camera_processor_node: PointCloud2受信→TF変換→core呼出→publish (OccupancyGrid を生成するのは adapter)
   - costmap_adapter_node: core出力（カスタム msg/OccupancyArray）→Nav2へ差分で反映
3. tools/（ユーティリティ）
   - ベンチ、変換ユーティリティ、テストヘルパー

## 3. コアデータモデル
- Point3D { float x,y,z; uint8_t r,g,b; }
- GridCellFeature { float z_min, z_max, z_variance; Eigen::Vector3f mean_normal; Eigen::Vector3f mean_rgb; }
- GridCostMap {
  std::unordered_map<std::pair<int,int>, uint8_t> costs; // 0..254 valid cost values, 255 reserved for unknown
  int min_ix, max_ix, min_iy, max_iy;
  int width; int height; // computed as max_ix-min_ix+1, max_iy-min_iy+1
  struct Origin { double x, y, z; } origin; // world coordinates of cell (min_ix, min_iy)
  double resolution_m;
  std::string frame_id;
}
- ObstacleCluster { vector<pair<int,int>> cells; Eigen::Vector2f centroid; enum Type; }

## 4. 主要コアAPI（統一）
- GridCostMap processPointCloud(const std::vector<Point3D>& points, const CoreParams& params);
- std::vector<ObstacleCluster> clusterCostMap(const GridCostMap& grid, const ClusterParams& params);
- GridCostMap mergeCostMaps(const GridCostMap& a, const GridCostMap& b, MergeMode mode);
- std::vector<uint8_t> convertToOccupancyArray(const GridCostMap& grid, const OccupancyOptions& opt); // returns row-major byte array: 0..254 valid cost values, 255 reserved for unknown

注: `convertToOccupancyArray` は ROS 型に依存しない生の配列を返す。アダプタがこの配列を用いて `nav_msgs::msg::OccupancyGrid` を作成する。

## コア／アダプタ間のデータ契約（統一ルール）

コアライブラリと ROS アダプタ間のデータ受け渡しについて、実装の一貫性を保つために明確な契約を定義します。

- コアの出力
  - 関数名（設計上の代表例）: `convertToOccupancyArray(const GridCostMap &)`
  - 返却値: 連続した生のバイト配列（row-major、幅×高さ の長さを持つ `std::vector<uint8_t>` など）
  - 値の意味:
    - 0..254: 有効なコスト値（0 = 完全に通行可能、254 = 致命的/最大コスト）
    - 255: 未知（unknown）を表す予約値
  - 注意: 配列は座標系/解像度（origin, resolution, width, height）のメタ情報と合わせて返すか、別の構造体で返却してください（例: GridCostMap にメタ情報を含める）。

- アダプタの責務（ROS側）
  - アダプタはコアが返す生のバイト配列を受け取り、`nav_msgs::msg::OccupancyGrid` を生成して publish します。
  - 明確なマッピング規則:
    - コア値 0..254 は OccupancyGrid の 0..100 へ線形スケーリングして格納します。スケーリング例:
      - occupancy = round((core_value / 254.0) * 100.0)
    - コア値 255 は OccupancyGrid の -1（unknown）として扱います。
  - 例外/実装メモ: 必要ならばアダプタ側で閾値処理（例: ある閾値以上を必ず 100 にする）や`latching`等を行えますが、コアは純粋にコスト配列を返すことに専念します。

この契約により、コアは ROS 非依存の純粋な処理ロジックに集中でき、異なるアダプタ（ROS2 ノード、シミュレータ用、別言語バインディングなど）が一貫した方式で OccupancyGrid を構築できるようになります。

## 5. 実装上の重要事項
- TFはadapter側で処理。coreは座標系に依存しないことを前提とする。
- coreは副作用を持たない純関数にしてテストを容易にする。
- データ表現はデフォルトでsparse（unordered_map）とし、出力時に連続配列へ展開する。
- 出力スケールは内部0..255を採用。ROSへ公開時に必要に応じて0..100へスケーリングするのはアダプタの責務とする。
- 並列化: 特徴量計算や法線推定は並列化可能（TBB/OpenMPオプション）。

## 6. ファイル構成案
- include/horiokart_depth_camera_costmap/core_types.hpp
- include/horiokart_depth_camera_costmap/core_processor.hpp
- src/core_processor.cpp
- src/depth_camera_processor_node.cpp
- src/costmap_adapter_node.cpp
- include/.../occupancy_conversion.hpp
- tests/core_test.cpp, tests/integration_node_test.cpp
- launch/processor.launch.py, launch/costmap_adapter.launch.py

## 7. テスト・CI
- Unit tests: coreのロジック（gtest）
- Integration tests: node起動後のE2Eテスト（固定点群→期待コスト）
- ベンチ: 異なる点群密度で処理時間・メモリ測定
- CI: build/test/clang-tidy/coverageをGitHub Actionsで実行

## 8. 移行フェーズ
- フェーズA: core抽出（1〜2日）
- フェーズB: standalone node 実装（0.5〜1日）
- フェーズC: costmap adapter 実装（0.5〜1日）
- フェーズD: 最適化・CI整備・ドキュメント（2〜4日）

## 9. 次のアクション
1. core_types.hpp と core_processor の最小実装を追加し、既存ロジックを移植して単体テストを通す。
2. depth_camera_processor_node を作成し、点群→GridCostMap→publish を確認する。

（終）
