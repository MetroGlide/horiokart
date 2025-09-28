
# 実装状況チェックリスト — horiokart_obstacle_detector_3d

以下は `doc/implementation_design.md` と `config/default_params.yaml` に記載された設計要件を、`horiokart_obstacle_detector_3d` 以下の実装コード（`src/`, `src_core/`, `include/`）と照合した結果です。

※各項目はチェックボックスで示します。実装済みは [x]、部分実装は [-]、未実装は [ ] とします。各項目に該当する実装ファイルを簡潔に示します。

## 実装済み (Done)
- [x] ROS2 ノード基盤（パラメータ、サブスクライブ、パブリッシュ、TF処理、diagnostics） — `src/obstacle_detector_node.cpp`
- [x] ROI 切り出し（base_link 変換後の座標チェック） — `src/obstacle_detector_node.cpp`
- [x] 前処理: VoxelGrid 相当のダウンサンプリング（独自実装 + PCL フォールバック） — `src_core/cluster_detector.cpp`
- [x] 外れ値除去（Radius outlier） — `src_core/cluster_detector.cpp`
- [x] GridHeightMap（セル集計、中央値/平均/分散計算、時間融合、タイムアウト、有限差分によるスロープ計算、穴の IDW 補間） — `src_core/grid_height_map.cpp`, `include/.../grid_height_map.hpp`
- [x] GroundSeparator（合成スコア、EMA/hysteresis, set/get API） — `include/.../ground_separator.hpp`, `src_core/ground_separator.cpp`
- [x] Euclidean クラスタ抽出＋クラスタ拡張（元点群へマッピング） — `src_core/cluster_detector.cpp`, `include/.../cluster_detector.hpp`
- [x] ScanProjector（障害点群を LaserScan バケットへ投影） — `src_core/scan_projector.cpp`, `include/.../scan_projector.hpp`
- [x] 障害物点群 / LaserScan / visualization markers / diagnostics の publish 実装 — `src/obstacle_detector_node.cpp`
- [x] 色 (RGB->HSV) による vegetation 判定の簡易処理（セル confidence の低下） — `src/obstacle_detector_node.cpp`
- [x] intensity の単純正規化とセル confidence へのブレンド — `src/obstacle_detector_node.cpp`
- [x] dynamic_leaf_size（入力点数に基づく簡易スケーリング）実装 — `src/obstacle_detector_node.cpp`
- [x] 自動パラメータ更新コールバック（on_set_parameters）で主要パラメータを反映 — `src/obstacle_detector_node.cpp`

## 部分実装 / 要改善 (Partial)
- [-] GridHeightMap のパラメータ読み取りが Node 側から YAML の値を使っていない（現在は node 内で固定値で setParameters している） — `src/obstacle_detector_node.cpp` (grid.setParameters(...) がハードコード)
- [-] footprint/traversability 判定関数は実装されているが、パイプライン内で使用され結果を publish していない（checkFootprintTraversable が存在するが呼び出し箇所なし） — `src/obstacle_detector_node.cpp`
- [-] スロープ算出手法: 設計書は PCA も選択肢としているが、実装は有限差分のみ（PCA オプション未実装） — `src_core/grid_height_map.cpp`
- [-] intensity の正規化・補正（距離・受光角補正などの高度な正規化）は簡易実装。LiDAR 固有の補正パイプラインは未実装 — `src/obstacle_detector_node.cpp`
- [-] vegetation / color ベースの高度なフィルタ（マスク割合に基づくセル-level の統合ルールや学習ベース等）は簡易実装に留まる — `src/obstacle_detector_node.cpp`
- [-] パラメータの一部（`conf_min_for_using_interpolated_cell`, `min_obs_for_median` 等）は YAML に定義されているが、Node から GridHeightMap 等へ完全に紐付けられていない箇所がある — `config/default_params.yaml` vs `src/obstacle_detector_node.cpp`

## 未実装 / 欠落 (Not done)
- [ ] odom/cmd_vel を購読して速度に応じた閾値の動的調整（速度依存の slope/height_tol 調整） — 設計で推奨されているが未実装
- [ ] launch ファイル（`launch/`）と実行例（README やサンプル launch）がない（パッケージはあるが launch ディレクトリ未検出）
- [ ] confidence map を専用トピック（PointCloud2 / OccupancyGrid など）として publish する機能（現在は RViz 用 Marker はある）
- [ ] より高度な vegetation / intensity ベースの false-positive 対策（正規化関数、距離補正、ガンマ補正等）は未実装
- [ ] PCA ベースのセル内法線推定（オプション実装）
- [ ] フル end-to-end の統合テスト（rosbag に対する自動化テストスクリプト / CI 用設定） — 単体テストはあるが、rosbag を用いた統合評価の自動化が未整備
- [ ] launch-time または runtime で `GridHeightMap` のパラメータを YAML から確実に反映するコード修正

## 優先度付き作業計画（チェックリスト）
次の項目は短期間で価値が高い順に並べています。各項目は Git で分割コミットしやすい小さなタスクに分割してください。

### 必須（短期: 今週）
- [ ] GridHeightMap のパラメータを `config/default_params.yaml` からノード経由で渡す（node の起動時 / パラメータ更新時に呼ぶ）
	- 影響箇所: `src/obstacle_detector_node.cpp` (grid.setParameters をパラメータ値で呼ぶ)
- [ ] checkFootprintTraversable をパイプライン内で呼び、結果（bool/traversability score）を diagnostics か専用トピックで publish する
	- 影響箇所: `src/obstacle_detector_node.cpp`
- [ ] Node が使用する主要パラメータのドキュメントを `README.md` または `doc/` に短く追記（起動例と合わせて）

### 中期（2〜4 週間）
- [ ] odom または /cmd_vel 購読を実装して速度に応じた slope/height_tol の動的調整を行う（設計書にある速度依存緩和）
	- 影響箇所: `src/obstacle_detector_node.cpp`, `include/...` に必要なら getter/setter の追加
- [ ] confidence map を PointCloud2 (各セルの confidence を横並びで格納) または OccupancyGrid で出力する機能を追加し、RViz 表示とテストを容易にする
	- 影響箇所: `src/obstacle_detector_node.cpp`, `src_core/grid_height_map.cpp`
- [ ] launch ファイルと簡易起動 README を追加（パラメータファイルを指定して起動する例）

### 整備 / 改良（中長期）
- [ ] PCA ベースの法線/スロープ推定をオプションで実装（セル単位のポイント数が充分な場合に選択可能にする）
	- 影響箇所: `src_core/grid_height_map.cpp`（オプションフラグと実装）
- [ ] intensity の距離・角度補正、レンジ依存正規化を実装して intensity ベースの confidence 補正精度を改善
	- 影響箇所: `src/obstacle_detector_node.cpp`
- [ ] vegetation 判定の集約ルールを改善（セル内割合ベースで confidence を下げる/除外する閾値を追加）

### テスト / CI
- [ ] rosbag を用いた統合テスト (ground truth との比較) を作成し、自動化する（目標: 検出精度と処理レイテンシを定期測定）
	- 参考: `test/` に単体テストが存在するため、これを拡張
- [ ] ユニットテスト: GridHeightMap の補間/timeout/EMA と GroundSeparator の hysteresis ロジックの追加ケースを作成

## 要点メモ / 発見事項
- GridHeightMap のパラメータが node から反映されていない点はバグの温床になりやすいため、優先的に修正してください。
- footprint 判定ロジックが既に実装済みであるため（checkFootprintTraversable）、これをフルパイプラインに組み込み結果を publish すれば、走行可否判断実装の早期改善になります。
- 多くの設計要素（補間、EMA、IDW、クラスタリング、scan 投影、diagnostics、マーカー表示）は既に実装済みで、基礎機能は動作する状態です。残りはパラメータ結合と運用周りの整備が中心です。

---
更新日: 2025-09-25
