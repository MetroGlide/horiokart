# slam_gnss_2d 開発フェーズ定義

**現在フェーズ: Phase 3**

---

## Phase 1 — オドメトリのみでスキャンを配置してマップを作る

### 実装スコープ

| ファイル                         | 内容                                                   |
| -------------------------------- | ------------------------------------------------------ |
| `data_types.py`                  | ScanData / OdomData / GnssData / PoseNode              |
| `input/base.py`                  | ScanSourceBase / OdomSourceBase / GnssSourceBase (ABC) |
| `input/ros2/ros_adapter.py`      | ROS2ScanSource / ROS2OdomSource / ROS2GnssSource       |
| `pose_graph/base.py`             | PoseGraphBuilderBase (ABC)                             |
| `pose_graph/odom_builder.py`     | OdomOnlyBuilder                                        |
| `map_manager/base.py`            | MapRendererBase (ABC)                                  |
| `map_manager/opencv_renderer.py` | OpenCVRenderer (ray-casting)                           |
| `slam_node.py`                   | ROS2ノード エントリポイント                            |

### 完了条件

- [x] rosbagをリプレイして `/slam_gnss_2d/map` に OccupancyGrid が 1 Hz 以上で配信される
- [x] `/slam_gnss_2d/path` に走行軌跡 (nav_msgs/Path) が配信される
- [x] RViz2 でリアルタイムにマップと軌跡を確認できる
- [x] `python3 -c "from slam_gnss_2d.data_types import ScanData"` がエラーなし

### Phase 2 への引継ぎ条件

- `ScanData.angle_min` は base_link フレーム基準である。アダプター層が保証する不変条件であり、コアロジックはこれを将来にわたって前提としてよい。
- LiDAR 位置オフセット（0.23 m）は Phase 1 で意図的に未補正。レイ原点はロボット中心としている。Phase 2 以降で要検討。

### スタブ（Phase 1では `raise NotImplementedError`）

- `input/ros2/bag_reader.py` — BagScanSource / BagOdomSource / BagGnssSource
- `pose_graph/scan_matching_builder.py` — ScanMatchingBuilder
- `scan_matching/icp_matcher.py` — ICPMatcher
- `gnss/kinematic_aligner.py` — KinematicHeadingAligner
- `gnss/constraint_inserter.py` — GnssConstraintInserter
- `optimizer/gtsam_optimizer.py` — GTSAMOptimizer

---

## Phase 2 — スキャンマッチングを導入する

### 前提条件

- `ScanData.angle_min` は base_link フレーム基準であること（`ROS2ScanSource` が保証）
- `ScanMatchingBuilder` はこの不変条件を引き継ぎ、独自のフレーム変換を行わない

### 実装スコープ

| ファイル                              | 内容                                            |
| ------------------------------------- | ----------------------------------------------- |
| `scan_matching/base.py`               | ScanMatcherBase (ABC) — Phase 1 で既に作成済み  |
| `scan_matching/icp_matcher.py`        | NumPy/SciPy による Point-to-Line ICP            |
| `pose_graph/scan_matching_builder.py` | オドメトリを初期値として ICP で補正するビルダー |

### 完了条件

- [x] ICPMatcher・NDTMatcher が前後フレームの相対変換 (dx, dy, dyaw) を正しく返す
- [x] ScanMatchingBuilder を slam_node.py に差し替えてマップ品質が向上する
- [x] OdomOnlyBuilder との差し替えがパラメータ1行で完結する
- [x] 連続収束失敗時の spiral of doom 対策（streak fallback）が動作する

### Phase 3 への引継ぎ条件

- `PoseGraphBuilderBase.get_edges()` が各実装で正しく返される（`GTSAMOptimizer` の入力として使用する）
- `GraphOptimizerBase.optimize(nodes, edges)` の新シグネチャが ABC・スタブで有効（Phase 2 末に修正済み）
- `LoopClosureBuilder` と `ScanMatchingBuilder` の共通ロジック（ICP マッチング・streak fallback）の共有方針（継承 vs コンポジション）を Phase 3 開始前に設計決定すること
- `optimize()` 呼び出しの主体（Builder 内部 vs `slam_node.py`）を Phase 3 開始前に決定すること

---

## Phase 3 — ループクロージャを導入する

### 実装スコープ

| ファイル                             | 内容                                                |
| ------------------------------------ | --------------------------------------------------- |
| `optimizer/base.py`                  | GraphOptimizerBase (ABC) — Phase 1 で既に作成済み   |
| `optimizer/gtsam_optimizer.py`       | GTSAM LevenbergMarquardt による2Dポーズグラフ最適化 |
| `pose_graph/loop_closure_builder.py` | ループ候補検出 + ループ辺追加ビルダー               |

### 完了条件

- [ ] ループを含む経路でグラフ最適化後にマップが閉合する
- [ ] 最適化後に `rerender_all()` でマップを再描画できる

---

## Phase 4 — GNSSの拘束を挿入する

### 実装スコープ

| ファイル                      | 内容                                           |
| ----------------------------- | ---------------------------------------------- |
| `gnss/aligner_base.py`        | GnssAlignerBase (ABC) — Phase 1 で既に作成済み |
| `gnss/kinematic_aligner.py`   | 運動ベクトルから初期方位を推定する整合実装     |
| `gnss/constraint_inserter.py` | 変換済みGNSS座標を GTSAM GPSFactor として挿入  |
| `input/ros2/bag_reader.py`    | rosbag2 から全センサデータを一括展開           |

### 完了条件

- [ ] KinematicHeadingAligner が GNSS↔SLAM の座標変換を自動推定できる
- [ ] GNSS拘束挿入後のマップが地球座標系（UTM）と整合する
- [ ] 同一rosbagで Phase 3（GNSS無し）と Phase 4（GNSS有り）のマップを比較できる
- [ ] 手動での原点設定・回転入力が不要

---

## フェーズ更新手順

このファイルの「現在フェーズ」を更新し、
`.github/instructions/slam_gnss_2d.instructions.md` の現在フェーズ記載も合わせて更新すること。
