# mg_slam

2D LiDAR + オドメトリ + GNSS を組み合わせた占有格子マップ生成パッケージ。
スキャンマッチング・ループクロージャ・GNSS拘束付き再最適化を段階的に実装した
自作SLAMエンジン（`slam_gnss_2d`）と、slam_toolbox のラッパー（`bringup_slam_toolbox`）の
2種類のバックエンドを提供する。

---

## 目次

- [パッケージ構成](#パッケージ構成)
- [slam\_gnss\_2d アーキテクチャ](#slam_gnss_2d-アーキテクチャ)
- [処理フロー](#処理フロー)
  - [オンラインSLAM](#オンラインslam)
  - [オフラインSLAM（bag再処理）](#オフラインslamtbag再処理)
  - [GNSS 2パス処理](#gnss-2パス処理)
- [アルゴリズム詳細](#アルゴリズム詳細)
  - [スキャンマッチング](#スキャンマッチング)
  - [ループクロージャ](#ループクロージャ)
  - [GNSS拘束](#gnss拘束)
- [実行手順](#実行手順)
  - [オンラインSLAM起動](#オンラインslam起動)
  - [rosbag 収録](#rosbag-収録)
  - [オフライン再処理](#オフライン再処理)
- [パラメータ一覧](#パラメータ一覧)
- [ディレクトリ構成](#ディレクトリ構成)
- [詳細ドキュメント](#詳細ドキュメント)

---

## パッケージ構成

```
mg_slam/
├── launch/
│   ├── bringup_slam_gnss_2d.launch.py   # slam_gnss_2d オンライン起動
│   ├── bringup_slam_toolbox.launch.py   # slam_toolbox ラッパー起動
│   ├── offline_slam_gnss_2d.launch.py   # bag再処理（GNSS付き）
│   └── record_bag.launch.py             # rosbag 収録
├── params/
│   └── slam_gnss_2d.yaml                # 全パラメータのデフォルト値
├── scripts/
│   └── slam_gnss_2d/                    # slam_gnss_2d エンジン本体
└── doc/
    ├── slam_gnss_2d_design.md           # アーキテクチャ設計
    ├── slam_gnss_2d_phases.md           # フェーズ定義
    └── slam_gnss_2d_gnss_algorithm.md   # GNSSアルゴリズム詳細
```

---

## slam_gnss_2d アーキテクチャ

### 層構造

コードは3層に分離されており、コアロジックはROSに依存しない。

```
┌─────────────────────────────────────────────────────────┐
│  Input Layer  — ROSに触れる唯一の層                      │
│                                                         │
│  ScanSourceBase  ─── ROS2ScanSource  (/scan)            │
│  OdomSourceBase  ─── ROS2OdomSource  (/odom)            │
│  GnssSourceBase  ─── ROS2GnssSource  (/gps/fix)         │
│                  ├── BagScanSource   (rosbag2)           │
│                  ├── BagOdomSource   (rosbag2)           │
│                  └── BagGnssSource   (rosbag2)           │
└─────────────────────────────────────────────────────────┘
                           │ dataclass のみ受け渡す
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Core Logic  — ROSに完全非依存                           │
│                                                         │
│  PoseGraphBuilderBase                                   │
│    ├── OdomOnlyBuilder       オドメトリのみ配置          │
│    ├── ScanMatchingBuilder   ICP/NDT補正                 │
│    └── LoopClosureBuilder    ループ検出 + GTSAM最適化    │
│                                                         │
│  GnssAlignerBase                                        │
│    └── KinematicHeadingAligner  座標系変換推定           │
│  GnssConstraintInserter         GNSS拘束生成             │
│  GTSAMOptimizer                 ポーズグラフ最適化       │
│                                                         │
│  MapRendererBase                                        │
│    └── OpenCVRenderer  ray-casting → OccupancyGrid      │
└─────────────────────────────────────────────────────────┘
                           │ list[PoseNode]
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Node Layer  — slam_node.py / slam_offline_node.py       │
│  ROS2パブリッシャ + パラメータ読み込み                    │
└─────────────────────────────────────────────────────────┘
```

### 主要データ型（`data_types.py`）

ROS メッセージ型はInput Layer内でのみ使用し、Core Logic へは以下のdataclassを渡す。

| 型            | フィールド                                           | 用途                    |
| ------------- | ---------------------------------------------------- | ----------------------- |
| `ScanData`    | timestamp, ranges, angle_min, angle_increment        | LiDARスキャン1フレーム  |
| `OdomData`    | timestamp, x, y, yaw                                 | オドメトリ姿勢          |
| `GnssData`    | timestamp, x, y, covariance[2,2]                     | GNSS測位（UTM変換済み） |
| `PoseNode`    | index, timestamp, x, y, yaw, scan                    | ポーズグラフのノード    |
| `PoseEdge`    | from_index, to_index, dx, dy, dyaw, information[3,3] | ノード間拘束            |
| `GnssPrior`   | node_index, x, y, information[2,2]                   | GNSS絶対位置拘束        |
| `MatchResult` | converged, dx, dy, dyaw, information[3,3]            | マッチング結果          |

`ScanData.angle_min` は常に `base_link` フレーム基準。アダプター層が保証する不変条件。

---

## 処理フロー

### オンラインSLAM

```
/scan ─────► ROS2ScanSource ─────► ScanData
                                        │
/odom ─────► ROS2OdomSource ─── get_odom_at(t)
                                        │
                              PoseGraphBuilder.add_scan()
                                        │
                              ┌─────────┴───────────┐
                         新ノードなし           新ノード追加
                         (移動量不足)          PoseNode
                              │                     │
                              │              renderer.add_node()
                              │                     │
                              │              loop_just_closed?
                              │                Yes  │  No
                              │         optimizer   │  インクリメンタル
                              │         .optimize() │  マップ更新
                              │         rerender_all│
                              │                     │
                              └─────────────────────┘
                                        │
                              map_publisher (1 Hz)
                              path_publisher (1 Hz)
```

**キーフレーム採択条件**: `min_translation = 0.3 m` または `min_rotation = 0.1 rad`
いずれかを超えた移動でのみノードを追加する。

### オフラインSLAM（bag再処理）

```
ROSBAG_FILE ─── BagScanSource.step() ───► ScanData (1件ずつ)
             ─── BagOdomSource ──────────► OdomData (補間)
             ─── BagGnssSource.start() ──► GnssData[] (全件プリロード)
                                                │
                                [offline_step_hz Hz でタイマー駆動]
                                                │
                               bag終端到達 → GNSS 2パス処理 → 完了
```

`offline_step_hz: 0.0` で最大速度（1ms タイマー）処理。
RViz2 可視化を行う場合は `offline_step_hz: 5.0` 程度を推奨。

### GNSS 2パス処理

bag全体の読み込みが完了した後に実行される。

```
GnssData[] (全件)
    │
    ├─ [1] KinematicHeadingAligner.estimate_transform(nodes, gnss_list)
    │       └─► (tx, ty, rotation_rad)   GNSS→SLAM座標変換
    │
    ├─ [2] GnssConstraintInserter.build_priors(nodes, gnss_list, transform)
    │       └─► list[GnssPrior]          SLAM系に変換された拘束
    │
    ├─ [3] GTSAMOptimizer.optimize(nodes, edges, gnss_priors=priors)
    │       └─► list[PoseNode]           最適化済みノード
    │
    └─ [4] renderer.rerender_all(updated)
            _rebuild_path(updated)
            map_dirty = True             /map を再配信
```

---

## アルゴリズム詳細

### スキャンマッチング

**ICPMatcher（Point-to-Line ICP）**

オドメトリを初期値として LiDAR スキャン間の相対変換を最適化する。
参照点群は `ScanToScanProvider`（前フレーム1枚）または
`LocalMapProvider`（直近Nノード合成）から供給される。

```
initial_guess (odom差分)
       │
       ├─ src_pts (参照点群) ← ReferenceProvider
       ├─ dst (現フレームScanData)
       │
       ▼
for iter in range(max_iterations):
    1. dst の各点から src_pts への最近傍対応を探す
    2. Point-to-Line残差で変換 (dx, dy, dyaw) を更新
    3. |更新量| < tolerance → 収束
       │
       ▼
MatchResult(converged, dx, dy, dyaw, information)
```

連続 `matcher_max_failure_streak` 回収束失敗でオドメトリ値にフォールバック。

**NDTMatcher**

空間をグリッドセル（`ndt_cell_size`）に分割し、各セルの正規分布と
スキャン点の尤度を最大化する変換を求める。ICPより外れ値に強い。

### ループクロージャ

現在のロボット位置から `loop_closure_search_radius` 以内かつ
`loop_closure_min_node_gap` 以上前のノードをループ候補として検出する。

```
候補ノード検出
    │
    └─ ICP（loop_closure_matcher_type）で検証
           │
           ├─ dyaw > loop_closure_max_dyaw_deg → 棄却
           ├─ 収束失敗 → 棄却（max_failure_streak まで再試行）
           └─ 承認 → PoseEdge(is_loop=True) 追加
                        │
                   optimize_every_n_loops 本溜まったら
                        │
                   GTSAMOptimizer.optimize()
                        │
                   rerender_all() + path 再生成
```

ループ検証に用いる参照点群は `loop_closure_submap_radius` 以内の
複数ノードのスキャンを合成したサブマップ（`LocalMapProvider` 相当）。

### GNSS拘束

詳細は [doc/slam_gnss_2d_gnss_algorithm.md](doc/slam_gnss_2d_gnss_algorithm.md) 参照。

#### KinematicHeadingAligner — 座標系変換の推定

GNSS（UTM座標系）とSLAM（ロボット起動基準ローカル座標系）の間の
剛体変換 `SLAM_xy = R(θ) * GNSS_xy + (tx, ty)` を走行データから自動推定する。

**Step 1 — 回転角 θ の推定**

速度フィルタ（`kinematic_min_speed_ms`）を通過した連続GNSSペアについて
GNSS方位とSLAM方位の差を計算し、**円形平均**で回転角を求める。

```
θ_i = atan2(Δs_y, Δs_x) − atan2(Δg_y, Δg_x)     (各ペアのサンプル)
θ   = atan2( Σ sin(θ_i),  Σ cos(θ_i) )             (円形平均)
```

単純平均を避けるのは ±π 付近のラップアラウンド誤差を防ぐため。

**Step 2 — 平行移動 (tx, ty) の推定**

全GNSS点を回転後、最近傍ノードとの差の平均を取る。

```
(tx, ty) = mean( node_xy − R(θ) * gnss_xy  for all gnss points )
```

#### GnssConstraintInserter — GNSS拘束の生成

推定変換で各GNSS測位をSLAM座標系に変換し、情報行列付きの `GnssPrior` を生成する。
GTSAMへの依存を持たず、optimizer層との入出力インターフェースは `GnssPrior` dataclass のみ。

```
C_slam = R * C_GNSS * R^T     (共分散を回転変換)
I      = C_slam^{-1}          (情報行列)
```

#### GTSAMOptimizer — GNSS拘束付きグラフ最適化

LevenbergMarquardt法によるポーズグラフ最適化。
GNSSは `PriorFactorPose2`（xy拘束のみ、yaw自由）として投入する。

```
I_3x3 = [[I_2x2,  0      ],      yaw 分散 = 1e6 rad²（実質自由）
          [0,      1/σ_θ²]]
```

ファクターグラフの構成:

| ファクター                     | 対象                   | 役割                    |
| ------------------------------ | ---------------------- | ----------------------- |
| `PriorFactorPose2`（アンカー） | `nodes[0]`             | ゲージ自由度の除去      |
| `BetweenFactorPose2`           | 全 `PoseEdge`          | 連続辺 + ループ辺の拘束 |
| `PriorFactorPose2`（GNSS）     | `GnssPrior` 対応ノード | グローバル絶対位置拘束  |

---

## 実行手順

> **前提**: 全コマンドはDockerコンテナ内で実行する。
> コンテナ外から起動する場合は `make shell-develop` でシェルを取得してから実行する。

### オンラインSLAM起動

```bash
# ROS2環境のセットアップ
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# 起動（ループクロージャ + RViz2あり）
ros2 launch mg_slam bringup_slam_gnss_2d.launch.py rviz:=true

# シミュレーション環境の場合
ros2 launch mg_slam bringup_slam_gnss_2d.launch.py simulation:=true rviz:=true
```

**配信トピック**:
- `/slam_gnss_2d/map` — `nav_msgs/OccupancyGrid`（`map_publish_hz` Hz）
- `/slam_gnss_2d/path` — `nav_msgs/Path`

### rosbag 収録

```bash
# GNSS付き収録（GNSS 2パス処理で後処理する場合は /gps/fix が必要）
ros2 launch mg_slam record_bag.launch.py

# 収録先: params/record_topic_list.txt に記載したトピック群
# 出力先: 環境変数 ROSBAG_DIR またはデフォルトディレクトリ
```

### オフライン再処理

#### GNSS なし（ループクロージャのみ）

```bash
ROSBAG_FILE=/path/to/bag \
ros2 launch mg_slam offline_slam_gnss_2d.launch.py rviz:=true
```

#### GNSS あり（2パス処理）

`params/slam_gnss_2d.yaml` の `slam_gnss_2d_offline_node` セクションで
`use_gnss: true` を設定してから起動する。

```bash
ROSBAG_FILE=/path/to/bag \
ros2 launch mg_slam offline_slam_gnss_2d.launch.py rviz:=true
```

処理の流れ:

1. bag 全件をステップ駆動で読み込み、ループクロージャ付きポーズグラフを構築する
2. bag 終端到達後、GNSS 2パス処理（Aligner → Inserter → 再最適化）を自動実行する
3. マップを全ノード再描画して `/slam_gnss_2d/map` を再配信する

ログで進行状況を確認できる:

```
[slam_gnss_2d_offline_node] GNSS align: tx=X.XXm ty=X.XXm rot=X.XXdeg (N fixes, M nodes)
[slam_gnss_2d_offline_node] GNSS inserting N prior constraints
[slam_gnss_2d_offline_node] GNSS phase complete: map re-rendered with GNSS constraints
```

#### パラメータをコマンドラインから上書き

```bash
# 最大速度で処理（RViz2可視化なし）
ROSBAG_FILE=/path/to/bag \
ros2 launch mg_slam offline_slam_gnss_2d.launch.py \
  params_file:=/path/to/custom.yaml
```

---

## パラメータ一覧

`params/slam_gnss_2d.yaml` で設定する。`/**:` セクションはオンライン/オフライン共通、
`slam_gnss_2d_offline_node:` セクションはオフライン専用。

### 共通パラメータ

| パラメータ             | デフォルト        | 説明                              |
| ---------------------- | ----------------- | --------------------------------- |
| `scan_topic`           | `/scan_top_lidar` | LiDARトピック名                   |
| `odom_topic`           | `/odom`           | オドメトリトピック名              |
| `map_resolution`       | `0.05`            | マップ解像度 [m/px]               |
| `map_expansion_margin` | `100.0`           | マップ自動拡張マージン [m]        |
| `min_translation`      | `0.3`             | キーフレーム採択 最小移動量 [m]   |
| `min_rotation`         | `0.1`             | キーフレーム採択 最小回転量 [rad] |
| `map_publish_hz`       | `1.0`             | マップ配信レート [Hz]             |

### ポーズグラフ構築

| パラメータ                   | デフォルト          | 説明                                           |
| ---------------------------- | ------------------- | ---------------------------------------------- |
| `pose_graph_builder`         | `loop_closure`      | `odom_only` / `scan_matching` / `loop_closure` |
| `scan_matcher_type`          | `ndt`               | `icp` / `ndt`                                  |
| `scan_reference`             | `scan_to_local_map` | `scan_to_scan` / `scan_to_local_map`           |
| `matcher_max_failure_streak` | `5`                 | 連続失敗上限（超えるとodomフォールバック）     |

### ICP パラメータ

| パラメータ                    | デフォルト | 説明               |
| ----------------------------- | ---------- | ------------------ |
| `icp_max_iterations`          | `100`      | 最大反復回数       |
| `icp_tolerance`               | `1e-5`     | 収束判定閾値       |
| `icp_max_correspondence_dist` | `1.0`      | 対応点最大距離 [m] |

### NDT パラメータ

| パラメータ      | デフォルト | 説明                   |
| --------------- | ---------- | ---------------------- |
| `ndt_cell_size` | `1.0`      | グリッドセルサイズ [m] |

### ローカルマップパラメータ

| パラメータ         | デフォルト | 説明                                  |
| ------------------ | ---------- | ------------------------------------- |
| `local_map_window` | `30`       | スライディングウィンドウ幅 [ノード数] |
| `local_map_radius` | `30.0`     | 参照点群抽出半径 [m]                  |

### ループクロージャパラメータ

| パラメータ                        | デフォルト | 説明                                 |
| --------------------------------- | ---------- | ------------------------------------ |
| `loop_closure_search_radius`      | `2.0`      | ループ候補検索半径 [m]               |
| `loop_closure_min_node_gap`       | `50`       | ループ候補の最小ノード間隔           |
| `loop_closure_max_failure_streak` | `3`        | ループ検証連続失敗上限               |
| `optimize_every_n_loops`          | `3`        | N本ごとに最適化実行                  |
| `loop_closure_matcher_type`       | `icp`      | ループ検証用マッチャー（`icp` 推奨） |
| `loop_closure_max_dyaw_deg`       | `145.0`    | ループ辺 yaw差 上限 [deg]            |
| `loop_closure_submap_radius`      | `5.0`      | ループ検証サブマップ合成半径 [m]     |

> `loop_closure_matcher_type: ndt` は 180° 対称性による誤検出リスクがあるため非推奨。

### オフライン専用パラメータ

| パラメータ               | デフォルト | 説明                                               |
| ------------------------ | ---------- | -------------------------------------------------- |
| `offline_step_hz`        | `0.0`      | bag処理速度 [Hz]（0以下で最大速度）                |
| `use_gnss`               | `true`     | GNSS 2パス処理を有効にする                         |
| `gnss_topic`             | `/gps/fix` | NavSatFix トピック名                               |
| `gnss_noise_xy_m`        | `3.0`      | position_covariance 不定時のフォールバック精度 [m] |
| `kinematic_min_speed_ms` | `0.5`      | 回転推定に使う最低移動速度 [m/s]                   |

---

## ディレクトリ構成

```
scripts/slam_gnss_2d/
├── data_types.py                       # 全dataclass定義（ROS非依存）
├── config.py                           # SlamConfig frozen dataclass
├── component_factory.py                # PoseGraphBuilder ファクトリ
├── slam_node.py                        # オンラインROS2ノード
├── slam_node_base.py                   # オンライン/オフライン共通基底
├── slam_offline_node.py                # オフラインROS2ノード（bag再処理）
│
├── input/
│   ├── base.py                         # ABC: ScanSourceBase / OdomSourceBase / GnssSourceBase
│   └── ros2/
│       ├── ros_adapter.py              # ROS2ScanSource / ROS2OdomSource / ROS2GnssSource
│       └── bag_reader.py              # BagScanSource / BagOdomSource / BagGnssSource
│
├── pose_graph/
│   ├── base.py                         # PoseGraphBuilderBase (ABC)
│   ├── odom_builder.py                 # OdomOnlyBuilder
│   ├── scan_matching_builder.py        # ScanMatchingBuilder
│   └── loop_closure_builder.py         # LoopClosureBuilder
│
├── scan_matching/
│   ├── base.py                         # ScanMatcherBase (ABC)
│   ├── icp_matcher.py                  # ICPMatcher（Point-to-Line ICP）
│   ├── ndt_matcher.py                  # NDTMatcher
│   └── reference_provider/
│       ├── base.py                     # ReferenceProviderBase (ABC)
│       ├── scan_to_scan.py             # ScanToScanProvider（前フレーム1枚）
│       └── local_map.py                # LocalMapProvider（直近Nノード合成）
│
├── gnss/
│   ├── aligner_base.py                 # GnssAlignerBase (ABC)
│   ├── kinematic_aligner.py            # KinematicHeadingAligner（運動ベクトル整合）
│   └── constraint_inserter.py          # GnssConstraintInserter（GNSS→GnssPrior変換）
│
├── optimizer/
│   ├── base.py                         # GraphOptimizerBase (ABC)
│   └── gtsam_optimizer.py              # GTSAMOptimizer（LevenbergMarquardt）
│
└── map_manager/
    ├── base.py                         # MapRendererBase (ABC)
    └── opencv_renderer.py              # OpenCVRenderer（ray-casting）
```

---

## 詳細ドキュメント

| ドキュメント                                                             | 内容                                      |
| ------------------------------------------------------------------------ | ----------------------------------------- |
| [doc/slam_gnss_2d_design.md](doc/slam_gnss_2d_design.md)                 | 全体アーキテクチャ・ABC定義・データフロー |
| [doc/slam_gnss_2d_phases.md](doc/slam_gnss_2d_phases.md)                 | 開発フェーズ定義・完了条件・引継ぎ条件    |
| [doc/slam_gnss_2d_gnss_algorithm.md](doc/slam_gnss_2d_gnss_algorithm.md) | GNSS拘束の各ステップ詳細アルゴリズム      |
