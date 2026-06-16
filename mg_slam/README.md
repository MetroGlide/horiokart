# mg_slam

2D LiDAR + オドメトリ + GNSS を組み合わせた占有格子マップ生成パッケージ。
スキャンマッチング・ループクロージャ・インクリメンタルGNSS拘束付き最適化を実装した
自作SLAMエンジン（`slam_gnss_2d`）と、slam_toolbox のラッパー（`bringup_slam_toolbox`）の
2種類のバックエンドを提供する。

---

## 目次

- [パッケージ構成](#パッケージ構成)
- [slam\_gnss\_2d アーキテクチャ](#slam_gnss_2d-アーキテクチャ)
- [処理フロー](#処理フロー)
  - [オンライン/オフライン共通SLAMフロー](#オンラインオフライン共通slamフロー)
  - [インクリメンタルGNSS統合](#インクリメンタルgnss統合)
- [アルゴリズム詳細](#アルゴリズム詳細)
  - [スキャンマッチング](#スキャンマッチング)
  - [ループクロージャ](#ループクロージャ)
  - [GNSS拘束](#gnss拘束)
- [実行手順](#実行手順)
  - [オンラインSLAM起動](#オンラインslam起動)
  - [rosbag 収録](#rosbag-収録)
  - [オフライン再処理](#オフライン再処理)
  - [マップ・パラメータの保存](#マップパラメータの保存)
  - [ナビゲーション時の連携（SlamGnssNavBridge）](#ナビゲーション時の連携slamgnssnavbridge)
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
│   ├── slam_gnss_2d/                    # slam_gnss_2d エンジンコアロジック
│   ├── slam_gnss_nav_bridge_node.py     # ナビゲーション連携用座標変換ブリッジノード
│   └── anchor_publisher_node.py         # マップ基準アンカー配信ノード
└── doc/
    ├── slam_gnss_2d_design.md           # アーキテクチャ設計・設計決定事項
    └── slam_gnss_2d_gnss_algorithm.md   # GNSSアルゴリズム詳細
```

---

## slam_gnss_2d アーキテクチャ

### 層構造

コードはモジュール化されており、コアロジックはROSに依存しない。

```
┌─────────────────────────────────────────────────────────┐
│  Input Layer  — ROSに触れる唯一の層                      │
│                                                         │
│  ScanSourceBase  ─── ROS2ScanSource  (/scan)            │
│  OdomSourceBase  ─── ROS2OdomSource  (/odom)            │
│  GnssSourceBase  ─── ROS2GnssSource  (/gps/fix / /navpvt)│
│                  ├── BagScanSource   (rosbag2)           │
│                  ├── BagOdomSource   (rosbag2)           │
│                  └── BagGnssSource   (rosbag2)           │
└─────────────────────────────────────────────────────────┘
                           │ dataclass のみ受け渡す
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Core Logic  — ROSに完全非依存                           │
│                                                         │
│  GraphOrchestrator   ─── 全体のポーズグラフ更新とGNSSの統合を統括│
│  PoseGraphBuilderBase                                   │
│    ├── OdomOnlyBuilder       オドメトリのみ配置          │
│    ├── ScanMatchingBuilder   ICP/NDT補正                 │
│    └── LoopClosureBuilder    ループ検出 + GTSAM最適化    │
│                                                         │
│  GnssAnchoredRunner  ─── アンカー管理と変位ベース初期方位推定    │
│    └── GnssAnchorManager  基準アンカー(UTM)管理          │
│                                                         │
│  IncrementalOptimizerBase                               │
│    ├── GtsamIncrementalAdapter                          │
│    └── ISAM2Optimizer        iSAM2を用いた逐次最適化    │
│  GraphOptimizerBase                                     │
│    └── GTSAMOptimizer        一括最適化                  │
│                                                         │
│  MapRendererBase                                        │
│    └── OpenCVRenderer        ray-casting → OccupancyGrid │
└─────────────────────────────────────────────────────────┘
                           │ list[PoseNode] / files
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Node / Output Layer                                    │
│  - slam_node.py / slam_offline_node.py (ROS2ノード)      │
│  - SlamDataSaver (gnss_transform.yaml, pose_graph.json) │
│  - slam_gnss_nav_bridge_node.py (変換パラメータの読込)     │
└─────────────────────────────────────────────────────────┘
```

### 主要データ型（`data_types.py`）

ROS メッセージ型はInput Layer内でのみ使用し、Core Logic へは以下のdataclassを渡す。

| 型            | フィールド                                           | 用途                    |
| ------------- | ---------------------------------------------------- | ----------------------- |
| `ScanData`    | timestamp, ranges, angle_min, angle_increment        | LiDARスキャン1フレーム  |
| `OdomData`    | timestamp, x, y, yaw                                 | オドメトリ姿勢          |
| `GnssData`    | timestamp, x, y, covariance[2,2], fix_status         | GNSS測位（UTM変換済み） |
| `PoseNode`    | index, timestamp, x, y, yaw, scan                    | ポーズグラフのノード    |
| `PoseEdge`    | from_index, to_index, dx, dy, dyaw, information[3,3] | ノード間拘束            |
| `GnssPrior`   | node_index, x, y, information[2,2]                   | GNSS絶対位置拘束        |
| `MatchResult` | converged, dx, dy, dyaw, information[3,3]            | マッチング結果          |

`ScanData.angle_min` は常に `base_link` フレーム基準。アダプター層が保証する不変条件。

---

## 処理フロー

### オンライン/オフライン共通SLAMフロー

スキャンが入力されるたびに `GraphOrchestrator` を通じて以下の処理が行われる。

```
/scan ────► ROS2ScanSource ───► ScanData ──┐
                                           ├─► GraphOrchestrator.process_scan()
/odom ────► ROS2OdomSource ───► OdomData ──┘         │
                                           PoseGraphBuilder.add_scan()
                                                     │
                                           ┌─────────┴───────────┐
                                      新ノードなし          新ノード追加
                                      (移動量不足)          PoseNode
                                           │                     │
                                           │             [GNSS有効かつ逐次最適化時]
                                           │             GnssAnchoredRunner.process()
                                           │             (Anchor/Heading推定後、Prior挿入)
                                           │                     │
                                           │             loop_just_closed?
                                           │               Yes   │   No
                                           │          optimizer  │  インクリメンタル
                                           │         .update()   │  マップ更新
                                           │       (GTSAM/iSAM2) │
                                           │         map_dirty   │
                                           │         (要再描画)   │
                                           │                     │
                                           └─────────────────────┘
                                                     │
                                           map_publisher (1 Hz)
                                           path_publisher (1 Hz)
```

**キーフレーム採択条件**: `min_translation = 1.0 m` または `min_rotation = 0.1 rad` （`slam_gnss_2d.yaml` で変更可能）
いずれかを超えた移動でのみノードを追加する。

### インクリメンタルGNSS統合

GNSS拘束は、従来のような2パス（オフライン一括）処理ではなく、オンライン・オフラインを問わず以下のステップで逐次（インクリメンタル）に処理される。

1. **アンカー設定 (`GnssAnchorManager`):**
   最初の有効なGNSS fix（RTK-Fixed / RTK-Float 等、`anchor_min_fix_status` 以上）を基準アンカーとして採用し、そのUTM座標を保持する。以後のGNSS座標はこのアンカーを原点とするローカル平面座標に変換される。
2. **初期方位の推定とグラフ初期化 (`GnssAnchoredRunner`):**
   ロボットが基準アンカーから `init_distance_m` 以上移動するのを待ち、SLAMのローカル変位とGNSSのローカル座標変位の方向から初期方位（`theta0`）を算出する。この方位に基づいてオプティマイザ（iSAM2等）の最初のノードを初期化し、ポーズグラフ全体をアライメントする。
3. **インクリメンタルな拘束追加:**
   新ノードが追加されるたびに、対応する時刻のGNSS測位データを取得。測位精度（`sigma`）が `gnss_max_sigma_m` 以下の良好なデータである場合にのみ、オプティマイザに絶対位置拘束（PriorFactor）を逐次挿入し更新する。
4. **マップの再描画判定:**
   最適化によってロボット位置が前回レンダリング時より `gnss_rerender_threshold_m` 以上変動した場合にのみ、マップ全体の再描画をトリガーする。これにより不要な再描画コストを抑制しつつ、一貫性のあるマップを維持する。mSource ─── get_odom_at(t)
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

### GNSS拘束

詳細は [doc/slam_gnss_2d_gnss_algorithm.md](doc/slam_gnss_2d_gnss_algorithm.md) 参照。

#### 1. アンカー設定 (`GnssAnchorManager`)

最初の有効な GNSS fix（RTK-Fixed / RTK-Float 等、`anchor_min_fix_status` 以上）を基準アンカーとして採用し、その時の経度から自動的に UTM zone を決定して UTM 座標 (`anchor_utm_easting`, `anchor_utm_northing`) を取得します。
以後はこのアンカー位置を基準としたローカル平面座標に GNSS 測位を変換して処理します。

#### 2. 初期方位の推定とグラフ初期化 (`GnssAnchoredRunner`)

起動直後はロボットの絶対方位（ローカル座標系とGNSS座標系の回転ずれ）が不明なため、グラフは `INITIALIZING` 状態となります。
ロボットが基準アンカーから `init_distance_m` (デフォルト: 2.0 m) 以上移動した時点で、これまでの SLAM の軌跡と GNSS 軌跡の移動方向を比較し、初期方位 `theta0` および回転オフセット `init_rotation` を算出します。これにより最初のノード位置および向きを初期化し、グラフ全体の絶対方位を確定させます。

#### 3. インクリメンタルな Prior 拘束の追加

状態が `RUNNING` に移行した後は、LiDAR キーフレーム（ノード）が追加されるたびに、最もタイムスタンプの近い GNSS 測位データを参照します。
測位の共分散から導出される標準偏差 $\sigma_{xy}$ が `gnss_max_sigma_m` (デフォルト: 2.0 m) 以下の場合にのみ、オプティマイザに対して絶対位置の Prior 拘束 (`add_gnss_prior`) を追加します。yaw 方向は観測できないため、大きな分散を設定することで位置のみを拘束します。

#### 4. iSAM2 / GTSAM による逐次最適化

オプティマイザ（`backend: isam2` または `gtsam`）によって、オドメトリ拘束、スキャンマッチング拘束、ループクロージャ拘束、および GNSS の絶対位置拘束を考慮したファクターグラフの最適化が逐次実行されます。
最適化による姿勢の変動量が `gnss_rerender_threshold_m` 以上となった場合にのみ、マップの再描画を行います。

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
# トピックを収録 (GNSS 連携を行う場合は /gps/fix または /navpvt が必要)
ros2 launch mg_slam record_bag.launch.py
```

### オフライン再処理

`params/slam_gnss_2d.yaml` において `gnss.enabled: true` が設定されていると、bag 再生中にオンライン同様のインクリメンタル GNSS 統合が適用されます。

```bash
ROSBAG_FILE=/path/to/bag \
ros2 launch mg_slam offline_slam_gnss_2d.launch.py rviz:=true
```

bag の再生終了時、最終のポーズグラフ最適化およびマップレンダリングが実行され、確定したマップおよびポーズグラフ、座標変換パラメータが自動保存されます。

### マップ・パラメータの保存

オンライン SLAM 実行中、またはオフライン再処理が終了した時点で、以下のサービスコールを実行することでマップと関連データを保存できます。

```bash
ros2 service call /slam_gnss_2d/save_slam_map mg_msgs/srv/SaveSlamMap "{map_dir: '/app/maps'}"
```

このサービスコールによって、指定ディレクトリに以下のファイル群が出力されます。
- `map.pgm` / `map.yaml` : 標準的な 2D 占有格子マップ（Map Server 用）
- `gnss_transform.yaml` : マップ座標系と地球座標系 (UTM/WGS84) の座標変換パラメータ (基準アンカー、初期回転角)
- `pose_graph.json` : 最適化されたポーズグラフ履歴（ノード、エッジ、および座標データ）

### ナビゲーション時の連携（SlamGnssNavBridge）

マップ生成完了後の自律移動（Navigation2）フェーズでは、`slam_gnss_nav_bridge_node` を起動します。
本ノードは保存された `gnss_transform.yaml` を読み込み、ロボットが受信する生の GNSS 座標（`/gps/fix` または `/navpvt`）をマップ座標系に変換した上で、`/odom/gps` トピック (`nav_msgs/Odometry`) として配信します。これが `robot_localization` などのカルマンフィルタによるオドメトリフュージョンに入力されることで、マップ座標系と一致したグローバルナビゲーションが実現します。

---

## パラメータ一覧

`params/slam_gnss_2d.yaml` で設定する。

### 共通パラメータ

| パラメータ | デフォルト | 説明 |
| --- | --- | --- |
| `topics.scan` | `/scan_top_lidar` | LiDARトピック名 |
| `topics.odom` | `/odom` | オドメトリトピック名 |
| `map.resolution` | `0.05` | マップ解像度 [m/px] |
| `map.expansion_margin` | `100.0` | マップ自動拡張マージン [m] |
| `map.publish_hz` | `1.0` | マップ配信レート [Hz] |
| `keyframe.min_translation` | `1.0` | キーフレーム採択 最小移動量 [m] |
| `keyframe.min_rotation` | `0.1` | キーフレーム採択 最小回転量 [rad] |

### スキャンマッチング / ループクロージャ / 最適化

| パラメータ | デフォルト | 説明 |
| --- | --- | --- |
| `scan_matching.enabled` | `true` | スキャンマッチングの有効化 |
| `scan_matching.type` | `ndt` | マッチング種別 (`icp` / `ndt` / `csm`) |
| `scan_matching.reference` | `scan_to_local_map` | 参照点群 (`scan_to_scan` / `scan_to_local_map`) |
| `loop_closure.enabled` | `true` | ループクロージャ検出の有効化 |
| `loop_closure.search_radius` | `2.0` | ループ候補検索半径 [m] |
| `loop_closure.min_node_gap` | `50` | ループ候補の最小ノード間隔 |
| `optimization.backend` | `gtsam` | 最適化バックエンド (`gtsam` / `isam2`) |
| `optimization.incremental` | `true` | インクリメンタル最適化の有効化 |
| `optimization.optimize_every_n_loops`| `3` | 最適化を実行するループ検出間隔 |
| `optimization.rerender_threshold_m` | `0.1` | マップ再描画をトリガーする位置変動閾値 [m] |

### GNSS パラメータ (`gnss`)

| パラメータ | デフォルト | 説明 |
| --- | --- | --- |
| `gnss.enabled` | `true` | GNSS 拘束の有効化 |
| `gnss.source` | `navpvt` | GNSS データソース (`navsat_fix` / `navpvt`) |
| `gnss.topics.fix` | `/gps/fix` | `NavSatFix` トピック名 |
| `gnss.topics.navpvt` | `/navpvt` | `NavPVT` トピック名 |
| `gnss.validation.max_sigma_m` | `2.0` | 拘束追加を許容する最大位置標準偏差 $\sigma$ [m] |
| `gnss.anchor.init_distance_m` | `2.0` | 初期方位算出に必要な最小移動距離 [m] |
| `gnss.sigma.fix_m` | `0.02` | RTK-Fixed 時の位置標準偏差 $\sigma$ [m]（フォールバック用） |
| `gnss.sigma.float_m` | `0.5` | RTK-Float 時の位置標準偏差 $\sigma$ [m]（フォールバック用） |

---

## ディレクトリ構成

```
scripts/slam_gnss_2d/
├── __init__.py
├── data_types.py                       # 全dataclass定義（ROS非依存）
├── config.py                           # SlamConfig 定義（パラメータ構造体）
├── component_factory.py                # コンポーネント生成ファクトリ
├── graph_orchestrator.py               # ポーズグラフ更新とGNSSの統合制御
├── slam_data_saver.py                  # マップ、ポーズグラフ、変換パラメータ保存
├── slam_node.py                        # オンラインROS2ノード
├── slam_node_base.py                   # オンライン/オフライン共通基底ノード
├── slam_offline_node.py                # オフラインROS2ノード（bag再処理）
│
├── input/
│   ├── base.py                         # 入力データソース基底クラス (ABC)
│   └── ros2/
│       ├── ros_adapter.py              # ROS2トピック入力用ソース群
│       └── bag_reader.py               # rosbag2入力用ソース群
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
│   ├── csm_matcher.py                  # CSMMatcher (Correlation Scan Matcher)
│   └── reference_provider/
│       ├── base.py                     # ReferenceProviderBase (ABC)
│       ├── scan_to_scan.py             # ScanToScanProvider（前フレーム1枚）
│       └── local_map.py                # LocalMapProvider（直近Nノード合成）
│
├── gnss/
│   ├── __init__.py
│   ├── anchor_manager.py               # 基準アンカー管理 (WGS84 ↔ UTM)
│   └── gnss_anchored_runner.py         # 初期方位推定・逐次 Prior 拘束追加制御
│
├── optimizer/
│   ├── __init__.py
│   ├── base.py                         # GraphOptimizerBase / IncrementalOptimizerBase (ABC)
│   ├── gtsam_optimizer.py              # GTSAMOptimizer（一括最適化用）
│   ├── isam2_optimizer.py              # ISAM2Optimizer（逐次最適化用）
│   └── gtsam_incremental_adapter.py    # GTSAMを用いた逐次最適化アダプター
│
└── map_manager/
    ├── base.py                         # MapRendererBase (ABC)
    └── opencv_renderer.py              # OpenCVRenderer（OccupancyGridレンダリング）
```

---

## 詳細ドキュメント

| ドキュメント | 内容 |
| --- | --- |
| [doc/slam_gnss_2d_design.md](doc/slam_gnss_2d_design.md) | 全体アーキテクチャ・ABC定義・データフロー・設計決定事項 |
| [doc/slam_gnss_2d_gnss_algorithm.md](doc/slam_gnss_2d_gnss_algorithm.md) | GNSS 拘束の各ステップ詳細アルゴリズム |
