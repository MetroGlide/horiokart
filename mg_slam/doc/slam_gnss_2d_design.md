# slam_gnss_2d 全体アーキテクチャ設計

## 目的

GNSSをポーズグラフの拘束として考慮し、地球座標系と矛盾の少ない2D占有格子マップを
**オンライン・オフラインを問わずインクリメンタルに**生成する。

## 設計の核心：インクリメンタルなGNSS統合と最適化

GNSSをインクリメンタルに統合する処理は以下の4つのステップで構成されます（アルゴリズムの詳細は [slam_gnss_2d_gnss_algorithm.md](./slam_gnss_2d_gnss_algorithm.md) を参照）。

1. **アンカー設定**: 最初の有効なGNSS fixをUTM平面上の原点とする。
2. **初期方位推定**: アンカーから一定距離移動後、SLAM軌跡とGNSS軌跡の変位ベクトルから初期方位を推定する。
3. **インクリメンタル最適化**: 新ノード追加時に、GNSSデータを絶対位置拘束として逐次挿入する。
4. **マップ再描画**: 最適化による変動量が閾値を超えた時のみ再描画する。

---

## 全体アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────┐
│  Input Layer  (ROSに触れる唯一の層)                               │
│                                                                  │
│  ScanSourceBase ──── ROS2ScanSource  (/scan)                     │
│  OdomSourceBase ──── ROS2OdomSource  (/odom)                     │
│  GnssSourceBase ──── ROS2GnssSource  (/gps/fix / /navpvt)        │
│                  ├── BagScanSource   (rosbag2 Scan)              │
│                  ├── BagOdomSource   (rosbag2 Odom)              │
│                  └── BagGnssSource   (rosbag2 GNSS / NavPVT)     │
└──────────────────────────────────────────────────────────────────┘
                              │ dataclass (ScanData / OdomData / GnssData)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Core Logic  (ROSに完全非依存)                                    │
│                                                                  │
│  GraphOrchestrator (全体のポーズグラフ更新とGNSSの統合・最適化制御)     │
│                                                                  │
│  PoseGraphBuilderBase                                            │
│    ├── OdomOnlyBuilder                                           │
│    ├── ScanMatchingBuilder                                       │
│    └── LoopClosureBuilder                                        │
│          │ uses ScanMatcherBase (ICPMatcher / NDTMatcher / CSMMatcher)│
│          │ uses ReferenceProviderBase (ScanToScan / LocalMap)    │
│                                                                  │
│  GnssAnchoredRunner (アンカー/初期方位推定、インクリメンタル拘束挿入)  │
│    └── GnssAnchorManager (WGS84 ↔ UTM 変換、原点アンカー管理)     │
│                                                                  │
│  IncrementalOptimizerBase                                        │
│    ├── GtsamIncrementalAdapter                                   │
│    └── ISAM2Optimizer (iSAM2を用いた逐次最適化実行)              │
│  GraphOptimizerBase (一括最適化用)                               │
│    └── GTSAMOptimizer (LevenbergMarquardt)                       │
│                                                                  │
│  MapRendererBase                                                 │
│    └── OpenCVRenderer  (ray-casting → OccupancyGrid)             │
└──────────────────────────────────────────────────────────────────┘
                              │ list[PoseNode] / files
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Output / Utility Layer                                          │
│                                                                  │
│  - slam_node.py / slam_offline_node.py (ROS2ノード)              │
│  - SlamDataSaver (gnss_transform.yaml, pose_graph.json保存)      │
│  - slam_gnss_nav_bridge_node.py (ナビゲーション時の座標変換ブリッジ)│
│  - anchor_publisher_node.py (マップ原点の緯度経度配信)             │
└──────────────────────────────────────────────────────────────────┘
```

---

## データコンテナ定義（`data_types.py`）

ROSメッセージ型はInputLayer内でのみ使用する。
コアロジックにはすべて以下のdataclassを渡す。

```python
@dataclass
class ScanData:
    timestamp: float        # ROSタイムスタンプ（秒）
    ranges: np.ndarray      # 距離データ (float32)
    angle_min: float        # base_link フレーム基準（アダプター層が保証する不変条件）
    angle_increment: float
    range_min: float = 0.1
    range_max: float = 30.0

@dataclass
class OdomData:
    timestamp: float
    x: float
    y: float
    yaw: float              # [rad]

@dataclass
class GnssData:
    timestamp: float
    x: float                # 平面直角座標（UTM等に変換後）
    y: float
    covariance: np.ndarray  # shape (2, 2)
    fix_status: int         # 測位ステータス (ublox の gpsFix または NavSatFix.status.status に準拠)

@dataclass
class PoseNode:
    index: int
    timestamp: float
    x: float
    y: float
    yaw: float
    scan: Optional[ScanData] = None

@dataclass
class MatchResult:
    converged: bool
    dx: float                   # 相対移動 x [m]
    dy: float                   # 相対移動 y [m]
    dyaw: float                 # 相対回転 [rad]
    score: float                # マッチングスコア（小さいほど良い）。ICP: 平均点対線残差 [m]、NDT: 平均負対数尤度。収束失敗時は 0.0。
    information: np.ndarray     # 情報行列 shape (3, 3) — GTSAM BetweenFactor に使用

@dataclass
class PoseEdge:
    from_index: int             # エッジ始点ノードインデックス
    to_index: int               # エッジ終点ノードインデックス
    dx: float
    dy: float
    dyaw: float
    information: np.ndarray     # 情報行列 shape (3, 3) — GTSAM BetweenFactor に使用

@dataclass
class GnssPrior:
    node_index: int             # 対応する PoseNode のインデックス
    x: float                    # SLAM 座標系での GNSS x 座標 [m]
    y: float                    # SLAM 座標系での GNSS y 座標 [m]
    information: np.ndarray     # 情報行列 shape (2, 2) — GTSAM PriorFactorPose2 に使用
```

---

## 各コンポーネントのABC・インターフェース

### ScanSourceBase

```python
def set_scan_callback(self, callback: Callable[[ScanData], None]) -> None
def start(self) -> None
def stop(self) -> None
```

### OdomSourceBase

```python
def get_odom_at(self, timestamp: float) -> Optional[OdomData]
def start(self) -> None
def stop(self) -> None
```

### GnssSourceBase

```python
def get_gnss_at(self, timestamp: float) -> Optional[GnssData]
def get_all_gnss(self) -> list[GnssData]
def start(self) -> None
def stop(self) -> None
```

### PoseGraphBuilderBase

```python
def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]
    # 移動量が閾値未満 → None（ノードをスキップ）
    # 新ノード追加 → PoseNode を返す
def get_nodes(self) -> list[PoseNode]
def get_edges(self) -> list[PoseEdge]
    # 連続辺・ループ辺を含む全拘束を返す
@property
def loop_just_closed(self) -> bool
    # 直前の add_scan() でループが閉合した場合に True を返す
def reset(self) -> None
```

### ScanMatcherBase

```python
def match(
    self,
    src_pts: np.ndarray,     # 参照点群 (N, 2)。前ノードのボディフレーム基準。
    dst: ScanData,           # 現フレーム（変換対象スキャン）
    initial_guess: OdomData, # オドメトリ由来の初期推定値
) -> MatchResult
```

### ReferenceProviderBase

```python
def update(self, node: PoseNode) -> None
    # 新しいノードが確定したときに呼ぶ
def get_reference_pts(self) -> Optional[np.ndarray]
    # 最後に確定したノードのボディフレームで参照点群 (N, 2) を返す
def invalidate_cache(self) -> None
    # グラフ最適化後にキャッシュを無効化する
```

### GnssAnchorManager

```python
def try_set_anchor(self, gnss: GnssData, min_status: int) -> bool
    # 最初の有効な GNSS 測位データから基準アンカー（UTM座標）を設定する。
def to_local(self, gnss: GnssData) -> tuple[float, float]
    # 与えられた GNSS 座標を、アンカー位置を原点とするローカル平面座標 [m] に変換する。
@property
def anchor_utm(self) -> Optional[tuple[float, float]]
    # UTM座標系でのアンカー位置 (easting, northing) を返す。
@property
def anchor_latlon(self) -> Optional[tuple[float, float]]
    # WGS84座標系でのアンカー位置 (latitude, longitude) を返す。
```

### GnssAnchoredRunner

```python
def on_gnss(self, gnss: GnssData | None) -> bool
    # GNSS データ受信時のハンドラ。アンカーの設定を試みる。
def process(
    self,
    nodes: list[PoseNode],
    edges: list[PoseEdge],
    latest_node: PoseNode,
    latest_edge: Optional[PoseEdge],
) -> tuple[dict[int, tuple[float, float, float]], bool]
    # 最新のポーズグラフ状態に基づき、初期方位推定（INITIALIZING）または
    # 逐次 Prior 拘束の追加（RUNNING）を処理する。
```

### IncrementalOptimizerBase

```python
def initialize(self, node_index: int, x: float, y: float, theta: float, pos_sigma: float, yaw_sigma: float) -> None
    # 最初のノードの初期値および Prior 拘束（アンカー）を挿入してグラフを初期化する。
def add_initial_estimate(self, node_index: int, x: float, y: float, yaw: float) -> None
    # ノードの初期推定値を挿入する。
def add_between_factor(self, from_index: int, to_index: int, dx: float, dy: float, dyaw: float, information: np.ndarray) -> None
    # ノード間の相対拘束（エッジ）を挿入する。
def add_gnss_prior(self, node_index: int, x: float, y: float, sigma_xy: float, yaw_variance: float) -> None
    # ノードに対する GNSS の絶対位置拘束を挿入する。
def update(self) -> None
    # ファクターグラフを更新・最適化する。
def get_all_poses(self) -> dict[int, tuple[float, float, float]]
    # 最適化された全ノードの最新姿勢 (x, y, yaw) を返す。
```

### LoopClosureBuilder のスコアフィルタ

ループクロージャの false positive を抑制するため、`MatchResult.score` に上限値を設けることができる。

```
ICP score  = mean(|r|)           [m]    小さいほど一致精度が高い
NDT score  = mean(-exponent)     [-]    小さいほど一致精度が高い
```

`loop_closure_max_score: 0.0` で無効（全ループ辺を採用）。正値を設定するとスコアが上限を超えたループ辺は警告ログを出して棄却される。ICPを使う場合は `0.05`〜`0.10` m 程度が目安。

### MapRendererBase

```python
def add_node(self, node: PoseNode) -> bool
    # インクリメンタル更新（オンライン用）
def rerender_all(self, nodes: list[PoseNode]) -> None
    # 全ノードから再描画（グラフ最適化後のバッチ更新用）
def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]
    # (data, origin_x, origin_y, resolution)
```

---

## ディレクトリ構成

```
mg_slam/scripts/slam_gnss_2d/
├── __init__.py
├── data_types.py                    # ScanData / OdomData / GnssData / PoseNode / MatchResult / PoseEdge
├── config.py                        # SlamConfig 定義（パラメータ構造体）
├── component_factory.py             # build_pose_graph_builder(config) ファクトリ関数
├── graph_orchestrator.py            # ポーズグラフ更新とGNSSの統合・最適化トリガーを制御するオーケストレータ
├── slam_data_saver.py               # マップ・ポーズグラフ・変換パラメータ保存用クラス
├── slam_node.py                     # ROS2オンラインノード
├── slam_node_base.py                # ROS2共通基底クラス（Node + ABC）
├── slam_offline_node.py             # rosbag2オフラインノード
├── input/
│   ├── __init__.py
│   ├── base.py                      # ScanSourceBase / OdomSourceBase / GnssSourceBase (ABC)
│   └── ros2/
│       ├── __init__.py
│       ├── ros_adapter.py           # ROS2ScanSource / ROS2OdomSource / ROS2GnssSource
│       └── bag_reader.py            # BagScanSource / BagOdomSource / BagGnssSource
├── pose_graph/
│   ├── __init__.py
│   ├── base.py                      # PoseGraphBuilderBase (ABC)
│   ├── odom_builder.py              # OdomOnlyBuilder
│   ├── scan_matching_builder.py     # ScanMatchingBuilder
│   └── loop_closure_builder.py      # LoopClosureBuilder
├── scan_matching/
│   ├── __init__.py
│   ├── base.py                      # ScanMatcherBase (ABC)
│   ├── icp_matcher.py               # ICPMatcher
│   ├── ndt_matcher.py               # NDTMatcher
│   ├── csm_matcher.py               # CSMMatcher
│   └── reference_provider/
│       ├── __init__.py
│       ├── base.py                  # ReferenceProviderBase (ABC)
│       ├── scan_to_scan.py          # ScanToScanProvider
│       └── local_map.py             # LocalMapProvider
├── gnss/
│   ├── __init__.py
│   ├── anchor_manager.py            # GnssAnchorManager (アンカー/座標変換管理)
│   └── gnss_anchored_runner.py      # GnssAnchoredRunner (初期方位推定/Prior拘束制御)
├── optimizer/
│   ├── __init__.py
│   ├── base.py                      # GraphOptimizerBase / IncrementalOptimizerBase (ABC)
│   ├── gtsam_optimizer.py           # GTSAMOptimizer (一括最適化用)
│   ├── isam2_optimizer.py           # ISAM2Optimizer (逐次最適化用)
│   └── gtsam_incremental_adapter.py # GTSAMを用いて逐次最適化を行うためのアダプター
└── map_manager/
    ├── __init__.py
    ├── base.py                      # MapRendererBase (ABC)
    └── opencv_renderer.py           # OpenCVRenderer
```

---

## 設計上の決定事項

開発フェーズ間で確定した主要な設計決定と背景をまとめる。

### ScanData.angle_min の不変条件

`ScanData.angle_min` は `base_link` フレーム基準。
`ROS2ScanSource` がアダプター層として保証する不変条件であり、
コアロジック（`pose_graph/` / `scan_matching/`）はこれを将来にわたって前提としてよい。

### LiDAR 位置オフセット

LiDAR の取付位置オフセット（0.23 m）は意図的に未補正。
レイキャスティングの原点はロボット中心としている。

### LoopClosureBuilder と ScanMatchingBuilder の共通ロジック共有方針

ICP マッチング・streak fallback の共通ロジックは**コンポジション**で共有する（継承ではない）。
`LoopClosureBuilder` は内部に `ScanMatcherBase` インスタンスを持つ設計。

### optimize() とオプティマイザの共有設計

`GraphOrchestrator` は、`LoopClosureBuilder`（ループクロージャ）と `GnssAnchoredRunner`（GNSS統合）が同じオプティマイザインスタンスを共有するよう配線（`_wire_shared_optimizer`）を行います。これにより、GNSS拘束で補正されたポーズ情報がループクロージャ検出に反映され、またループが閉じた際の最適化がGNSS拘束を維持したまま実行されます。

### PoseGraphBuilderBase.get_edges() の責務

`get_edges()` は連続辺・ループ辺を含む全拘束を返す。
オプティマイザはこれを `BetweenFactorPose2` ファクターとして使用する。

### BagGnssSource での UTM 変換責務

lat/lon → UTM 変換は `BagGnssSource.start()` 内部で行う。
`pyproj.Transformer` を使い、最初の fix から UTM zone を自動検出する。
コアロジック（`gnss/` 以下）は変換済みのデカルト座標のみを受け取る。

### インクリメンタルGNSS統合フロー

オンライン走行時・オフラインbag再生時を問わず、`GraphOrchestrator` がスキャン入力ごとにGNSS拘束処理をトリガーします。
処理アルゴリズムの4ステップ（アンカー設定、初期方位推定、拘束の逐次追加、最適化と再描画）の詳細な数学的背景や設定パラメータについては、[slam_gnss_2d_gnss_algorithm.md](./slam_gnss_2d_gnss_algorithm.md) を参照してください。

---

## slam_node.py / slam_node_base.py の役割

オンライン/オフライン共通のコアロジックを `slam_node_base.py`（`SlamNodeBase(Node, ABC)`）に抽出した。

| ファイル | 役割 |
| --- | --- |
| `slam_node_base.py` | ROSパラメータ宣言・`SlamConfig` 生成・`_on_scan()` コールバック・マップ/TF配信・統計ログを実装 |
| `slam_node.py` | `SlamNodeBase` を継承し、`_setup_io()` で `ROS2ScanSource` + `ROS2OdomSource` + `ROS2GnssSource` (GNSS有効時) を生成 |
| `slam_offline_node.py` | `SlamNodeBase` を継承し、`_setup_io()` で `BagScanSource` + `BagOdomSource` + `BagGnssSource` (GNSS有効時) を生成。ステップタイマーで bag を進め、完了時に `SlamDataSaver` でデータを保存する。 |

`component_factory.py` の `build_pose_graph_builder(config)` が、設定に応じて適切なビルダーを組み立てて返す。

`build_gnss_source(config, bag_path)` が `config.gnss.source` に応じて GNSS データソースを選択する。

| `gnss.source` 値 | 実装クラス | 共分散の出所 |
| --- | --- | --- |
| `navsat_fix` | `BagGnssSource` | `NavSatFix.position_covariance[0,1,3,4]`。`COVARIANCE_TYPE_UNKNOWN` の場合は `gnss.validation.max_sigma_m` 等に準拠する。 |
| `navpvt` | `BagNavPVTSource` | `NavPVT.h_acc` (mm) × `navpvt_hacc_scale` → 等方性 2×2 共分散行列。 |

---

## 注意事項

- `input/ros2/` 以外で `import rclpy` を使わない
- 各ABCのメソッドシグネチャを変更する場合は必ずこのドキュメントを先に更新する
- MapRenderer は内部マップに `128=unknown / 255=free / 0=occupied` を使い、
  `to_occupancy_array()` でROSの `-1/0/100` に変換して返す
- OccupancyGrid は 1 Hz タイマーで配信する（スキャン毎の変換・配信はCPU負荷が高い）
