# slam_gnss_2d 全体アーキテクチャ設計

## 目的

GNSSをポーズグラフの拘束として考慮し、地球座標系と矛盾の少ない2D占有格子マップを
**手作業なし**で生成する。

## 設計の核心：GNSSなしでグラフを先に作る

```
Step 1: OdomOnly/ScanMatching でポーズグラフ構築 → ローカルマップ生成
Step 2: GnssAligner で GNSS軌跡 ↔ SLAM座標系の変換（平行移動 + 回転）を推定
Step 3: 推定変換で全GNSS座標をSLAM座標系に変換 → PriorFactor として一括投入
Step 4: 再最適化 → グローバル一貫性のあるマップ
```

オンライン走行中はローカルSLAMとして動作し、
収集したrosbagをオフラインバッチ処理でGNSS拘束付き再最適化する運用を想定する。

---

## 全体アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────┐
│  Input Layer  (ROSに触れる唯一の層)                               │
│                                                                  │
│  ScanSourceBase ──── ROS2ScanSource  (/scan)                     │
│  OdomSourceBase ──── ROS2OdomSource  (/odom または /odom/gnss)    │
│  GnssSourceBase ──── ROS2GnssSource  (/gps/fix)       [Phase 4] │
│                  ├── BagScanSource   (rosbag2 Scan)    [実装済み] │
│                  ├── BagOdomSource   (rosbag2 Odom)    [実装済み] │
│                  └── BagGnssSource   (rosbag2 GNSS)    [Phase 4] │
└──────────────────────────────────────────────────────────────────┘
                              │ dataclass (ScanData / OdomData / GnssData)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Core Logic  (ROSに完全非依存)                                    │
│                                                                  │
│  PoseGraphBuilderBase                                            │
│    ├── OdomOnlyBuilder           (Phase 1)                       │
│    ├── ScanMatchingBuilder       (Phase 2)                       │
│    └── LoopClosureBuilder        (Phase 3)                       │
│          │ uses ScanMatcherBase                                  │
│          │   ├── ICPMatcher      (Phase 2)                       │
│          │   └── NDTMatcher      (Phase 2)                       │
│          │ uses ReferenceProviderBase                            │
│          │   ├── ScanToScanProvider   (Phase 2)                  │
│          │   └── LocalMapProvider     (Phase 2)                  │
│                                                                  │
│  GnssAlignerBase                 (Phase 4)                       │
│    └── KinematicHeadingAligner                                   │
│                                                                  │
│  GnssConstraintInserter          (Phase 4)                       │
│                                                                  │
│  GraphOptimizerBase              (Phase 3)                       │
│    └── GTSAMOptimizer                                            │
└──────────────────────────────────────────────────────────────────┘
                              │ list[PoseNode]
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Output Layer  (ROSに非依存)                                      │
│                                                                  │
│  MapRendererBase                                                 │
│    └── OpenCVRenderer  (ray-casting → OccupancyGrid)  (Phase 1) │
└──────────────────────────────────────────────────────────────────┘
                              │ nav_msgs/OccupancyGrid
                              ▼
                         slam_node.py  (ROS2 Publisher)
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
    score: float                # マッチングスコア（小さいほど良い）
    information: np.ndarray     # 情報行列 shape (3, 3) — GTSAM BetweenFactor に使用

@dataclass
class PoseEdge:
    i: int                      # エッジ始点ノードインデックス
    j: int                      # エッジ終点ノードインデックス
    dx: float
    dy: float
    dyaw: float
    information: np.ndarray     # shape (3, 3)
    is_loop: bool = False       # ループ辺かどうか
```

---

## 各コンポーネントのABCインターフェース

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

**差し替えポイント**: `topic='/odom'` → `topic='/odom/gnss'` だけで
GNSS補正オドメトリベースに切り替え可能。

### GnssSourceBase

```python
def get_gnss_at(self, timestamp: float) -> Optional[GnssData]
def get_all_gnss(self) -> list[GnssData]      # バッチ最適化用
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
    # 連続辺・ループ辺を含む全拘束を返す（GTSAMOptimizer の入力として使用）
@property
def loop_just_closed(self) -> bool
    # 直前の add_scan() でループが閉合した場合に True を返す
    # 読み取り後に自動リセットされる（slam_node_base.py が rerender_all() の契機に使用）
def reset(self) -> None
```

**差し替えポイント**: OdomOnly → ScanMatching → LoopClosure → SlamToolboxAdapter

### ScanMatcherBase

```python
def match(
    self,
    src_pts: np.ndarray,     # 参照点群 (N, 2)。前ノードのボディフレーム基準。
                             # スキャン1枚分またはローカルマップ集約分のいずれかが渡る。
    dst: ScanData,           # 現フレーム（変換対象スキャン）
    initial_guess: OdomData, # オドメトリ由来の初期推定値
) -> MatchResult
```

**差し替えポイント**: ICPMatcher / NDTMatcher 等に差し替え可能。
src_pts の生成元（1枚スキャン/ローカルマップ）は ReferenceProviderBase が担うため、
マッチャーは参照の由来を意識しない。

### ReferenceProviderBase

```python
def update(self, node: PoseNode) -> None
    # 新しいノードが確定したときに呼ぶ
def get_reference_pts(self) -> Optional[np.ndarray]
    # 最後に確定したノードのボディフレームで参照点群 (N, 2) を返す
    # 未準備時は None を返す
def invalidate_cache(self) -> None
    # グラフ最適化後にキャッシュを無効化する（デフォルト実装: pass）
    # LocalMapProvider はワールド座標キャッシュを持つため override が必要
```

**差し替えポイント**: ScanToScanProvider（前スキャン1枚）/ LocalMapProvider（直近Nノード蓄積）

### GnssAlignerBase

```python
def estimate_transform(
    self,
    nodes: list[PoseNode],
    gnss_list: list[GnssData],
) -> tuple[float, float, float]  # (tx, ty, rotation_rad)
```

### GraphOptimizerBase

```python
def optimize(
    self,
    nodes: list[PoseNode],
    edges: list[PoseEdge],
) -> list[PoseNode]
    # ノードの順序・インデックスを保持して更新後のリストを返す
    # edges: 連続辺・ループ辺を含む全拘束（GTSAM BetweenFactor として使用する）
```

### MapRendererBase

```python
def add_node(self, node: PoseNode) -> bool
    # インクリメンタル更新（オンライン用）
    # True: 描画成功 / False: ロボット位置がマップ範囲外（呼び出し元は rerender_all() を呼ぶ）

def rerender_all(self, nodes: list[PoseNode]) -> None
    # 全ノードから再描画（グラフ最適化後のバッチ更新用）

def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]
    # (data, origin_x, origin_y, resolution)
    # data: int8配列 -1(unknown) / 0(free) / 100(occupied)
```

---

## ディレクトリ構成

```
mg_slam/scripts/slam_gnss_2d/
├── __init__.py
├── data_types.py                    # ScanData / OdomData / GnssData / PoseNode / MatchResult / PoseEdge
├── config.py                        # SlamConfig frozen dataclass（全パラメータのデフォルト値）
├── component_factory.py             # build_pose_graph_builder(config) ファクトリ関数
├── slam_node.py                     # ROS2オンラインノード（SlamNodeBase を継承、IO組み合わせ設定）
├── slam_node_base.py                # ROS2共通基底クラス（Node + ABC）。_declare_params / _build_config
│                                    #   / _on_scan コールバックを実装。オンライン/オフラインで共有
├── slam_offline_node.py             # rosbag2オフラインノード（SlamNodeBase を継承、BagXxxSource を使用）
├── input/
│   ├── __init__.py
│   ├── base.py                      # ScanSourceBase / OdomSourceBase / GnssSourceBase
│   └── ros2/
│       ├── __init__.py
│       ├── ros_adapter.py           # ROS2ScanSource / ROS2OdomSource / ROS2GnssSource
│       └── bag_reader.py            # BagScanSource / BagOdomSource（実装済み）
│                                    #   BagGnssSource（Phase 4 スタブ）
├── pose_graph/
│   ├── __init__.py
│   ├── base.py                      # PoseGraphBuilderBase
│   ├── odom_builder.py              # OdomOnlyBuilder [Phase 1]
│   ├── scan_matching_builder.py     # ScanMatchingBuilder [Phase 2]
│   └── loop_closure_builder.py      # LoopClosureBuilder [Phase 3]
├── scan_matching/
│   ├── __init__.py
│   ├── base.py                      # ScanMatcherBase
│   ├── icp_matcher.py               # ICPMatcher [Phase 2]
│   ├── ndt_matcher.py               # NDTMatcher [Phase 2]
│   └── reference_provider/
│       ├── __init__.py
│       ├── base.py                  # ReferenceProviderBase
│       ├── scan_to_scan.py          # ScanToScanProvider [Phase 2]
│       └── local_map.py             # LocalMapProvider [Phase 2]
├── gnss/
│   ├── __init__.py
│   ├── aligner_base.py              # GnssAlignerBase
│   ├── kinematic_aligner.py         # KinematicHeadingAligner [Phase 4]
│   └── constraint_inserter.py       # GnssConstraintInserter [Phase 4]
├── optimizer/
│   ├── __init__.py
│   ├── base.py                      # GraphOptimizerBase
│   └── gtsam_optimizer.py           # GTSAMOptimizer [Phase 3]
└── map_manager/
    ├── __init__.py
    ├── base.py                      # MapRendererBase
    └── opencv_renderer.py           # OpenCVRenderer [Phase 1]
```

---

## slam_node.py / slam_node_base.py の役割

Phase 2 以降でオンライン/オフライン共通のコアロジックが増加したため、
共通部分を `slam_node_base.py`（`SlamNodeBase(Node, ABC)`）に抽出した。

| ファイル               | 役割                                                                                                                |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `slam_node_base.py`    | ROSパラメータ宣言・`SlamConfig` 生成・`_on_scan()` コールバック・マップ/TF配信・統計ログを実装                      |
| `slam_node.py`         | `SlamNodeBase` を継承し、`_setup_io()` で `ROS2ScanSource` + `ROS2OdomSource` を生成するだけ                        |
| `slam_offline_node.py` | `SlamNodeBase` を継承し、`_setup_io()` で `BagScanSource` + `BagOdomSource` を生成。ステップタイマーで bag を進める |

`component_factory.py` の `build_pose_graph_builder(config)` が、`config.pose_graph_builder` の文字列値に応じて適切なビルダーを組み立てて返す。これにより `slam_node_base.py` がビルダーの具体型に依存しない。

```python
# slam_node.py（オンライン）の実装例
class SlamNode(SlamNodeBase):
    def _setup_io(self) -> None:
        self._scan_source = ROS2ScanSource(self, topic=self._config.scan_topic)
        self._odom_source = ROS2OdomSource(self, topic=self._config.odom_topic)
```

---

## 注意事項

- `input/ros2/` 以外で `import rclpy` を使わない
- 各ABCのメソッドシグネチャを変更する場合は必ずこのドキュメントを先に更新する
- MapRenderer は内部マップに `128=unknown / 255=free / 0=occupied` を使い、
  `to_occupancy_array()` でROSの `-1/0/100` に変換して返す
- OccupancyGrid は 1 Hz タイマーで配信する（スキャン毎の変換・配信はCPU負荷が高い）
