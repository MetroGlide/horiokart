# slam_gnss_2d 全体アーキテクチャ設計

## 目的

GNSSをポーズグラフの拘束として考慮し、地球座標系と矛盾の少ない2D占有格子マップを
**オンライン・オフラインを問わずインクリメンタルに**生成する。

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
                              │ 同期済みの SensorFrame パケットとして渡す
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Core Logic  (ROSに完全非依存)                                    │
│                                                                  │
│  GraphOrchestrator (全体のポーズグラフ更新と最適化トリガー)           │
│    ├── GnssAnchorManager (WGS84 ↔ UTM 変換、原点アンカー管理)     │
│    ├── ISAM2Optimizer (iSAM2を用いた逐次最適化実行)              │
│    └── PoseGraphBuilderBase (ノード・エッジの構築)                │
│          ├── OdomOnlyBuilder                                     │
│          ├── ScanMatchingBuilder                                 │
│          └── LoopClosureBuilder                                  │
│                                                                  │
│  MapRendererBase                                                 │
│    └── OpenCVRenderer  (ray-casting → OccupancyGrid)             │
└──────────────────────────────────────────────────────────────────┘
```

## データコンテナ定義

ROSメッセージ型はInputLayer内でのみ使用する。
コアロジックにはすべて純粋なPythonデータクラスを渡す。
詳細は [data_types.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/core/data_types.py) を参照。

- `ScanData`, `OdomData`, `GnssData`: 各センサの計測データ
- `SensorFrame`: 1時刻に同期されたセンサ群（Orchestratorへの入力単位）
- `PoseNode`, `PoseEdge`, `GnssPrior`: ポーズグラフを構成するノードと制約

## 各コンポーネントとインターフェース

詳細なメソッド定義は各ベースクラスの実装を参照。

- **入力ソース**: [input/base.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/input/base.py)
- **グラフ構築**: [pose_graph/base.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/pose_graph/base.py)
- **スキャンマッチング**: [scan_matching/base.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/scan_matching/base.py)

## 設計上の決定事項

- **I/Oとアルゴリズムの分離**: `SlamNodeBase` で全データを同期して `SensorFrame` を作成し、`GraphOrchestrator` は純粋にデータを受け取って処理する設計とした。
- **インクリメンタルな単一パイプライン**: バッチ最適化とオンライン最適化を分ける複雑なロジックを廃止し、常に `ISAM2` による単一のインクリメンタルオプティマイザが稼働する。GNSSデータは利用可能であれば即座に PriorFactor として投入される。
- **デグレードモードの廃止**: GNSSロスト時の複雑な状態管理を廃止し、取得できたデータのみを愚直にグラフに反映する極めてシンプルな設計とした。

## ノードの役割

- `slam_node.py`: ROS2オンラインノード。各センサのTopicを購読する。
- `slam_offline_node.py`: rosbag2オフラインノード。Bagからデータを読み込み、高速に処理する。
